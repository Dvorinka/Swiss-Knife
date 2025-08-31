// Includes
#include <Arduino.h>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_arduino_version.h>
#include <esp32-hal-ledc.h>
#endif
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <SD_MMC.h>
#include <USB.h>
#include <USBMSC.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Preferences.h>
#include <vector>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

/**********************
 * Forward decls      *
 **********************/
void setStatusLED(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 31);

/**********************
 * Configurable pins  *
 **********************/
// Button (default BOOT)
#ifndef PIN_BUTTON
#define PIN_BUTTON 0
#endif

// ST7735 0.96" 80x160 SPI pins (adjust to your board)
// If you don't know yet, leave defaults and we can update after confirming pin map.
#ifndef PIN_TFT_CS
#define PIN_TFT_CS   4    // Chip Select (LilyGO T-Dongle S3)
#endif
#ifndef PIN_TFT_DC
#define PIN_TFT_DC   2    // Data/Command
#endif
#ifndef PIN_TFT_RST
#define PIN_TFT_RST  1    // Reset (can be -1 if tied to EN)
#endif
#ifndef PIN_TFT_BL
#define PIN_TFT_BL   38   // Backlight (active LOW per board config)
#endif
#ifndef PIN_TFT_SCLK
#define PIN_TFT_SCLK 5    // SPI SCK
#endif
#ifndef PIN_TFT_MOSI
#define PIN_TFT_MOSI 3    // SPI MOSI (display is write-only)
#endif
#ifndef PIN_TFT_MISO
#define PIN_TFT_MISO -1   // Not used
#endif

// APA102 RGB LED (single LED)
#ifndef PIN_DOTSTAR_DATA
#define PIN_DOTSTAR_DATA 40
#endif
#ifndef PIN_DOTSTAR_CLK
#define PIN_DOTSTAR_CLK  39
#endif

/**********************
 * Wi-Fi configuration *
 **********************/
static const char* AP_SSID = "Tdongle-SwissKnife";
static const char* AP_PASS = "tdongle123"; // 8+ chars

// Connection timeouts
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

/**********************
 * Globals            *
 **********************/
WebServer server(80);

// ST7735: 80x160 panel (rotate as needed). Using INITR_MINI160x80.
Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_MOSI, PIN_TFT_SCLK, PIN_TFT_RST);
bool tftReady = false;
bool backlightOn = true;
uint8_t backlightLevel = 255; // 0..255, active-low output

// Preferences for storing Wi-Fi credentials
Preferences prefs;
String staSsid;
String staPass;
uint16_t lastOpenPorts = 0;
uint16_t lastDiscoverHosts = 0;

// LED rainbow mode
bool ledRainbow = false;
uint16_t ledHue = 0; // 0..359
uint32_t lastRainbowMs = 0;

// Async discovery task state
bool discoverRunning = false;
String discBase;
int discStart = 1;
int discCount = 0;
int discIndex = 0; // 0..discCount-1
std::vector<uint16_t> discPorts;
std::vector<String> discFound;
unsigned long lastDiscoverStepMs = 0;
// Optional per-host details (JSON strings)
std::vector<String> discInfo;
// mDNS mapping
bool mdnsStarted = false;
struct MdnsEntry { String ip; String name; String services; };
std::vector<MdnsEntry> mdnsMap;
// MSC (USB Mass Storage) placeholder state
bool mscEnabled = false;
bool mscMounted = false;
uint64_t mscTfBytes = 0;
bool mscArmed = false; // persistable flag: auto-enable MSC on boot
bool mscAutoArm = false; // auto-arm MSC on next boot
// SDMMC TF pin map for LilyGO T-Dongle S3 (per board docs)
constexpr int TF_CLK = 12;
constexpr int TF_CMD = 16;
constexpr int TF_D0  = 14;
constexpr int TF_D1  = 17;
constexpr int TF_D2  = 21;
constexpr int TF_D3  = 18;

static bool tfMount() {
  if (mscMounted) return true;
  SD_MMC.setPins(TF_CLK, TF_CMD, TF_D0, TF_D1, TF_D2, TF_D3);
  if (!SD_MMC.begin("/sdcard", true, true)) { // 4-bit, high speed
    mscMounted = false; mscTfBytes = 0; return false;
  }
  mscMounted = true;
  mscTfBytes = SD_MMC.totalBytes();
  return true;
}

static void tfUnmount() {
  if (mscMounted) { SD_MMC.end(); mscMounted = false; }
}

// ---------------- TF Passthrough MSC using SDMMC host ----------------
USBMSC usbMsc;
static sdmmc_card_t* g_card = nullptr;
static uint16_t mscBlockSize = 512;
static uint32_t mscBlockCount = 0;

static int32_t msc_read_cb_fn(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  if (!g_card) return -1;
  uint8_t* out = (uint8_t*)buffer;
  uint32_t remaining = bufsize;
  uint32_t curLba = lba;
  uint32_t curOff = offset;
  static uint8_t scratch[512];
  while (remaining) {
    if (curOff == 0 && remaining >= 512) {
      uint32_t sectors = remaining / 512;
      esp_err_t err = sdmmc_read_sectors(g_card, out, curLba, sectors);
      if (err != ESP_OK) return -1;
      uint32_t bytes = sectors * 512;
      out += bytes; remaining -= bytes; curLba += sectors;
    } else {
      // partial sector
      esp_err_t err = sdmmc_read_sectors(g_card, scratch, curLba, 1);
      if (err != ESP_OK) return -1;
      uint32_t n = 512 - curOff;
      if (n > remaining) n = remaining;
      memcpy(out, scratch + curOff, n);
      out += n; remaining -= n; curLba += 1; curOff = 0;
    }
  }
  return (int32_t)bufsize;
}

static int32_t msc_write_cb_fn(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  if (!g_card) return -1;
  uint8_t* in = (uint8_t*)buffer;
  uint32_t remaining = bufsize;
  uint32_t curLba = lba;
  uint32_t curOff = offset;
  static uint8_t scratch[512];
  while (remaining) {
    if (curOff == 0 && remaining >= 512) {
      uint32_t sectors = remaining / 512;
      esp_err_t err = sdmmc_write_sectors(g_card, in, curLba, sectors);
      if (err != ESP_OK) return -1;
      uint32_t bytes = sectors * 512;
      in += bytes; remaining -= bytes; curLba += sectors;
    } else {
      // read-modify-write for partial sector
      esp_err_t err = sdmmc_read_sectors(g_card, scratch, curLba, 1);
      if (err != ESP_OK) return -1;
      uint32_t n = 512 - curOff;
      if (n > remaining) n = remaining;
      memcpy(scratch + curOff, in, n);
      err = sdmmc_write_sectors(g_card, scratch, curLba, 1);
      if (err != ESP_OK) return -1;
      in += n; remaining -= n; curLba += 1; curOff = 0;
    }
  }
  return (int32_t)bufsize;
}

static bool msc_start_stop_cb_fn(uint8_t /*power_condition*/, bool /*start*/, bool /*load_eject*/) {
  return true;
}

static bool mscBegin() {
  if (mscEnabled) return true;
  // Ensure SD_MMC wrapper is not holding the bus
  if (mscMounted) { SD_MMC.end(); mscMounted = false; }

  // Init SDMMC host and slot
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = SDMMC_HOST_SLOT_1; // 4-bit slot
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.clk = (gpio_num_t)TF_CLK;
  slot.cmd = (gpio_num_t)TF_CMD;
  slot.d0  = (gpio_num_t)TF_D0;
  slot.d1  = (gpio_num_t)TF_D1;
  slot.d2  = (gpio_num_t)TF_D2;
  slot.d3  = (gpio_num_t)TF_D3;
  slot.width = 4;

  if (sdmmc_host_init() != ESP_OK) return false;
  if (sdmmc_host_init_slot(host.slot, &slot) != ESP_OK) {
    sdmmc_host_deinit();
    return false;
  }
  static sdmmc_card_t card; // static storage
  if (sdmmc_card_init(&host, &card) != ESP_OK) {
    sdmmc_host_deinit_slot(host.slot);
    sdmmc_host_deinit();
    return false;
  }
  g_card = &card;

  // Compute capacity
  uint64_t bytes = (uint64_t)g_card->csd.capacity * g_card->csd.sector_size;
  mscBlockSize = 512;
  mscBlockCount = (uint32_t)(bytes / mscBlockSize);

  usbMsc.onStartStop(msc_start_stop_cb_fn);
  usbMsc.onRead(msc_read_cb_fn);
  usbMsc.onWrite(msc_write_cb_fn);
  usbMsc.isWritable(true);
  usbMsc.mediaPresent(true);
  usbMsc.begin(mscBlockCount, mscBlockSize);
  #ifdef USB_VID
  USB.begin();
  #endif
  mscEnabled = true;
  return true;
}

static void mscEnd() {
  if (!mscEnabled) return;
  usbMsc.end();
  // Deinit SDMMC host
  sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_1);
  sdmmc_host_deinit();
  g_card = nullptr;
  mscEnabled = false;
}

// Button handling
volatile bool buttonState = true; // pull-up
unsigned long pressStartMs = 0;
bool inPress = false;

// Mode: start in AP mode for simplicity
bool apMode = true;

// Simple menu page index: 0=status, 1=network
uint8_t uiPage = 0;

/**********************
 * Helpers            *
 **********************/
String ipToString(const IPAddress& ip) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return String(buf);
}

// Simple OS guess based on headers/banners
static String osGuess(const String& httpServer, const String& sshBanner) {
  String s = httpServer; s.toLowerCase();
  String b = sshBanner; b.toLowerCase();
  if (s.indexOf("microsoft-iis") >= 0) return "Windows (IIS)";
  if (s.indexOf("apache") >= 0) return "Unix-like (Apache)";
  if (s.indexOf("nginx") >= 0) return "Unix-like (nginx)";
  if (b.indexOf("openssh") >= 0) return "Unix-like (OpenSSH)";
  if (s.indexOf("cisco") >= 0 || b.indexOf("cisco") >= 0) return "Cisco";
  if (s.length() || b.length()) return "Unknown (from headers)";
  return "Unknown";
}

// NBNS (NetBIOS) name query to UDP/137, returns hostname if available
static String nbnsQueryName(const IPAddress& ip) {
  WiFiUDP udp;
  if (!udp.begin(0)) return String();
  uint8_t pkt[50] = {0};
  // Transaction ID
  pkt[0] = 0x12; pkt[1] = 0x34;
  // Flags: recursion desired
  pkt[2] = 0x01; pkt[3] = 0x10;
  // QDCOUNT = 1
  pkt[4] = 0x00; pkt[5] = 0x01;
  // QName: '*'
  // NetBIOS encoding of '*' (0x2A) as 32-byte label 'CKAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'
  // We'll just query for <00> name wildcard: 0x20 label length then encoded name
  int idx = 12;
  pkt[idx++] = 0x20;
  const char* star = "CKAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
  for (int i=0;i<32;i++) pkt[idx++] = star[i];
  pkt[idx++] = 0x00;
  // QTYPE NB (0x0020), QCLASS IN (0x0001)
  pkt[idx++] = 0x00; pkt[idx++] = 0x20; pkt[idx++] = 0x00; pkt[idx++] = 0x01;
  udp.beginPacket(ip, 137);
  udp.write(pkt, idx);
  udp.endPacket();
  uint32_t start = millis();
  String name;
  while (millis() - start < 250) {
    int p = udp.parsePacket();
    if (p > 0) {
      uint8_t buf[300]; int n = udp.read(buf, sizeof(buf));
      // Very simple parse: find number of names at offset 56 per NBNS node status response
      if (n > 57) {
        int names = buf[56];
        int off = 57;
        for (int i=0;i<names && off+15 < n; ++i) {
          char nm[16]; memset(nm, 0, sizeof(nm));
          for (int j=0;j<15 && off+j<n; ++j) nm[j] = (char)buf[off+j];
          nm[15] = 0;
          uint8_t suffix = buf[off+15];
          off += 18; // 16 name+suffix + 2 flags
          // 0x00 is workstation service name
          if (suffix == 0x00) { name = String(nm); name.trim(); break; }
        }
      }
      break;
    }
    delay(5); yield();
  }
  udp.stop();
  return name;
}

// mDNS helpers
static void ensureMdnsStarted() {
  if (!mdnsStarted && !apMode) {
    MDNS.begin("tdongle");
    mdnsStarted = true;
  }
}
static void addMdnsEntry(const String& ip, const String& name, const String& svc) {
  for (auto &e: mdnsMap) {
    if (e.ip == ip) {
      if (e.services.length()) e.services += "," + svc; else e.services = svc;
      if (!e.name.length()) e.name = name;
      return;
    }
  }
  MdnsEntry e{ip, name, svc}; mdnsMap.push_back(e);
}
static void refreshMdnsServices() {
  mdnsMap.clear();
  if (apMode) return;
  ensureMdnsStarted();
  int n = MDNS.queryService("_http", "_tcp");
  for (int i=0;i<n;++i) addMdnsEntry(ipToString(MDNS.address(i)), MDNS.hostname(i), "http");
  int m = MDNS.queryService("_ssh", "_tcp");
  for (int i=0;i<m;++i) addMdnsEntry(ipToString(MDNS.address(i)), MDNS.hostname(i), "ssh");
}

// Read up to max bytes or until CRLF from client with a short timeout
static String readLine(WiFiClient &c, uint16_t maxLen = 256, uint32_t toMs = 200) {
  String line;
  uint32_t start = millis();
  while (millis() - start < toMs && line.length() < maxLen) {
    if (c.available()) {
      char ch = (char)c.read();
      if (ch == '\n') break;
      if (ch != '\r') line += ch;
    } else {
      delay(1);
      yield();
    }
  }
  return line;
}

// Try to capture basic service fingerprints from a host (HTTP server header/title, SSH banner)
String fingerprintHost(const IPAddress& ip) {
  StaticJsonDocument<256> doc;
  // HTTP
  {
    WiFiClient http; http.setTimeout(200);
    if (http.connect(ip, 80)) {
      String host = ipToString(ip);
      http.print(String("GET / HTTP/1.0\r\nHost: ") + host + "\r\nConnection: close\r\n\r\n");
      // Read headers briefly
      String serverHdr;
      String title;
      uint32_t t0 = millis();
      bool inHeaders = true;
      String buf;
      while (millis() - t0 < 300 && http.connected()) {
        while (http.available()) {
          char ch = (char)http.read();
          buf += ch;
          if (inHeaders) {
            int eol;
            while ((eol = buf.indexOf('\n')) >= 0) {
              String ln = buf.substring(0, eol);
              if (ln.endsWith("\r")) ln.remove(ln.length()-1);
              buf.remove(0, eol+1);
              if (ln.length() == 0) { inHeaders = false; break; }
              if (ln.startsWith("Server:")) { serverHdr = ln.substring(7); serverHdr.trim(); }
            }
          } else {
            // Body: look for <title>
            int ps = buf.indexOf("<title>");
            if (ps >= 0) {
              int pe = buf.indexOf("</title>", ps);
              if (pe > ps) {
                title = buf.substring(ps+7, pe);
                title.trim();
                break;
              }
            }
          }
        }
        if (!http.available()) { delay(2); yield(); }
      }
      http.stop();
      if (serverHdr.length()) doc["http_server"] = serverHdr;
      if (title.length()) doc["http_title"] = title;
    }
  }
  // SSH
  {
    WiFiClient ssh; ssh.setTimeout(200);
    if (ssh.connect(ip, 22)) {
      String banner = readLine(ssh, 200, 250);
      ssh.stop();
      if (banner.startsWith("SSH-")) doc["ssh_banner"] = banner;
    }
  }
  // NBNS hostname
  {
    String nb = nbnsQueryName(ip);
    if (nb.length()) doc["nbns_name"] = nb;
  }
  // OS guess
  {
    String hs = doc.containsKey("http_server") ? (const char*)doc["http_server"] : String();
    String sb = doc.containsKey("ssh_banner") ? (const char*)doc["ssh_banner"] : String();
    String os = osGuess(hs, sb);
    if (os.length()) doc["os_guess"] = os;
  }
  String out; serializeJson(doc, out);
  return out;
}

String subnetBase(const IPAddress& ip) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%u.%u.%u", ip[0], ip[1], ip[2]);
  return String(buf);
}

void setBacklight(bool on) {
  backlightOn = on;
  if (PIN_TFT_BL >= 0) {
    pinMode(PIN_TFT_BL, OUTPUT);
    // Digital on/off via LEDC helper
    // Map on -> 255, off -> 0
    backlightLevel = on ? 255 : 0;
    // If LEDC is configured, duty will be applied there
    digitalWrite(PIN_TFT_BL, on ? LOW : HIGH);
  }
}

// LEDC PWM for stable backlight
const int BL_LEDC_CHANNEL = 0;
const int BL_LEDC_FREQ = 5000; // 5kHz
const int BL_LEDC_RES = 8;     // 8-bit

void setupBacklightPWM() {
  if (PIN_TFT_BL < 0) return;
  #ifdef ARDUINO_ARCH_ESP32
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      // ESP32 core v3+: attach pin directly with freq and resolution
      ledcAttach(PIN_TFT_BL, BL_LEDC_FREQ, BL_LEDC_RES);
    #else
      // Older cores: use classic setup + attachPin
      ledcSetup(BL_LEDC_CHANNEL, BL_LEDC_FREQ, BL_LEDC_RES);
      ledcAttachPin(PIN_TFT_BL, BL_LEDC_CHANNEL);
    #endif
    // Active LOW: duty = 255 - level (pin-based API on ESP32 core 3.3.0)
    ledcWrite(PIN_TFT_BL, 255 - backlightLevel);
  #else
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, backlightOn ? LOW : HIGH);
  #endif
}

void setBacklightLevel(uint8_t level) {
  backlightLevel = level;
  backlightOn = (level > 0);
  if (PIN_TFT_BL >= 0) {
    #ifdef ARDUINO_ARCH_ESP32
      // Apply duty via pin-based API (active LOW)
      ledcWrite(PIN_TFT_BL, 255 - backlightLevel);
    #else
      // Fallback: simple ON/OFF
      digitalWrite(PIN_TFT_BL, backlightOn ? LOW : HIGH);
    #endif
  }
}

void drawStatusPage() {
  if (!tftReady) return;
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1); // 1 or 3 typically best for 80x160
  tft.setTextWrap(false);

  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(1);

  tft.println("T-Dongle SwissKnife");

  tft.setTextColor(ST77XX_WHITE);
  if (uiPage == 0) {
    tft.print("Mode: "); tft.println(apMode ? "AP" : "STA");
    tft.print("SSID: "); tft.println(apMode ? AP_SSID : WiFi.SSID());
    tft.print("IP:   "); tft.println(ipToString(apMode ? WiFi.softAPIP() : WiFi.localIP()));
    if (!apMode) {
      tft.print("RSSI: "); tft.println(WiFi.RSSI());
      tft.print("CH:   "); tft.println(WiFi.channel());
    }
    tft.print("BL:   "); tft.println(backlightOn ? "ON" : "OFF");
    tft.print("Heap: "); tft.println(ESP.getFreeHeap());
    tft.setTextColor(ST77XX_YELLOW);
    tft.println("\nBTN: short=Menu, long=Sleep");
  } else if (uiPage == 1) {
    tft.println("Network");
    tft.print("STA SSID: "); tft.println(staSsid);
    tft.print("STA IP:   "); tft.println(ipToString(WiFi.localIP()));
    tft.print("GW:  "); tft.println(ipToString(WiFi.gatewayIP()));
    tft.print("DNS: "); tft.println(ipToString(WiFi.dnsIP()));
    tft.println("Web: /config to set WiFi");
    tft.print("Ports open: "); tft.println(lastOpenPorts);
    tft.print("Hosts found: "); tft.println(lastDiscoverHosts);
  }
}

/**********************
 * Web Server         *
 **********************/
void handleRoot() {
  String html = F("<!doctype html><html lang='en'><head>"
  "<meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=5.0, user-scalable=yes'>"
  "<title>T-Dongle SwissKnife</title>"
  "<link href='https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap' rel='stylesheet'>");
  
  // Add CSS
  html += F("<style>"
  ":root {"
  "--primary: #4361ee;"
  "--primary-hover: #3a56d4;"
  "--bg: #f8f9fa;"
  "--card-bg: #ffffff;"
  "--text: #2d3748;"
  "--text-secondary: #718096;"
  "--border: #e2e8f0;"
  "--success: #10b981;"
  "--warning: #f59e0b;"
  "--error: #ef4444;"
  "--shadow: 0 1px 3px rgba(0,0,0,0.1), 0 1px 2px rgba(0,0,0,0.06);"
  "--radius: 8px;"
  "}"
  
  "* { box-sizing: border-box; margin: 0; padding: 0; }"
  
  "body {"
  "font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, sans-serif;"
  "line-height: 1.5;"
  "color: var(--text);"
  "background-color: var(--bg);"
  "padding: 20px;"
  "max-width: 1200px;"
  "margin: 0 auto;"
  "}"
  
  "h1, h2, h3, h4 { color: #1a202c; font-weight: 600; margin-bottom: 1rem; }"
  "h1 { font-size: 1.75rem; margin: 0.5rem 0 1.5rem; }"
  "h2 { font-size: 1.5rem; margin: 1.5rem 0 1rem; }"
  "h3 { font-size: 1.25rem; margin: 1.25rem 0 0.75rem; }"
  
  ".container { display: grid; grid-template-columns: 240px 1fr; gap: 1.5rem; }"
  ".sidebar { position: sticky; top: 1rem; height: fit-content; background: var(--card-bg); border-radius: var(--radius); padding: 1.25rem; box-shadow: var(--shadow); }"
  ".main-content { display: grid; gap: 1.5rem; }"
  ".card { background: var(--card-bg); border-radius: var(--radius); padding: 1.5rem; box-shadow: var(--shadow); margin-bottom: 1.5rem; }"
  ".tabs { display: flex; flex-direction: column; gap: 0.5rem; margin: 1rem 0; }"
  
  ".tab-button {"
  "display: flex;"
  "align-items: center;"
  "gap: 0.75rem;"
  "padding: 0.75rem 1rem;"
  "border: none;"
  "background: none;"
  "border-radius: 6px;"
  "cursor: pointer;"
  "font-size: 0.9375rem;"
  "font-weight: 500;"
  "color: var(--text);"
  "text-align: left;"
  "transition: all 0.2s;"
  "}"
  
  ".tab-button:hover { background: #f1f5f9; }"
  ".tab-button.active { background: var(--primary); color: white; }"
  
  ".panel {"
  "display: none;"
  "animation: fadeIn 0.3s ease;"
  "background: var(--card-bg);"
  "border-radius: var(--radius);"
  "padding: 1.5rem;"
  "box-shadow: var(--shadow);"
  "}"
  
  ".panel.active { display: block; }"
  
  "button, .btn {"
  "background: var(--primary);"
  "color: white;"
  "border: none;"
  "border-radius: 6px;"
  "padding: 0.625rem 1.25rem;"
  "font-size: 0.9375rem;"
  "font-weight: 500;"
  "cursor: pointer;"
  "transition: all 0.2s;"
  "display: inline-flex;"
  "align-items: center;"
  "gap: 0.5rem;"
  "margin: 0.25rem 0;"
  "}"
  
  "button:hover, .btn:hover { background: var(--primary-hover); transform: translateY(-1px); }"
  
  "input[type='text'], input[type='number'], input[type='password'], select, textarea {"
  "width: 100%;"
  "padding: 0.625rem 0.875rem;"
  "border: 1px solid var(--border);"
  "border-radius: 6px;"
  "font-size: 0.9375rem;"
  "transition: border-color 0.2s;"
  "margin: 0.25rem 0;"
  "}"
  
  "input:focus, select:focus, textarea:focus {"
  "outline: none;"
  "border-color: var(--primary);"
  "box-shadow: 0 0 0 2px rgba(67, 97, 238, 0.2);"
  "}"
  
  "label { display: block; margin-bottom: 0.5rem; font-weight: 500; font-size: 0.875rem; }"
  
  "fieldset {"
  "border: 1px solid var(--border);"
  "border-radius: var(--radius);"
  "padding: 1rem;"
  "margin: 1rem 0;"
  "}"
  
  "legend { padding: 0 0.5rem; font-weight: 600; }"
  
  "pre, code {"
  "font-family: 'SFMono-Regular', Consolas, 'Liberation Mono', Menlo, monospace;"
  "background: #f8fafc;"
  "border: 1px solid var(--border);"
  "border-radius: 6px;"
  "padding: 1rem;"
  "overflow-x: auto;"
  "font-size: 0.875rem;"
  "line-height: 1.5;"
  "}"
  
  "code { padding: 0.2em 0.4em; font-size: 0.9em; }"
  
  ".status-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 1rem; margin: 1rem 0; }"
  
  ".status-item {"
  "background: #f8fafc;"
  "border-left: 3px solid var(--primary);"
  "padding: 0.875rem 1rem;"
  "border-radius: 0 var(--radius) var(--radius) 0;"
  "}"
  
  ".status-item h4 {"
  "margin: 0 0 0.5rem;"
  "font-size: 0.875rem;"
  "color: var(--text-secondary);"
  "text-transform: uppercase;"
  "letter-spacing: 0.05em;"
  "}"
  
  ".status-value { font-size: 1.125rem; font-weight: 600; }"
  
  "@media (max-width: 768px) {"
  "  .container { grid-template-columns: 1fr; }"
  "  .sidebar { position: static; margin-bottom: 1.5rem; }"
  "  .tabs { flex-direction: row; overflow-x: auto; padding-bottom: 0.5rem; }"
  "  .tab-button { white-space: nowrap; }"
  "}"
  
  "@keyframes fadeIn { from { opacity: 0; transform: translateY(10px); } to { opacity: 1; transform: translateY(0); } }"
  
  "/* Custom scrollbar */"
  "::-webkit-scrollbar { width: 8px; height: 8px; }"
  "::-webkit-scrollbar-track { background: #f1f1f1; border-radius: 4px; }"
  "::-webkit-scrollbar-thumb { background: #c1c1c1; border-radius: 4px; }"
  "::-webkit-scrollbar-thumb:hover { background: #a8a8a8; }"
  "</style>");
  
  // Close head and open body
  html += F("</head>");
  html += F("<body class='dark-theme'>");
  
  // Container and sidebar
  html += F("<div class='container'>");
  html += F("<div class='sidebar'>");
  html += F("<h2>SwissKnife</h2>");
  
  // Tabs
  html += F("<div class='tabs'>");
  html += F("<button id='tabbtn-status' class='tab-button' onclick='showTab(\"status\")'>Status</button>");
  html += F("<button id='tabbtn-backlight' class='tab-button' onclick='showTab(\"backlight\")'>Backlight</button>");
  html += F("<button id='tabbtn-ports' class='tab-button' onclick='showTab(\"ports\")'>Port Scan</button>");
  html += F("<button id='tabbtn-led' class='tab-button' onclick='showTab(\"led\")'>LED</button>");
  html += F("<button id='tabbtn-discovery' class='tab-button' onclick='showTab(\"discovery\")'>Discovery</button>");
  html += F("<button id='tabbtn-boot' class='tab-button' onclick='showTab(\"boot\")'>Boot</button>");
  html += F("<button id='tabbtn-tools' class='tab-button' onclick='showTab(\"tools\")'>Tools</button>");
  html += F("<button id='tabbtn-host' class='tab-button' onclick='showTab(\"host\")'>Host</button>");
  html += F("</div>"); // Close tabs
  
  // Status grid
  html += F("<div class='status-grid'>");
  
  // WiFi Status
  html += F("<div class='status-item'>");
  html += F("<h4>WiFi Status</h4>");
  html += F("<div id='wifi-status' class='status-value'>-</div>");
  html += F("</div>");
  
  // SD Card Status
  html += F("<div class='status-item'>");
  html += F("<h4>SD Card</h4>");
  html += F("<div id='sd-status' class='status-value'>-</div>");
  html += F("</div>");
  
  // Uptime
  html += F("<div class='status-item'>");
  html += F("<h4>Uptime</h4>");
  html += F("<div id='uptime' class='status-value'>-</div>");
  html += F("</div>");
  
  // Close status grid and sidebar
  html += F("</div>"); // Close status-grid
  html += F("</div>"); // Close sidebar
  
  // Status Panel
  html += F("<div id='panel-status' class='panel active'>");
  html += F("<h2>Status</h2>");
  html += F("<div class='card'>");
  html += F("<h3>System Information</h3>");
  html += F("<div id='system-info'></div>");
  html += F("</div>"); // Close card
  html += F("</div>"); // Close panel-status

  // Backlight Panel
  html += F("<div id='panel-backlight' class='panel'>");
  html += F("<h2>Backlight Control</h2>");
  html += F("<div class='card'>");
  html += F("<label for='backlight-brightness'>Brightness: <span id='brightness-value'>50</span>%</label>");
  html += F("<input type='range' id='backlight-brightness' min='0' max='100' value='50' oninput='updateBrightness(this.value)'>");
  html += F("</div>"); // Close card
  html += F("</div>"); // Close panel-backlight

  // Port Scan Panel
  html += F("<div id='panel-ports' class='panel'>");
  html += F("<h2>Port Scanner</h2>");
  html += F("<div class='card'>");
  html += F("<label for='scan-ip'>Target IP:</label>");
  html += F("<input type='text' id='scan-ip' placeholder='192.168.1.1' value='192.168.1.1'>");
  html += F("<label for='scan-ports'>Ports (e.g., 80,443 or 1-1024):</label>");
  html += F("<input type='text' id='scan-ports' placeholder='80,443,8080,8443' value='21,22,23,80,443,445,8080,8443'>");
  html += F("<button onclick='startPortScan()' class='btn'>Start Scan</button>");
  html += F("<div id='scan-progress' style='margin-top: 1rem; display: none;'>");
  html += F("<div style='width: 100%; background-color: #e2e8f0; border-radius: 4px;'>");
  html += F("<div id='progress-bar' style='width: 0%; height: 20px; background-color: #4361ee; border-radius: 4px;'></div>");
  html += F("</div>");
  html += F("<div id='scan-status' style='margin-top: 0.5rem; font-size: 0.875rem;'></div>");
  html += F("</div>");
  html += F("<pre id='scan-results' style='margin-top: 1rem; display: none;'></pre>");
  html += F("</div>"); // Close card
  // LED Panel
  html += F("<div id='panel-led' class='panel'>");
  html += F("<h2>LED Control</h2>");
  html += F("<div class='card'>");
  html += F("<button onclick='toggleLED()' class='btn'>Toggle LED</button>");
  html += F("<p>Current LED state: <span id='led-state'>Off</span></p>");
  html += F("</div>"); // Close card
  html += F("</div>"); // Close panel-led

  // Discovery Panel
  html += F("<div id='panel-discovery' class='panel'>");
  html += F("<h2>Network Discovery</h2>");
  html += F("<div class='card'>");
  html += F("<label for='discovery-range'>Network Range (e.g., 192.168.1.0/24):</label>");
  html += F("<input type='text' id='discovery-range' placeholder='192.168.1.0/24' value='192.168.1.0/24'>");
  html += F("<button onclick='startDiscovery()' class='btn'>Start Discovery</button>");
  html += F("<div id='discovery-results' style='margin-top: 1rem;'></div>");
  html += F("</div>"); // Close card
  html += F("</div>"); // Close panel-discovery

  // Close main content and container
  html += F("</div>"); // Close main-content
  html += F("</div>"); // Close container

  // Add JavaScript
  html += F("<script>");
  // Show/hide panels
  html += F("function showTab(tabName) {");
  html += F("  // Hide all panels");
  html += F("  document.querySelectorAll('.panel').forEach(panel => panel.classList.remove('active'));");
  html += F("  // Deactivate all tab buttons");
  html += F("  document.querySelectorAll('.tab-button').forEach(btn => btn.classList.remove('active'));");
  html += F("  // Show selected panel and activate button");
  html += F("  document.getElementById('panel-' + tabName).classList.add('active');");
  html += F("  document.getElementById('tabbtn-' + tabName).classList.add('active');");
  html += F("  // Save active tab");
  html += F("  localStorage.setItem('activeTab', tabName);");
  html += F("}");
  
  // Load saved tab
  html += F("document.addEventListener('DOMContentLoaded', function() {");
  html += F("  const savedTab = localStorage.getItem('activeTab') || 'status';");
  html += F("  showTab(savedTab);");
  
  // Update system info
  html += F("  updateSystemInfo();");
  html += F("  setInterval(updateSystemInfo, 5000);");
  html += F("});");
  
  // System info updater
  html += F("function updateSystemInfo() {");
  html += F("  fetch('/api/system')");
  html += F("    .then(response => response.json())");
  html += F("    .then(data => {");
  html += F("      document.getElementById('system-info').innerHTML = `");
  html += F("        <p><strong>Device:</strong> ${data.device}</p>");
  html += F("        <p><strong>IP Address:</strong> ${data.ip}</p>");
  html += F("        <p><strong>MAC Address:</strong> ${data.mac}</p>");
  html += F("        <p><strong>Uptime:</strong> ${data.uptime}</p>");
  html += F("        <p><strong>Free Heap:</strong> ${data.freeHeap} bytes</p>");
  html += F("      `;");
  html += F("    });");
  html += F("  document.getElementById('wifi-status').textContent = WiFi.status() === 3 ? 'Connected' : 'Disconnected';");
  html += F("  document.getElementById('sd-status').textContent = '${SD_MMC.cardSize() / (1024 * 1024)} MB';");
  html += F("  document.getElementById('uptime').textContent = formatUptime(millis() / 1000);");
  html += F("}");
  
  // Uptime formatter
  html += F("function formatUptime(seconds) {");
  html += F("  const days = Math.floor(seconds / 86400);");
  html += F("  const hours = Math.floor((seconds % 86400) / 3600);");
  html += F("  const mins = Math.floor((seconds % 3600) / 60);");
  html += F("  const secs = Math.floor(seconds % 60);");
  html += F("  return `${days}d ${hours.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;");
  html += F("}");
  
  // Port scan functions
  html += F("async function startPortScan() {");
  html += F("  const ip = document.getElementById('scan-ip').value;");
  html += F("  const ports = document.getElementById('scan-ports').value;");
  html += F("  const progressBar = document.getElementById('progress-bar');");
  html += F("  const scanStatus = document.getElementById('scan-status');");
  html += F("  const results = document.getElementById('scan-results');");
  html += F("  const progress = document.getElementById('scan-progress');");
  
  html += F("  // Show progress and clear previous results");
  html += F("  progress.style.display = 'block';");
  html += F("  progressBar.style.width = '0%';");
  html += F("  scanStatus.textContent = 'Preparing scan...';");
  html += F("  results.style.display = 'none';");
  
  html += F("  try {");
  html += F("    const response = await fetch(`/portscan?ip=${encodeURIComponent(ip)}&ports=${encodeURIComponent(ports)}`);");
  html += F("    const data = await response.json();");
  html += F("    ");
  html += F("    if (data.error) {");
  html += F("      throw new Error(data.error);");
  html += F("    }");
  html += F("    ");
  html += F("    // Display results");
  html += F("    results.textContent = JSON.stringify(data, null, 2);");
  html += F("    results.style.display = 'block';");
  html += F("    progressBar.style.width = '100%';");
  html += F("    scanStatus.textContent = `Scan complete: ${data.open.length} open ports found`;");
  html += F("  } catch (error) {");
  html += F("    scanStatus.textContent = `Error: ${error.message}`;");
  html += F("    progressBar.style.backgroundColor = '#ef4444';");
  html += F("  }");
  html += F("}");
  
  // LED control
  html += F("async function toggleLED() {");
  html += F("  try {");
  html += F("    const response = await fetch('/led/toggle', { method: 'POST' });");
  html += F("    const data = await response.json();");
  html += F("    document.getElementById('led-state').textContent = data.state ? 'On' : 'Off';");
  html += F("  } catch (error) {");
  html += F("    console.error('Error toggling LED:', error);");
  html += F("  }");
  html += F("}");
  
  // Brightness control
  html += F("async function updateBrightness(value) {");
  html += F("  document.getElementById('brightness-value').textContent = value;");
  html += F("  try {");
  html += F("    await fetch('/backlight', {");
  html += F("      method: 'POST',");
  html += F("      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },");
  html += F("      body: `brightness=${value}`");
  html += F("    });");
  html += F("  } catch (error) {");
  html += F("    console.error('Error updating brightness:', error);");
  html += F("  }");
  html += F("}");
  
  // Network discovery
  html += F("async function startDiscovery() {");
  html += F("  const range = document.getElementById('discovery-range').value;");
  html += F("  const results = document.getElementById('discovery-results');");
  html += F("  results.innerHTML = '<p>Scanning network... This may take a while.</p>';");
  html += F("  ");
  html += F("  try {");
  html += F("    const response = await fetch(`/discover?range=${encodeURIComponent(range)}`);");
  html += F("    const hosts = await response.json();");
  html += F("    ");
  html += F("    if (hosts.length === 0) {");
  html += F("      results.innerHTML = '<p>No hosts found on the network.</p>';");
  html += F("      return;");
  html += F("    }");
  html += F("    ");
  html += F("    // Create a table to display the results");
  html += F("    let table = '<table class=\"result-table\">' +");
  html += F("      '<tr><th>IP Address</th><th>Hostname</th><th>Status</th><th>Services</th></tr>';");
  html += F("    ");
  html += F("    hosts.forEach(host => {");
  html += F("      table += '<tr>' +");
  html += F("        '<td>' + (host.ip || 'N/A') + '</td>' +");
  html += F("        '<td>' + (host.hostname || 'N/A') + '</td>' +");
  html += F("        '<td>' + (host.status || 'Unknown') + '</td>' +");
  html += F("        '<td>' + (host.services ? host.services.join(', ') : 'None detected') + '</td>'");
  html += F("      + '</tr>';");
  html += F("    });");
  html += F("    ");
  html += F("    table += '</table>';");
  html += F("    results.innerHTML = table;");
  html += F("  } catch (error) {");
  html += F("    console.error('Discovery error:', error);");
  html += F("    var errorMsg = document.createElement('p');");
  html += F("    errorMsg.style.color = 'red';");
  html += F("    errorMsg.textContent = 'Error: ' + error.message;");
  html += F("    results.innerHTML = '';");
  html += F("    results.appendChild(errorMsg);");
  html += F("  }");
  html += F("}");
  
  // Close script tag
  html += F("</script>");
  
  // Close body and html tags
  html += F("</body>");
  html += F("</html>");
  
  // Send the complete HTML response
  server.send(200, "text/html", html);
}

void handleStatus() {
  StaticJsonDocument<512> doc;
  doc["mode"] = apMode ? "AP" : "STA";
  doc["ip"] = apMode ? ipToString(WiFi.softAPIP()) : ipToString(WiFi.localIP());
  doc["heap"] = (uint32_t)ESP.getFreeHeap();
  if (!apMode) doc["gateway"] = ipToString(WiFi.gatewayIP());
  // Always provide base from current interface
  doc["base"] = apMode ? subnetBase(WiFi.softAPIP()) : subnetBase(WiFi.localIP());
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleScan() {
  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  StaticJsonDocument<4096> doc;
  JsonArray arr = doc.createNestedArray("aps");
  for (int i = 0; i < n; ++i) {
    JsonObject o = arr.createNestedObject();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
    o["chan"] = WiFi.channel(i);
    o["enc"]  = WiFi.encryptionType(i);
    o["bssid"] = WiFi.BSSIDstr(i);
  }
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Config page (prefill SSID)
void handleConfigPage() {
  String html = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>WiFi Config</title>"
                "<style>body{font-family:system-ui,Arial;margin:16px}</style>"
                "</head><body>"
                "<h3>WiFi Station Configuration</h3>"
                "<div><button onclick=scan()>Scan WiFi</button> <span id='scaninfo'></span></div>"
                "<form method='POST' action='/save_wifi'>"
                "<label>SSID <input list='ssids' id='ssid' name='ssid' value='" + staSsid + "'></label>"
                "<datalist id='ssids'></datalist><br>"
                "<label>Password <input id='pass' name='pass' type='password' value=''></label><br><br>"
                "<button type='submit'>Save</button>"
                "</form>"
                "<p><a href='/'>Back</a></p>"
                "<script>"
                "async function scan(){document.getElementById('scaninfo').textContent='Scanning...';"
                "const j=await fetch('/scan').then(r=>r.json());const dl=document.getElementById('ssids');dl.innerHTML='';"
                "(j.aps||[]).forEach(ap=>{const o=document.createElement('option');o.value=ap.ssid;dl.appendChild(o);});"
                "document.getElementById('scaninfo').textContent=`Found ${(j.aps||[]).length} networks`;}"
                "</script>"
                "</body></html>";
  server.send(200, "text/html", html);
}

// Save WiFi and reboot
void handleSaveWifi() {
  if (!server.hasArg("ssid")) { server.send(400, "text/plain", "ssid required"); return; }
  String s = server.arg("ssid");
  String p = server.hasArg("pass") ? server.arg("pass") : "";
  prefs.begin("wifi", false);
  prefs.putString("ssid", s);
  prefs.putString("pass", p);
  prefs.end();
  server.send(200, "text/plain", "Saved. Rebooting...");
  delay(300);
  ESP.restart();
}

// Basic TCP port scanner: /scan_ports?ip=192.168.1.1&ports=22,80,443 or range 1-1024
void handlePortScan() {
  if (!server.hasArg("ip")) { server.send(400, "text/plain", "IP address required"); return; }
  IPAddress target; if (!target.fromString(server.arg("ip"))) { server.send(400, "text/plain", "Invalid IP address"); return; }

  // Build port list
  std::vector<uint16_t> ports;
  const uint16_t MAX_PORTS = 252; // Maximum number of ports to scan at once
  
  if (server.hasArg("ports")) {
    String ps = server.arg("ports");
    int dash = ps.indexOf('-');
    if (dash > 0) {
      // Handle port range (e.g., 1-1024)
      int a = ps.substring(0, dash).toInt();
      int b = ps.substring(dash + 1).toInt();
      if (a <= 0 || b <= 0 || b < a) {
        server.send(400, "text/plain", "Invalid port range");
        return;
      }
      // Limit range to MAX_PORTS ports
      if ((b - a + 1) > MAX_PORTS) {
        b = a + MAX_PORTS - 1;
        if (b > 65535) b = 65535;
      }
      for (int p = a; p <= b; ++p) ports.push_back(p);
    } else {
      // Handle comma-separated list
      int start = 0;
      while (start < (int)ps.length() && ports.size() < MAX_PORTS) {
        int comma = ps.indexOf(',', start);
        if (comma < 0) comma = ps.length();
        int val = ps.substring(start, comma).toInt();
        if (val > 0 && val <= 65535) ports.push_back(val);
        start = comma + 1;
      }
    }
  }
  if (ports.empty()) { 
    // Default ports if none specified
    uint16_t defaults[] = {21, 22, 23, 53, 80, 139, 443, 445, 8080, 8443};
    ports.assign(defaults, defaults + (sizeof(defaults)/sizeof(defaults[0])));
  }

  StaticJsonDocument<4096> doc; // Increased buffer for larger responses
  doc["ip"] = server.arg("ip");
  doc["scanned"] = ports.size();
  JsonArray open = doc.createNestedArray("open");
  
  WiFiClient client;
  client.setTimeout(500); // Increased timeout for better reliability
  uint16_t opened = 0;
  
  // Scan ports with progress updates for large scans
  for (size_t i = 0; i < ports.size(); ++i) {
    uint16_t p = ports[i];
    if (client.connect(target, p, 500)) { // 500ms connection timeout
      open.add(p); 
      opened++;
      client.stop();
      delay(5); // Small delay between successful connections
    } else {
      delay(2); // Shorter delay for failed attempts
    }
    // Allow other tasks to run every 16 ports
    if ((i & 0x0F) == 0) yield();
  }
  lastOpenPorts = opened;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void startWeb() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/scan", handleScan);
  server.on("/config", handleConfigPage);
  server.on("/save_wifi", HTTP_POST, handleSaveWifi);
  server.on("/scan_ports", handlePortScan);
  server.on("/scan_ports_all", [](){
    if (!server.hasArg("ip")) { server.send(400, "text/plain", "ip required"); return; }
    IPAddress target; if (!target.fromString(server.arg("ip"))) { server.send(400, "text/plain", "bad ip"); return; }
    StaticJsonDocument<4096> doc;
    doc["ip"] = server.arg("ip");
    JsonArray open = doc.createNestedArray("open");
    WiFiClient client; client.setTimeout(60); // fast scan, 60ms
    uint16_t opened = 0;
    for (uint16_t p = 1; p <= 1024; ++p) {
      if (client.connect(target, p)) { open.add(p); opened++; client.stop(); }
      delay(1);
      yield();
    }
    lastOpenPorts = opened;
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // List ISO files in TF root and read Ventoy status
  server.on("/iso_list", [](){
    StaticJsonDocument<4096> doc; JsonArray arr = doc.createNestedArray("isos");
    if (mscEnabled) { doc["error"]="MSC enabled"; String out; serializeJson(doc,out); server.send(409,"application/json",out); return; }
    if (!tfMount()) { doc["error"]="SD mount failed"; String out; serializeJson(doc,out); server.send(500,"application/json",out); return; }
    // root ISO scan
    File root = SD_MMC.open("/");
    if (root) { File f; while ((f = root.openNextFile())) { if (!f.isDirectory()) { String nm=f.name(); nm.toLowerCase(); if (nm.endsWith(".iso")) { JsonObject o=arr.createNestedObject(); o["name"]=String(f.name()).substring(1); o["size"]=(uint32_t)f.size(); } } f.close(); } root.close(); }
    // Ventoy config presence
    File vf = SD_MMC.open("/ventoy/ventoy.json"); if (vf) { doc["ventoy"] = true; vf.close(); } else { doc["ventoy"] = false; }
    String out; serializeJson(doc, out); server.send(200, "application/json", out);
  });
  server.on("/ventoy_config_get", [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled"); return; }
    if (!tfMount()) { server.send(500, "text/plain", "SD mount failed"); return; }
    File f = SD_MMC.open("/ventoy/ventoy.json"); if (!f) { server.send(404, "text/plain", "ventoy.json not found"); return; }
    server.streamFile(f, "application/json"); f.close();
  });
  server.on("/ventoy_config_set", HTTP_POST, [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled"); return; }
    if (!tfMount()) { server.send(500, "text/plain", "SD mount failed"); return; }
    String body = server.arg("plain"); if (body.length()==0) { server.send(400, "text/plain", "Empty body"); return; }
    SD_MMC.mkdir("/ventoy");
    File f = SD_MMC.open("/ventoy/ventoy.json", FILE_WRITE);
    if (!f) { server.send(500, "text/plain", "Open failed"); return; }
    size_t w = f.print(body); f.close(); if (w != body.length()) { server.send(500, "text/plain", "Write failed"); return; }
    server.send(200, "text/plain", "Saved ventoy.json");
  });
  server.on("/logs_zip", [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled"); return; }
    if (!tfMount()) { server.send(500, "text/plain", "SD mount failed"); return; }
    File dir = SD_MMC.open("/logs");
    if (!dir || !dir.isDirectory()) { server.send(404, "text/plain", "No logs"); return; }
    struct ZItem { String name; uint32_t size; uint32_t crc; uint32_t lhoff; };
    std::vector<ZItem> items;
    // First pass: compute size and CRC for each file
    File f;
    while ((f = dir.openNextFile())) {
      if (f.isDirectory()) { f.close(); continue; }
      String full = String(f.name());
      String name = full.substring(6); // strip /logs/
      uint32_t crc = 0xFFFFFFFFu; uint32_t sz = 0; uint8_t buf[1024];
      int r; while ((r = f.read(buf, sizeof(buf))) > 0) {
        sz += (uint32_t)r;
        for (int i=0;i<r;i++) {
          crc ^= buf[i];
          for (int k=0;k<8;k++) crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
        }
      }
      crc ^= 0xFFFFFFFFu;
      items.push_back({name, sz, crc, 0});
      f.close();
    }
    dir.close();
    if (items.empty()) { server.send(404, "text/plain", "No logs"); return; }
    auto w16=[&](WiFiClient &c, uint16_t v){ uint8_t b[2]={(uint8_t)(v&0xFF),(uint8_t)(v>>8)}; c.write(b,2); };
    auto w32=[&](WiFiClient &c, uint32_t v){ uint8_t b[4]={(uint8_t)(v&0xFF),(uint8_t)((v>>8)&0xFF),(uint8_t)((v>>16)&0xFF),(uint8_t)((v>>24)&0xFF)}; c.write(b,4); };
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/zip", "");
    WiFiClient client = server.client();
    uint32_t offset = 0;
    // Second pass: write local headers + data
    for (auto &it : items) {
      it.lhoff = offset;
      // local file header
      w32(client, 0x04034b50); // sig
      w16(client, 20); // version
      w16(client, 0); // flags
      w16(client, 0); // method store
      w16(client, 0); w16(client, 0); // time/date
      w32(client, it.crc);
      w32(client, it.size);
      w32(client, it.size);
      w16(client, (uint16_t)it.name.length());
      w16(client, 0); // extra len
      client.write((const uint8_t*)it.name.c_str(), it.name.length());
      offset += 30 + it.name.length();
      // file data
      File rf = SD_MMC.open(String("/logs/") + it.name, FILE_READ);
      if (rf) {
        uint8_t buf[1024]; int r2;
        while ((r2 = rf.read(buf, sizeof(buf))) > 0) { client.write(buf, r2); offset += r2; }
        rf.close();
      }
      yield();
    }
    // Central directory
    uint32_t cd_start = offset;
    uint32_t cd_size = 0;
    for (auto &it : items) {
      w32(client, 0x02014b50); // central header
      w16(client, 20); // ver made by
      w16(client, 20); // ver needed
      w16(client, 0); // flags
      w16(client, 0); // method store
      w16(client, 0); w16(client, 0); // time/date
      w32(client, it.crc);
      w32(client, it.size);
      w32(client, it.size);
      w16(client, (uint16_t)it.name.length());
      w16(client, 0); // extra len
      w16(client, 0); // comment len
      w16(client, 0); // disk start
      w16(client, 0); // int attr
      w32(client, 0); // ext attr
      w32(client, it.lhoff); // rel offset
      client.write((const uint8_t*)it.name.c_str(), it.name.length());
      uint32_t add = 46 + it.name.length();
      offset += add; cd_size += add;
      yield();
    }
    // End of central directory
    w32(client, 0x06054b50);
    w16(client, 0); w16(client, 0); // disk
    w16(client, (uint16_t)items.size()); w16(client, (uint16_t)items.size());
    w32(client, cd_size);
    w32(client, cd_start);
    w16(client, 0); // comment len
  });
  
  // Accept raw JSON in POST body and save to /logs/<name or millis>.json
  server.on("/save_json", HTTP_POST, [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled; disable Boot Drive Mode first"); return; }
    String name = server.hasArg("name") ? server.arg("name") : String("client-") + String(millis()) + ".json";
    String body = server.arg("plain");
    if (body.length() == 0) { server.send(400, "text/plain", "Empty body"); return; }
    if (!name.endsWith(".json")) name += ".json";
    if (!tfMount()) { server.send(500, "text/plain", "SD mount failed"); return; }
    SD_MMC.mkdir("/logs");
    File f = SD_MMC.open(String("/logs/") + name, FILE_WRITE);
    if (!f) { server.send(500, "text/plain", "Open failed"); return; }
    size_t w = f.print(body);
    f.close();
    if (w != body.length()) { server.send(500, "text/plain", "Write failed"); return; }
    server.send(200, "text/plain", String("Saved /logs/") + name);
  });

  // Build a device-side snapshot and save it server-side (no browser extras)
  server.on("/save_snapshot", [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled; disable Boot Drive Mode first"); return; }
    StaticJsonDocument<1024> doc;
    // status
    {
      StaticJsonDocument<512> s; s["mode"] = apMode ? "AP" : "STA"; s["ip"] = apMode ? ipToString(WiFi.softAPIP()) : ipToString(WiFi.localIP()); s["gateway"] = ipToString(WiFi.gatewayIP()); s["rssi"] = WiFi.RSSI(); doc["status"] = s;
    }
    // device
    {
      StaticJsonDocument<256> d; d["chip"] = ESP.getChipModel(); d["cores"] = ESP.getChipCores(); d["freqMHz"] = ESP.getCpuFreqMHz(); d["flashMB"] = ESP.getFlashChipSize() / (1024*1024); doc["device"] = d;
    }
    // msc status
    {
      StaticJsonDocument<128> m; m["enabled"] = mscEnabled; m["armed"] = mscAutoArm; doc["msc"] = m;
    }
    // optional sd health (only if mounted ok)
    {
      bool ok = tfMount();
      StaticJsonDocument<256> sd; sd["mounted"] = ok; if (ok) { sd["total_bytes"] = SD_MMC.totalBytes(); sd["used_bytes"] = SD_MMC.usedBytes(); }
      doc["sd_health"] = sd;
    }
    String out; serializeJson(doc, out);
    // save
    String fname = String("snapshot-") + String(millis()) + ".json";
    // Need a tiny wrapper since this handler is a lambda without capture of helper; reopen SD directly
    SD_MMC.mkdir("/logs");
    File f = SD_MMC.open(String("/logs/") + fname, FILE_WRITE);
    if (!f) { server.send(500, "text/plain", "Open failed"); return; }
    size_t w = f.print(out); f.close(); if (w != out.length()) { server.send(500, "text/plain", "Write failed"); return; }
    server.send(200, "text/plain", String("Saved /logs/") + fname);
  });

  // Logs management
  server.on("/logs_list", [](){
    StaticJsonDocument<2048> doc; JsonArray arr = doc.createNestedArray("files");
    if (mscEnabled) { doc["error"] = "MSC enabled"; String out; serializeJson(doc, out); server.send(409, "application/json", out); return; }
    if (!tfMount()) { doc["error"] = "SD mount failed"; String out; serializeJson(doc, out); server.send(500, "application/json", out); return; }
    File dir = SD_MMC.open("/logs");
    if (!dir || !dir.isDirectory()) { String out; serializeJson(doc, out); server.send(200, "application/json", out); return; }
    File f; while ((f = dir.openNextFile())) { if (!f.isDirectory()) { JsonObject o = arr.createNestedObject(); o["name"] = String(f.name()).substring(6); o["size"] = (uint32_t)f.size(); } f.close(); }
    dir.close(); String out; serializeJson(doc, out); server.send(200, "application/json", out);
  });
  server.on("/logs_get", [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled"); return; }
    if (!server.hasArg("name")) { server.send(400, "text/plain", "Missing name"); return; }
    String name = server.arg("name");
    if (name.indexOf("..") >= 0) { server.send(400, "text/plain", "Bad name"); return; }
    if (!tfMount()) { server.send(500, "text/plain", "SD mount failed"); return; }
    File f = SD_MMC.open(String("/logs/") + name, FILE_READ);
    if (!f) { server.send(404, "text/plain", "Not found"); return; }
    String ctype = name.endsWith(".json") ? "application/json" : "application/octet-stream";
    server.streamFile(f, ctype); f.close();
  });
  server.on("/logs_delete", [](){
    if (mscEnabled) { server.send(409, "text/plain", "MSC enabled"); return; }
    if (!server.hasArg("name")) { server.send(400, "text/plain", "Missing name"); return; }
    String name = server.arg("name"); if (name.indexOf("..") >= 0) { server.send(400, "text/plain", "Bad name"); return; }
    if (!tfMount()) { server.send(500, "text/plain", "SD mount failed"); return; }
    bool ok = SD_MMC.remove(String("/logs/") + name);
    server.send(ok ? 200 : 500, "text/plain", ok ? "Deleted" : "Delete failed");
  });
  // MSC placeholder endpoints
  server.on("/msc_status", [](){
    StaticJsonDocument<256> doc;
    doc["enabled"] = mscEnabled;
    doc["armed"] = mscArmed;
    if (mscEnabled && g_card) {
      uint64_t bytes = (uint64_t)g_card->csd.capacity * g_card->csd.sector_size;
      doc["mounted"] = true;
      doc["tf_size_bytes"] = bytes;
    } else {
      // lazy mount via SD_MMC only for status
      if (!mscMounted) tfMount();
      doc["mounted"] = mscMounted;
      doc["tf_size_bytes"] = (uint64_t)mscTfBytes;
    }
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  server.on("/msc_enable", [](){
    if (mscBegin()) server.send(200, "text/plain", "MSC enabled"); else server.send(500, "text/plain", "MSC enable failed");
  });
  server.on("/msc_disable", [](){
    mscEnd();
    server.send(200, "text/plain", "MSC disabled");
  });
  server.on("/msc_eject", [](){
    // For now, same as disable
    mscEnd();
    server.send(200, "text/plain", "MSC ejected");
  });
  server.on("/msc_arm_on", [](){
    prefs.begin("boot", false);
    prefs.putBool("msc_auto", true);
    prefs.end();
    mscArmed = true;
    // Enable MSC immediately in this session too
    if (mscBegin()) {
      server.send(200, "text/plain", "Boot Drive Mode ON (persistent)");
    } else {
      server.send(500, "text/plain", "Failed to enable MSC now, but setting persisted");
    }
  });
  server.on("/msc_arm_off", [](){
    prefs.begin("boot", false);
    prefs.putBool("msc_auto", false);
    prefs.end();
    mscArmed = false;
    // Disable MSC immediately
    mscEnd();
    server.send(200, "text/plain", "Boot Drive Mode OFF (persistent)");
  });
  // Client info
  server.on("/whoami", [](){
    StaticJsonDocument<256> doc;
    IPAddress ip = server.client().remoteIP();
    doc["client_ip"] = ip.toString();
    if (server.hasHeader("User-Agent")) {
      doc["user_agent"] = server.header("User-Agent");
    }
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // Host info (fingerprint HTTP/SSH of the connected client)
  server.on("/host_info", [](){
    IPAddress ip = server.client().remoteIP();
    String fp = fingerprintHost(ip); // returns JSON string
    // Merge IP into the fingerprint JSON
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, fp);
    if (err) {
      doc.clear();
    }
    doc["ip"] = ip.toString();
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // SD Health (requires MSC off)
  server.on("/sd_health", [](){
    StaticJsonDocument<256> doc;
    if (mscEnabled) {
      doc["error"] = "MSC enabled; disable Boot Drive Mode first";
      String out; serializeJson(doc, out);
      server.send(409, "application/json", out);
      return;
    }
    bool ok = tfMount();
    doc["mounted"] = ok;
    if (ok) {
      doc["total_bytes"] = (uint64_t)SD_MMC.totalBytes();
      doc["used_bytes"] = (uint64_t)SD_MMC.usedBytes();
    }
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // SD Benchmark (write+read N MB file)
  server.on("/sd_bench", [](){
    StaticJsonDocument<256> doc;
    if (mscEnabled) {
      doc["error"] = "MSC enabled; disable Boot Drive Mode first";
      String out; serializeJson(doc, out);
      server.send(409, "application/json", out);
      return;
    }
    int szmb = server.hasArg("size_mb") ? constrain(server.arg("size_mb").toInt(), 2, 64) : 8;
    if (!tfMount()) { doc["error"] = "TF mount failed"; String out; serializeJson(doc, out); server.send(500, "application/json", out); return; }
    const char* path = "/bench.bin";
    const size_t chunk = 4096;
    uint8_t buf[chunk];
    for (size_t i=0;i<chunk;i++) buf[i] = (uint8_t)(i*31u + 7u);
    // Write
    File w = SD_MMC.open(path, FILE_WRITE);
    if (!w) { doc["error"] = "open write failed"; String out; serializeJson(doc, out); server.send(500, "application/json", out); return; }
    size_t total = (size_t)szmb * 1024u * 1024u;
    unsigned long t0 = millis();
    size_t written = 0;
    while (written < total) {
      size_t n = min(chunk, total - written);
      if (w.write(buf, n) != n) { w.close(); doc["error"] = "write error"; String out; serializeJson(doc, out); server.send(500, "application/json", out); return; }
      written += n; yield();
    }
    w.flush(); w.close();
    unsigned long t1 = millis();
    // Read
    File r = SD_MMC.open(path, FILE_READ);
    if (!r) { doc["error"] = "open read failed"; String out; serializeJson(doc, out); server.send(500, "application/json", out); return; }
    unsigned long t2 = millis();
    size_t readBytes = 0;
    while (readBytes < total) {
      size_t n = r.read(buf, min(chunk, total - readBytes));
      if (n == 0) { r.close(); doc["error"] = "read error"; String out; serializeJson(doc, out); server.send(500, "application/json", out); return; }
      readBytes += n; yield();
    }
    r.close();
    unsigned long t3 = millis();
    SD_MMC.remove(path);
    float wsec = (t1 - t0) / 1000.0f;
    float rsec = (t3 - t2) / 1000.0f;
    float mb = total / 1048576.0f;
    doc["size_mb"] = szmb;
    doc["write_s"] = wsec;
    doc["read_s"] = rsec;
    doc["write_MBps"] = wsec > 0 ? (mb / wsec) : 0.0f;
    doc["read_MBps"] = rsec > 0 ? (mb / rsec) : 0.0f;
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // Recovery actions
  server.on("/reboot", [](){ server.send(200, "text/plain", "Rebooting"); delay(200); ESP.restart(); });
  server.on("/reset_prefs", [](){
    prefs.begin("wifi", false); prefs.clear(); prefs.end();
    prefs.begin("boot", false); prefs.clear(); prefs.end();
    server.send(200, "text/plain", "Preferences reset");
  });
  // Host tests
  server.on("/ping", [](){
    // Minimal body to avoid caching
    StaticJsonDocument<64> doc; doc["t"] = millis(); String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  server.on("/speed_dl", [](){
    size_t total = server.hasArg("bytes") ? (size_t)server.arg("bytes").toInt() : (size_t)(2*1024*1024);
    if (total > (size_t)(5*1024*1024)) total = (size_t)(5*1024*1024);
    const size_t chunk = 1460; // good TCP MSS-sized chunks
    uint8_t buf[chunk];
    for (size_t i=0;i<chunk;i++) buf[i] = (uint8_t)(i*73u + 19u);
    server.setContentLength(total);
    server.send(200, "application/octet-stream", "");
    WiFiClient client = server.client();
    size_t sent = 0;
    while (sent < total) {
      size_t n = min(chunk, total - sent);
      size_t w = client.write(buf, n);
      if (w == 0) break;
      sent += w;
      delay(0);
    }
  });
  server.on("/led", [](){
    uint8_t r = server.hasArg("r") ? (uint8_t)constrain(server.arg("r").toInt(), 0, 255) : 0;
    uint8_t g = server.hasArg("g") ? (uint8_t)constrain(server.arg("g").toInt(), 0, 255) : 0;
    uint8_t b = server.hasArg("b") ? (uint8_t)constrain(server.arg("b").toInt(), 0, 255) : 0;
    uint8_t br = server.hasArg("br") ? (uint8_t)constrain(server.arg("br").toInt(), 0, 31) : 8;
    ledRainbow = false; // stop rainbow if manual set
    setStatusLED(r, g, b, br);
    server.send(200, "text/plain", "OK");
  });
  server.on("/led_rainbow", [](){
    if (server.hasArg("on")) {
      String v = server.arg("on");
      if (v == "1" || v == "true") ledRainbow = true;
      else if (v == "0" || v == "false") ledRainbow = false;
      else if (v == "toggle") ledRainbow = !ledRainbow;
    } else {
      ledRainbow = !ledRainbow;
    }
    server.send(200, "text/plain", ledRainbow ? "ON" : "OFF");
  });
  server.on("/bl", [](){
    if (!server.hasArg("val")) { server.send(400, "text/plain", "val required"); return; }
    int v = constrain(server.arg("val").toInt(), 0, 255);
    setBacklightLevel((uint8_t)v);
    prefs.begin("ui", false);
    prefs.putUInt("bll", v);
    prefs.end();
    server.send(200, "text/plain", "OK");
  });
  server.on("/discover", [](){
    if (!server.hasArg("base")) { server.send(400, "text/plain", "base required (e.g. 192.168.1)"); return; }
    String base = server.arg("base");
    int start = server.hasArg("start") ? server.arg("start").toInt() : 1;
    int count = server.hasArg("count") ? server.arg("count").toInt() : 32;
    count = constrain(count, 1, 64);
    // build probe ports
    std::vector<uint16_t> ports;
    String ps = server.hasArg("ports") ? server.arg("ports") : String("80,443");
    int s = 0; while (s < (int)ps.length()) { int c = ps.indexOf(',', s); if (c < 0) c = ps.length(); int val = ps.substring(s, c).toInt(); if (val > 0) ports.push_back(val); s = c+1; if (ports.size()>=8) break; }
    if (ports.empty()) { uint16_t d[] = {80, 443}; ports.assign(d, d+2); }
    StaticJsonDocument<4096> doc;
    JsonArray arr = doc.createNestedArray("hosts");
    WiFiClient client;
    client.setTimeout(200);
    uint16_t found = 0;
    for (int i = 0; i < count; ++i) {
      String ipStr = base + "." + String(start + i);
      IPAddress ip; if (!ip.fromString(ipStr)) continue;
      bool alive = false;
      for (uint16_t p : ports) {
        if (client.connect(ip, p)) { alive = true; client.stop(); break; }
      }
      if (alive) { arr.add(ipStr); found++; }
      delay(5);
    }
    lastDiscoverHosts = found;
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // Start async discovery
  server.on("/discover_start", [](){
    if (!server.hasArg("base") || !server.hasArg("start") || !server.hasArg("count")) {
      server.send(400, "text/plain", "args: base,start,count required"); return;
    }
    discBase = server.arg("base");
    discStart = server.arg("start").toInt();
    discCount = server.arg("count").toInt();
    discIndex = 0;
    discFound.clear();
    discInfo.clear();
    // Build ports
    discPorts.clear();
    if (server.hasArg("ports")) {
      String ps = server.arg("ports");
      int dash = ps.indexOf('-');
      if (dash > 0) {
        int a = ps.substring(0, dash).toInt();
        int b = ps.substring(dash + 1).toInt();
        if (a > 0 && b >= a && b <= 65535) {
          for (int p = a; p <= b; ++p) discPorts.push_back((uint16_t)p);
        }
      } else {
        int from = 0;
        while (from < ps.length()) {
          int comma = ps.indexOf(',', from);
          if (comma < 0) comma = ps.length();
          int pv = ps.substring(from, comma).toInt();
          if (pv > 0 && pv <= 65535) discPorts.push_back((uint16_t)pv);
          from = comma + 1;
        }
      }
    }
    if (discPorts.empty()) { uint16_t defaults[] = {22,80,443}; discPorts.assign(defaults, defaults+3); }
    discoverRunning = true;
    lastDiscoverStepMs = 0;
    server.send(200, "text/plain", "STARTED");
  });
  // Async discovery status
  server.on("/discover_status", [](){
    StaticJsonDocument<4096> doc;
    doc["running"] = discoverRunning;
    doc["total"] = discCount;
    doc["progress"] = discIndex;
    JsonArray arr = doc.createNestedArray("found");
    for (size_t i=0;i<discFound.size();++i){
      JsonObject o = arr.createNestedObject();
      o["ip"] = discFound[i];
      if (i < discInfo.size()) {
        // discInfo[i] holds a compact JSON; parse and attach if possible
        StaticJsonDocument<256> tmp; DeserializationError e = deserializeJson(tmp, discInfo[i]);
        if (!e) {
          for (JsonPair kv : tmp.as<JsonObject>()) o[kv.key()] = kv.value();
        }
      }
      // attach mDNS if we have it
      for (auto &md : mdnsMap) {
        if (md.ip == discFound[i]) { if (md.name.length()) o["mdns_name"] = md.name; if (md.services.length()) o["mdns_services"] = md.services; break; }
      }
    }
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  // Device / connection info
  server.on("/device", [](){
    StaticJsonDocument<1024> doc;
    doc["mode"] = apMode ? "AP" : "STA";
    doc["uptime_s"] = (uint32_t)(millis()/1000);
    doc["heap"] = (uint32_t)ESP.getFreeHeap();
    doc["chip"] = String(ESP.getChipModel());
    doc["cpu_mhz"] = (uint32_t)getCpuFrequencyMhz();
    doc["hostname"] = WiFi.getHostname() ? WiFi.getHostname() : "";
    String macSta = WiFi.macAddress();
    String macAp = WiFi.softAPmacAddress();
    doc["mac_sta"] = macSta;
    doc["mac_ap"] = macAp;
    if (apMode) {
      doc["ssid"] = AP_SSID;
      doc["ip"] = ipToString(WiFi.softAPIP());
      doc["gw"] = ipToString(WiFi.softAPIP());
      doc["mask"] = ipToString(WiFi.softAPSubnetMask());
    } else {
      doc["ssid"] = WiFi.SSID();
      doc["bssid"] = WiFi.BSSIDstr();
      doc["rssi"] = WiFi.RSSI();
      doc["chan"] = WiFi.channel();
      doc["ip"] = ipToString(WiFi.localIP());
      doc["gw"] = ipToString(WiFi.gatewayIP());
      doc["mask"] = ipToString(WiFi.subnetMask());
      doc["dns"] = ipToString(WiFi.dnsIP());
    }
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  server.begin();
}

/**********************
 * Wi-Fi modes        *
 **********************/
void startAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
}

bool tryStartSTA() {
  if (staSsid.isEmpty()) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(staSsid.c_str(), staPass.c_str());
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  apMode = !ok;
  return ok;
}

/**********************
 * Button handling    *
 **********************/
void updateButton() {
  bool s = digitalRead(PIN_BUTTON);
  if (!inPress && s == LOW) { // pressed
    inPress = true;
    pressStartMs = millis();
  } else if (inPress && s == HIGH) { // released
    unsigned long dur = millis() - pressStartMs;
    inPress = false;
    if (dur >= 2000) {
      // Long press: turn off screen and LED
      setBacklight(false);
      setStatusLED(0, 0, 0, 0);
      drawStatusPage();
    } else if (dur >= 50) {
      // Short press: cycle UI page
      uiPage = (uiPage + 1) % 2;
      drawStatusPage();
    }
  }
}

/**********************
 * APA102 LED         *
 **********************/
inline void dotstarWriteBit(bool b) {
  digitalWrite(PIN_DOTSTAR_DATA, b ? HIGH : LOW);
  digitalWrite(PIN_DOTSTAR_CLK, HIGH);
  digitalWrite(PIN_DOTSTAR_CLK, LOW);
}

void setStatusLED(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  pinMode(PIN_DOTSTAR_DATA, OUTPUT);
  pinMode(PIN_DOTSTAR_CLK, OUTPUT);
  // Start frame
  for (int i = 0; i < 32; ++i) dotstarWriteBit(0);
  // One LED frame: 0b111xxxxx + BGR
  uint32_t hdr = 0b11100000 | (brightness & 0x1F);
  for (int i = 7; i >= 0; --i) dotstarWriteBit((hdr >> i) & 1);
  for (int i = 7; i >= 0; --i) dotstarWriteBit((b >> i) & 1);
  for (int i = 7; i >= 0; --i) dotstarWriteBit((g >> i) & 1);
  for (int i = 7; i >= 0; --i) dotstarWriteBit((r >> i) & 1);
  // End frame
  for (int i = 0; i < 32; ++i) dotstarWriteBit(1);
}

// HSV to RGB (simple, H:0..359, S/V:0..255)
static void hsvToRgb(uint16_t h, uint8_t s, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
  uint8_t region = h / 60;
  uint16_t remainder = (h % 60) * 255 / 60;
  uint8_t p = (uint16_t)v * (255 - s) / 255;
  uint8_t q = (uint16_t)v * (255 - (uint16_t)s * remainder / 255) / 255;
  uint8_t t = (uint16_t)v * (255 - (uint16_t)s * (255 - remainder) / 255) / 255;
  switch (region % 6) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
}

/**********************
 * Setup/Loop         *
 **********************/
void setup() {
  // Serial
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[T-Dongle SwissKnife] booting...");

  // Button
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Backlight default ON
  setBacklight(true);

  // SPI + TFT init
  if (PIN_TFT_SCLK >= 0 && PIN_TFT_MOSI >= 0) {
    SPI.begin(PIN_TFT_SCLK, PIN_TFT_MISO, PIN_TFT_MOSI);
  }
  tft.initR(INITR_MINI160x80);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tftReady = true;
  setupBacklightPWM();
  // Load saved backlight level
  prefs.begin("ui", true);
  uint32_t bll = prefs.getUInt("bll", 255);
  prefs.end();
  setBacklightLevel((uint8_t)bll);

  // Wi-Fi AP + Web
  // Load MSC auto-enable preference early
  prefs.begin("boot", true);
  mscArmed = prefs.getBool("msc_auto", false);
  prefs.end();
  if (mscArmed) {
    // Try to enable MSC immediately so device enumerates as USB disk on plug-in
    mscBegin();
  }

  prefs.begin("wifi", true);
  staSsid = prefs.getString("ssid", "");
  staPass = prefs.getString("pass", "");
  prefs.end();

  if (!staSsid.isEmpty() && tryStartSTA()) {
    // Connected as STA
    apMode = false;
  } else {
    startAP();
  }
  startWeb();

  drawStatusPage();
  Serial.print("Mode: "); Serial.println(apMode ? "AP" : "STA");
  Serial.print("IP:   "); Serial.println(apMode ? WiFi.softAPIP() : WiFi.localIP());
  // LED: green if STA, blue if AP
  if (apMode) setStatusLED(0, 0, 64); else setStatusLED(0, 64, 0);
}

void loop() {
  server.handleClient();
  updateButton();
  yield();
  static unsigned long lastUi = 0;
  if (millis() - lastUi > 3000) {
    drawStatusPage();
    lastUi = millis();
  }
  // Rainbow step
  if (ledRainbow && millis() - lastRainbowMs > 30) {
    lastRainbowMs = millis();
    uint8_t r, g, b;
    hsvToRgb(ledHue, 255, 64, r, g, b); // medium brightness
    setStatusLED(r, g, b, 8);
    ledHue = (ledHue + 3) % 360;
  }
  // Async discovery stepper
  if (discoverRunning) {
    // Limit rate: every ~25ms process a small batch
    if (millis() - lastDiscoverStepMs > 25) {
      lastDiscoverStepMs = millis();
      const int batch = 8; // tune for speed vs. responsiveness
      int processed = 0;
      while (processed < batch && discIndex < discCount) {
        String ipStr = discBase + "." + String(discStart + discIndex);
        IPAddress ip; ip.fromString(ipStr);
        bool alive = false;
        WiFiClient client; client.setTimeout(120);
        for (uint16_t p : discPorts) {
          if (client.connect(ip, p)) { alive = true; client.stop(); break; }
          delay(1);
          yield();
        }
        if (alive) {
          discFound.push_back(ipStr);
          // Fingerprint host quickly (HTTP/SSH)
          String info = fingerprintHost(ip);
          discInfo.push_back(info);
        }
        discIndex++;
        processed++;
      }
      lastDiscoverHosts = (uint16_t)discFound.size();
      if (discIndex >= discCount) {
        discoverRunning = false;
        // After finishing, try to enrich with mDNS
        refreshMdnsServices();
      }
    }
  }
}