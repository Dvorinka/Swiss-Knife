# ESP32 SwissKnife

A versatile multi-tool firmware for ESP32-based development boards, providing network diagnostics, device control, and system utilities through a web interface.

## Features

- **Network Tools**
  - Port scanning
  - Network discovery
  - mDNS browser
  - Wi-Fi scanner
  - Network information

- **Device Control**
  - RGB LED control
  - Backlight control
  - System status monitoring
  - Reboot and reset functions

- **Storage**
  - SD card management
  - File browser
  - Log management
  - Snapshots and backups

- **System**
  - Real-time system information
  - Memory usage
  - CPU frequency
  - Flash memory information

## Hardware Requirements

- ESP32 development board
- MicroSD card slot (for storage features)
- RGB LED (for status indication)
- Display (optional, for local UI)

## Installation

1. Install the Arduino IDE or PlatformIO
2. Install the ESP32 board support
3. Clone this repository
4. Open the project in your IDE
5. Install required libraries:
   - ArduinoJson
   - ESPAsyncWebServer
   - AsyncTCP
   - Adafruit_NeoPixel (for RGB LED control)
   - SD_MMC (for SD card access)
6. Select your ESP32 board and port
7. Compile and upload the firmware

## First Boot

1. After flashing, the device will start in Access Point mode
2. Connect to the Wi-Fi network named "ESP32-SwissKnife-XXXX"
3. Open a web browser and navigate to `http://192.168.4.1`
4. Configure your Wi-Fi settings in the web interface
5. The device will reboot and connect to your network

## Web Interface

The web interface provides access to all features:

- **Status**: View system and network information
- **Network**: Scan networks and discover devices
- **Port Scanner**: Check open ports on network devices
- **LED Control**: Control the onboard RGB LED
- **Storage**: Manage files on the SD card
- **Tools**: System utilities and settings

## API Endpoints

The firmware exposes the following REST API endpoints:

- `GET /` - Web interface
- `GET /status` - System status
- `GET /scan` - Wi-Fi network scan
- `GET /scan_ports` - Port scanner
- `POST /led` - Control RGB LED
- `GET /msc_status` - Get mass storage status
- `POST /msc_arm_on` - Enable mass storage on boot
- `POST /msc_arm_off` - Disable mass storage on boot
- `GET /sd_health` - Get SD card health info
- `POST /save_snapshot` - Save system snapshot

## Configuration

Configuration can be done through the web interface or by editing the source code:

- Wi-Fi credentials
- LED pin assignments
- Display settings
- Network scan parameters

## Troubleshooting

- **Can't connect to the web interface**
  - Ensure you're connected to the correct Wi-Fi network
  - Check if the device is powered on and running
  - Try accessing `http://esp32-swissknife.local` if mDNS is supported on your network

- **SD card not detected**
  - Ensure the SD card is properly inserted
  - Check if the SD card is formatted as FAT32
  - Verify the SD card slot connections

- **Wi-Fi connection issues**
  - Check if the Wi-Fi credentials are correct
  - Ensure the network is within range
  - Try rebooting the device

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- Built with the ESP32 Arduino Core
- Uses various open-source libraries (see source code for details)
- Inspired by network diagnostic tools and multi-function firmwares
