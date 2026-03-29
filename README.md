# ESP Open Tracker

An open-source GPS tracker built on the ESP32-C3 microcontroller, designed for real-time vehicle or asset tracking via the OsmAnd protocol.

## Features

- **Real-time GPS tracking** — sends location data to any OsmAnd-compatible server (e.g. Traccar)
- **Adaptive send interval** — adjusts automatically based on movement:
  - Stationary: every 30 seconds
  - Moving: every 10 seconds
  - Turning (>7°): every 2 seconds
- **Black box** — stores track points to LittleFS flash when offline, automatically uploads when connection is restored
- **Battery monitoring** — measures voltage via ADC with resistor divider, sends battery level with each packet
- **Power management** — low battery warning via buzzer, deep sleep on critical battery level (<2.9V)
- **FIltering zone** — replaces real coordinates with home location when without a configurable radius to make spoofing filtering
- **Wi-Fi manager** — web-based configuration portal (AP mode) for managing Wi-Fi networks and server settings
- **Auto reconnect** — automatically reconnects to saved Wi-Fi networks in the background
- **LED indicators** — non-blocking LED feedback for Wi-Fi connection and successful data transmission
- **Buzzer alerts** — audio feedback on Wi-Fi connect and low battery

## Hardware

- **MCU**: ESP32-C3 Super Mini
- **GPS**: GP-02 (NMEA, 9600 baud, UART on GPIO3/GPIO4)
- **Battery**: Li-ion, monitored via voltage divider on GPIO1
- **Buzzer**: Passive buzzer on GPIO5
- **Button**: Long press on GPIO10 opens config portal

## Configuration

All key parameters are defined in `include/TrackerConfig.h`:

```cpp
#define TRACKER_DEVICE_ID        "esp-nav-001"
#define TRACKER_INTERVAL_STATIC  30000
#define TRACKER_INTERVAL_MOVING  10000
#define TRACKER_INTERVAL_TURNING 2000
#define TRACKER_SPEED_THRESHOLD  1.0f
#define TRACKER_BEARING_THRESHOLD 7.0f
#define TRACKER_HOME_LAT         49.123456
#define TRACKER_HOME_LNG         32.123456
#define TRACKER_HOME_RADIUS_KM   0.2f
```

## OsmAnd Protocol

Data is sent via HTTP GET to your server:

```
http://<host>:<port>/?id=<device_id>&lat=...&lon=...&timestamp=...
  &speed=...&bearing=...&altitude=...&hdop=...&satellites=...
  &batt=...&free_kb=...&distance=...
```

## Config Portal

Long press the button on GPIO10 to open the configuration portal:

- Connect to Wi-Fi AP: **ESP Nav** / password: **12345678**
- Open browser: **http://192.168.4.1**
- Add/remove Wi-Fi networks
- Set server host and port

## Dependencies

- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)
- Arduino framework for ESP32 (PlatformIO)

## License

LGPL3
