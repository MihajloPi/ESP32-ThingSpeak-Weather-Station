# ESP32 ThingSpeak Weather Station

A feature-rich Arduino-based weather station built around the ESP32 that measures a wide range of environmental parameters, serves them on a local web page, and uploads them to the [ThingSpeak](https://thingspeak.com) IoT cloud for logging and visualization. The firmware also supports over-the-air (OTA) updates so the device can be reflashed without a USB connection.

## Features

- **Temperature, humidity, barometric pressure, and gas (VOC) resistance** via the Bosch BME680
- **Sea-level pressure compensation** based on a user-configured altitude
- **Pressure trend detection** (rising / falling / steady) using a 24-hour rolling buffer of readings sampled every 5 minutes
- **12-hour weather forecast** derived from current pressure, pressure trend, and the season (Zambretti-style algorithm in the `WeatherCalculations` library)
- **Dew point, heat index, and human comfort level** calculations
- **UV index** via the VEML6075
- **Ambient light intensity (lux)** via the BH1750
- **Particulate matter (PM1.0, PM2.5, PM10) and Air Quality Index (AQI)** via the PMS7003
- **Local web dashboard** served on port 80, auto-refreshing every 60 seconds, showing all live readings plus pressure values from 3, 6, 12, and 24 hours ago
- **ThingSpeak cloud upload** every 5 minutes (8 fields + status message with the forecast)
- **Arduino OTA** firmware updates with password protection
- **IPv6 enabled** on the Wi-Fi station interface
- **Automatic Wi-Fi reconnection** on disconnect

## Hardware

| Component | Purpose |
|---|---|
| ESP32 development board | Main microcontroller (Wi-Fi + processing) |
| Bosch BME680 | Temperature, humidity, pressure, gas |
| VEML6075 | UV index (UVA / UVB) |
| BH1750 | Ambient light (lux) |
| Plantower PMS7003 | Particulate matter (PM1.0 / PM2.5 / PM10) |

The BME680, VEML6075, and BH1750 share the I²C bus (default ESP32 pins: SDA = GPIO 21, SCL = GPIO 22). The PMS7003 is connected to a software/hardware serial port using:

- `pollutants_TX` = GPIO 18
- `pollutants_RX` = GPIO 19

## Required Libraries

Install the following from the Arduino Library Manager (or from the linked sources):

- `WiFi` and `ArduinoOTA` (bundled with the ESP32 core)
- `Adafruit_BME680`
- `Adafruit_VEML6075`
- `BH1750`
- `PMS7003-SOLDERED` ([Soldered Electronics](https://github.com/SolderedElectronics))
- `ThingSpeak` (by MathWorks)
- [`WeatherCalculations`](https://github.com/MihajloPi/WeatherCalculations) — my own library, provides the `Weather` class used for sea-level pressure, dew point, heat index, comfort level, AQI, and Zambretti-style forecasting

You will also need the **ESP32 board support package** installed in the Arduino IDE (Boards Manager → "esp32" by Espressif Systems).

## Configuration

Before flashing, open `main.ino` and replace the placeholder values near the top of the file:

```cpp
const double altitude        = XXX;            // Your altitude above sea level in meters
const char*  SSID            = "XXXXXXXXXXXX"; // Your Wi-Fi SSID
const char*  password        = "XXXXXXXXXXXX"; // Your Wi-Fi password
const unsigned long channelID = XXXXXXXXXXXX;  // Your ThingSpeak channel ID
const char*  APIkey          = "XXXXXXXXXXXX"; // Your ThingSpeak Write API key
const char*  OTApassword     = "admin";        // Change this!
const char*  WXStationHostname = "esp32-wxstation";
const uint8_t month          = 6;              // Current month (used by the forecast algorithm)
```

> **Note:** the firmware does not currently include a real-time clock, so the `month` constant must be set manually before flashing if you want accurate seasonal forecasting. Adding an RTC or NTP sync is on the to-do list.

### Setting up ThingSpeak

1. Create a free account at [thingspeak.com](https://thingspeak.com).
2. Create a new channel with **8 fields** in this exact order:
   1. Temperature (°C)
   2. Humidity (%)
   3. Sea-level Pressure (hPa)
   4. Dew Point (°C)
   5. Heat Index (°C)
   6. UV Index
   7. Light Intensity (lux)
   8. AQI
3. Copy the **Channel ID** and **Write API Key** into `main.ino`.

## Building and Flashing

1. Open `main.ino` in the Arduino IDE.
2. Select your ESP32 board under **Tools → Board**.
3. Select the correct serial port.
4. Click **Upload**.

After the first upload, subsequent updates can be performed wirelessly: the device will appear as a network port named `esp32-wxstation` in the Arduino IDE as long as it's on the same network.

## Usage

Once the ESP32 is running and connected to Wi-Fi, it will print its IP address to the serial monitor at boot. Open that address in any web browser on the same network to see the live dashboard. The page auto-refreshes every 60 seconds and shows:

- Current temperature, humidity, pressure, dew point, heat index
- Pressure values from 3, 6, 12, and 24 hours ago
- Pressure trend (rising / falling / steady)
- UV index, light intensity, gas resistance
- PM1.0, PM2.5, PM10, and AQI
- Comfort level
- 12-hour forecast

In parallel, every 5 minutes the device pushes the eight measurement fields plus the forecast string to ThingSpeak, where you can build long-term graphs, alerts, or MATLAB analyses.

## Update Intervals

| Task | Interval |
|---|---|
| BME680, UV, light readings + derived values | 2 seconds |
| PMS7003 particulate readings | 10 seconds |
| Pressure history shift + ThingSpeak upload | 5 minutes |
| Web page auto-refresh (browser-side) | 60 seconds |

The pressure history buffer holds 288 samples (24 hours × 12 samples/hour), which is also what enables the "pressure N hours ago" readings on the dashboard.

## Known Limitations / To-Do

- No real-time clock — the `month` constant has to be set manually before flashing for the seasonal forecast to be accurate. Adding NTP sync would fix this.
- The local web server is single-client and blocking; it's fine for occasional checks but not designed for many concurrent users.
- No persistent storage of historical data — the pressure buffer lives in RAM and is lost on reboot.
- Default OTA password is `admin`; change it before deploying.

## License

This project is released under the **GNU General Public License v3.0**. See the [LICENSE](LICENSE) file for the full text.
