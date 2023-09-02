#include <WiFi.h>
#include <BH1750.h>
#include <Adafruit_VEML6075.h>
#include <WeatherCalculations.h>
#include <Adafruit_BME680.h>
#include <PMS7003-SOLDERED.h>
#include <ThingSpeak.h>

unsigned long two_second_jobs = 0, ten_second_jobs = 0, five_minute_jobs = 0;

const byte pollutants_TX = 18;
const byte pollutants_RX = 19;
const double altitude = XXX;  //Set your altitude, necessary for barometric pressure calculation
double temperature = 0, humidity = 0, seaLevelPressure = 0, gas = 0, heatIndex = 0, dewPoint = 0, lightIntensity = 0, UVindex = 0;
int PM01, PM25, PM10, AQI;
byte comfort = 1;             //Set the correct month! TO BE ADDED A REAL TIME CLOCK!
char comfortLevel[7][17] = { "Uncomfortable", "Comfortable", "Some discomfort", "Hot feeling", "Great discomfort", "Dangerous" };
double pressureData[288];
int pressureTrend = 3;
double pressureDifference = 0.5;  //Determines if the pressure is steady, falling or rising. If the pressure now is by 1.5 hPa greater or less than 3 hours ago, change is recorded, otherwise it is considered to be steady.
int month = 9;                    //Currently it's September

const char* SSID = "XXXXXXXXXXXX";               //Replace with your own WiFI SSID
const char* password = "XXXXXXXXXXXX";           //Replace with your WiFi network's password
const unsigned long channelID = XXXXXX;          //Replace with your ThingSpeak channel's ID
const char* APIkey = "XXXXXXXXXXXX";             //Replace with your ThingSpeak API key

WiFiServer server(80);
WiFiClient client;
Adafruit_BME680 bme;
PMS7003 pollutants(pollutants_RX, pollutants_TX);
Adafruit_VEML6075 uv = Adafruit_VEML6075();
BH1750 light;
Weather weather;

void setup() {
  bme.begin();
  /* Default settings from datasheet. */
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms

  pollutants.begin();
  uv.begin();
  light.begin();
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(SSID, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  server.begin();
  ThingSpeak.begin(client);
}

void loop() {
  if (millis() - ten_second_jobs >= 10000) {
    pollutants.read();

    PM01 = pollutants.pm01;
    PM25 = pollutants.pm25;
    PM10 = pollutants.pm10;
    AQI = weather.getAQI(PM25, PM10);

    ten_second_jobs = millis();
  }

  if (millis() - two_second_jobs >= 2000) {
    bme.performReading();

    temperature = bme.temperature;
    humidity = bme.humidity;
    seaLevelPressure = weather.getSeaLevelPressure(bme.pressure / 100.0, altitude);
    gas = bme.gas_resistance / 1000.0;
    dewPoint = weather.getDewPoint(temperature, humidity);
    heatIndex = weather.getHeatIndex(temperature, humidity);
    comfort = weather.getComfort(heatIndex);
    UVindex = uv.readUVI();
    lightIntensity = light.readLightLevel();

    humidity = rounding(humidity, 1);
    temperature = rounding(temperature, 1);
    seaLevelPressure = rounding(seaLevelPressure, 1);
    gas = rounding(gas, 1);
    dewPoint = rounding(dewPoint, 1);
    heatIndex = rounding(heatIndex, 1);
    UVindex = rounding(UVindex, 1);
    lightIntensity = rounding(lightIntensity, 1);

    two_second_jobs = millis();
  }

  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println(F("HTTP/1.1 200 OK"));
            client.println(F("Content-type:text/html"));
            client.println(F("Connection: close"));
            client.println();

            //Here goes the HTML code
            client.println(F("<!DOCTYPE html><html>"));
            client.println(F("<head><meta charset=\"UTF-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\"><meta http-equiv=\"refresh\" content=\"2\"><title>ESP32 Weather Station</title></head>"));
            client.println(F("<body><center><h1>ESP32 Weather Station</h1></center><center>"));

            client.print(F("<p>Temperature: "));
            client.print(temperature, 1);
            client.println(F(" °C</p>"));

            client.print(F("<p>Humidity: "));
            client.print(humidity, 1);
            client.println(F(" %</p>"));

            client.print(F("<p>Pressure: "));
            client.print(seaLevelPressure, 1);
            client.println(F(" hPa</p>"));

            client.print(F("<p>Pressure Trend: "));
            if (pressureTrend == 1) {
              client.print(F("Rising"));
            } else if (pressureTrend == 2) {
              client.print(F("Falling"));
            } else if (pressureTrend == 3) {
              client.print(F("Steady"));
            } else {
              client.print(F("Invalid!"));
            }
            client.println(F("</p>"));

            client.print(F("<p>Pressure 3 Hours Ago: "));
            client.print(pressureData[35], 1);
            client.println(F(" hPa</p>"));

            client.print(F("<p>Pressure 6 Hours Ago: "));
            client.print(pressureData[71], 1);
            client.println(F(" hPa</p>"));

            client.print(F("<p>Pressure 12 Hours Ago: "));
            client.print(pressureData[143], 1);
            client.println(F(" hPa</p>"));

            client.print(F("<p>Pressure 24 Hours Ago: "));
            client.print(pressureData[287], 1);
            client.println(F(" hPa</p>"));

            client.print(F("<p>Dew Point: "));
            client.print(dewPoint, 1);
            client.println(F(" °C</p>"));

            client.print(F("<p>Heat Index: "));
            client.print(heatIndex, 1);
            client.println(F(" °C</p>"));

            client.print(F("<p>UV Index: "));
            client.println(UVindex, 1);
            client.println(F("</p>"));

            client.print(F("<p>Light Intensity: "));
            client.print(lightIntensity, 1);
            client.println(F(" lux</p>"));

            client.print(F("<p>Gas: "));
            client.print(gas, 1);
            client.println(F(" kOhms</p>"));

            client.print(F("<p>PM10: "));
            client.print(PM10);
            client.println(F(" ug/m3</p>"));

            client.print(F("<p>PM2.5: "));
            client.print(PM25);
            client.println(F(" ug/m3</p>"));

            client.print(F("<p>PM1: "));
            client.print(PM01);
            client.println(F(" ug/m3</p>"));

            client.print(F("<p>AQI: "));
            client.print(AQI);
            client.println(F("</p>"));

            client.print(F("<p>Comfort Level: "));
            client.println(comfortLevel[comfort - 1]);
            client.println(F("</p>"));

            client.print(F("<p>12-hour Forecast: "));
            client.println(weather.getForecast(seaLevelPressure, month, "NOW", pressureTrend));
            client.println(F("</p>"));

            client.println(F("</body></html>"));

            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }

  if (millis() - five_minute_jobs >= 300000) {  //5 minute update interval

    rightShiftArray(pressureData, sizeof(pressureData) / sizeof(pressureData[0]));
    pressureData[0] = seaLevelPressure;

    if ((pressureData[0] - pressureData[35] >= pressureDifference) && (pressureData[35] != 0.0) && (pressureData[0] != 0.0)) {
      pressureTrend = 1;
    } else if ((pressureData[0] - pressureData[35] <= -1 * pressureDifference) && (pressureData[35] != 0.0) && (pressureData[0] != 0.0)) {
      pressureTrend = 2;
    } else {
      pressureTrend = 3;
    }

    ThingSpeak.setField(1, (float)temperature);
    ThingSpeak.setField(2, (float)humidity);
    ThingSpeak.setField(3, (float)seaLevelPressure);
    ThingSpeak.setField(4, (float)dewPoint);
    ThingSpeak.setField(5, (float)heatIndex);
    ThingSpeak.setField(6, (float)UVindex);
    ThingSpeak.setField(7, (float)lightIntensity);
    ThingSpeak.setStatus(String(weather.getForecast(seaLevelPressure, month, "NOW", pressureTrend)));

    ThingSpeak.writeFields(channelID, APIkey);

    five_minute_jobs = millis();
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
      wifiOnDisconnect();
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      WiFi.enableIpV6();  //Enable IPv6 support for ESP32
      break;
    default:
      break;
  }
}

void wifiOnDisconnect() {
  delay(1000);
  WiFi.begin(SSID, password);
}

void rightShiftArray(double array[], const int size) {
  for (int i = size - 1; i > 0; i--) {
    array[i] = array[i - 1];
  }
}

double rounding(const double number, const int decimals) {
  double factor = pow(10, decimals);
  return round(number * factor) / factor;
}
