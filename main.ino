#include <WiFi.h>
#include <BH1750.h>
#include <Adafruit_VEML6075.h>
#include <WeatherCalculations.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <ThingSpeak.h>

unsigned long sensorTimer, ThingSpeakTimer = 0;

#define dhtType DHT22
const byte dhtPin = 19;
const float altitude = XXX;                      //Set your altitude, necessary for barometric pressure calculation
float temperature, humidity, seaLevelPressure, heatIndex, dewPoint, lightIntensity, UVindex = 0;
byte comfort = 1;
char comfortLevel[7][17] = {"Uncomfortable", "Comfortable", "Some discomfort", "Hot feeling", "Great discomfort", "Dangerous"};

const char* SSID = "XXXXXXXXXXXX";               //Replace with your own WiFI SSID
const char* password = "XXXXXXXXXXXX";           //Replace with your WiFi network's password
const unsigned long channelID = XXXXXX;          //Replace with your ThingSpeak channel's ID
const char* APIkey = "XXXXXXXXXXXX";             //Replace with your ThingSpeak API key

WiFiServer server(80);
WiFiClient client;
DHT dht(dhtPin, dhtType);
Adafruit_BMP280 bmp;
Adafruit_VEML6075 uv = Adafruit_VEML6075();
BH1750 light;
Weather weather;

void setup() {
  dht.begin();
  bmp.begin();
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
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
  if (millis() - sensorTimer >= 2000) {
    humidity = dht.readHumidity();
    temperature = bmp.readTemperature();
    seaLevelPressure = weather.getSeaLevelPressure(bmp.readPressure() / 100.0, altitude);
    dewPoint = weather.getDewPoint(temperature, humidity);
    heatIndex = weather.getHeatIndex(temperature, humidity);
    comfort = weather.getComfort(heatIndex);
    UVindex = uv.readUVI();
    lightIntensity = light.readLightLevel();

    sensorTimer = millis();
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

            //Here goes HTML code
            client.println(F("<!DOCTYPE html><html>"));
            client.println(F("<head><meta charset=\"UTF-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\"><meta http-equiv=\"refresh\" content=\"2\"></head>"));
            client.println(F("<body><center><h1>ESP32 Weather Station</h1></center>"));

            client.print(F("<p>Temperature: "));
            client.print(temperature, 1);
            client.println(F(" °C</p>"));

            client.print(F("<p>Humidity: "));
            client.print(humidity, 1);
            client.println(F(" %</p>"));

            client.print(F("<p>Pressure: "));
            client.print(seaLevelPressure, 1);
            client.println(F(" hPa</p>"));

            client.print(F("<p>Dew point: "));
            client.print(dewPoint, 1);
            client.println(F(" °C</p>"));

            client.print(F("<p>Heat Index: "));
            client.print(heatIndex, 1);
            client.println(F(" °C</p>"));

            client.print(F("<p>UV index: "));
            client.println(UVindex, 1);
            client.println(F("</p>"));

            client.print(F("<p>Light intensity: "));
            client.print(lightIntensity);
            client.println(F(" lux</p>"));

            client.print(F("<p>Comfort: "));
            client.println(comfortLevel[comfort - 1]);
            client.println(F("</p>"));

            client.println(F("</body></html>"));

            client.println();
            break;
          }
          else {
            currentLine = "";
          }
        }
        else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }

  if (millis() - ThingSpeakTimer >= 300000) {       //5 minute update interval
    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, humidity);
    ThingSpeak.setField(3, seaLevelPressure);
    ThingSpeak.setField(4, dewPoint);
    ThingSpeak.setField(5, heatIndex);
    ThingSpeak.setField(6, UVindex);
    ThingSpeak.setField(7, lightIntensity);
    ThingSpeak.setStatus(comfortLevel[comfort - 1]);

    ThingSpeak.writeFields(channelID, APIkey);

    ThingSpeakTimer = millis();
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
      wifiOnDisconnect();
      break;
    default:
      break;
  }
}

void wifiOnDisconnect() {
  delay(1000);
  WiFi.begin(SSID, password);
}
