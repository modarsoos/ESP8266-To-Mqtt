#include <Arduino.h>
#include <U8g2lib.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define RX_PIN D3                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN D4                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

#define SEALEVELPRESSURE_HPA (1014)
#define MQTT_TOPIC_PRESSURE "home/ElternZimmer/bme280/pressure"
#define MQTT_TOPIC_HUMIDITY "home/ElternZimmer/bme280/humidity"
#define MQTT_TOPIC_TEMPERATURE "home/ElternZimmer/bme280/temperature"
#define MQTT_TOPIC_Altitude "home/ElternZimmer/bme280/Altitude"
#define MQTT_TOPIC_CO2 "home/ElternZimmer/mhz19/co2"
#define MQTT_TOPIC_STATE "home/ElternZimmer/bme280/status"
#define MQTT_PUBLISH_DELAY 600000
#define MQTT_CLIENT_ID "ESP8266_ElternZimmer"

#define BME280_ADDRESS 0x76
MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial
long lastMsgTime = 0;
unsigned long getDataTimer = 0;
float humidity;
float temperature;
float pressure;
float Altitude;

const char *WIFI_SSID = "Your Wifi Name";
const char *WIFI_PASSWORD = "Your Wifi Password";

const char *MQTT_SERVER = "192.168.178.106";
const char *MQTT_USER = "MQTT Username here"; // NULL for no authentication
const char *MQTT_PASSWORD = "MQTT Password here"; // NULL for no authentication

WiFiClient espClient;
PubSubClient mqttClient(espClient);

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //********************HW is faster than SW*************************************
Adafruit_BME280 bme;

void setup()
{
  Serial.begin(9600);                                     // Device to serial monitor feedback
  u8g2.begin();

  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or BME-280 address!");
    while (1);
  }

  // Use force mode so that the sensor returns to sleep mode when the measurement is finished
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);

  mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start
  myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin().

  myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
  setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);
}

void loop()
{
  if (millis() - getDataTimer >= MQTT_PUBLISH_DELAY)
  {
    int CO2;

    /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even
      if below background CO2 levels or above range (useful to validate sensor). You can use the
      usual documented command with getCO2(false) */

    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)

    //Serial.print("CO2 (ppm): ");
    //Serial.println(CO2);

    int8_t Temp;
    Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
    //Serial.print("Temperature (C): ");
    //Serial.println(Temp);

    getDataTimer = millis();

    if (!mqttClient.connected()) {
      mqttReconnect();
    }
    mqttClient.loop();

    long now = millis();
    if (now - lastMsgTime > MQTT_PUBLISH_DELAY) {
      lastMsgTime = now;

      // Reading BME280 sensor data
      bme.takeForcedMeasurement(); // has no effect in normal mode
      humidity = bme.readHumidity();
      temperature = bme.readTemperature();
      pressure = bme.readPressure() / 100.0F;
      Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

      if (isnan(humidity) || isnan(temperature)) {
        Serial.println("BME280 reading issues");
        return;
      }

      // Publishing sensor data
      mqttPublish(MQTT_TOPIC_TEMPERATURE, temperature);
      mqttPublish(MQTT_TOPIC_HUMIDITY, humidity);
      mqttPublish(MQTT_TOPIC_PRESSURE, pressure);
      mqttPublish(MQTT_TOPIC_Altitude, Altitude);
      mqttPublish(MQTT_TOPIC_CO2, CO2);
    }

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_tenfatguys_tr);
      u8g2.setCursor(0, 10);
      u8g2.print("CO2");
      u8g2.setFont(u8g2_font_fub35_tn);  //Best bold number font for ssd1306 128 x 32
      u8g2.setCursor(0, 58);
      u8g2.print(CO2);
      u8g2.setFont(u8g2_font_tenthinnerguys_tr   );
      u8g2.setCursor(50, 10);               // Best center text for 128 x 64
      u8g2.print("ppm");
      u8g2.setFont(u8g2_font_open_iconic_human_2x_t);
      u8g2.drawGlyph(100, 16, 0x0042);  /* dec 66/hex 0042 Heart */
    } while ( u8g2.nextPage() );
  }
}

void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);    //https://github.com/knolleary/pubsubclient/issues/138#issuecomment-326113915
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttPublish(char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}
