#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include "certificates.h"
#include "config.h"
#include <stdio.h>
#include "PubSubClient.h"
#include <ArduinoLowPower.h>
#include <RTCZero.h>


#include <WiFiNINA.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int INTERVAL = 1;
float bmeData[4];
RTCZero rtc;

float bmeTemp;
float bmeHumid;
float bmeAlt;
float bmePressure;

#define BME_PWR 6

// MQTT
const char* mqtt_server = "192.168.50.100";  // IP of the MQTT broker
const char* temp_topic = "outdoor/weather/temperature";
const char* humid_topic = "outdoor/weather/humidity";
const char* pressure_topic = "outdoor/weather/pressure";
const char* altitude_topic = "outdoor/weather/altitude";
const char* mqtt_username = "akendrick"; // MQTT username
const char* mqtt_password = "akendrick"; // MQTT password
const char* clientID = "arduino"; // MQTT client ID

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
unsigned int firstRun = 1;

/****************************************
* Auxiliary Functions
****************************************/



void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);


  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging - Output the IP Address
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
// Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void getBMEData(float bmeData[4]) {
      bool BMEstarted = false;

    // Power ON the BME280 board
    pinMode(BME_PWR, OUTPUT);
    digitalWrite(BME_PWR, HIGH);
    delay(50);

    unsigned status;
    status = bme.begin();  

    while(!status) {
     Serial.println("Stuck in BME While");
     // Try cycling BME280 power
        digitalWrite(BME_PWR, LOW);
        delay(50);
        digitalWrite(BME_PWR, HIGH);
        delay(50);
        status = bme.begin();
    }

        bmeTemp = bme.readTemperature();
        bmeData[0] = bmeTemp;
  
        bmePressure = bme.readPressure() / 100.0F;
        bmeData[1] = bmePressure;
  
        bmeAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
        bmeData[2] = bmeAlt;
  
        bmeHumid = bme.readHumidity();
        bmeData[3] = bmeHumid;
  

    
}

/****************************************
 * Main Functions
****************************************/
void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    Serial.println(F("Send data test"));
    
    rtc.begin(); // enable real time clock functionalities

    pinMode(LED_BUILTIN, OUTPUT);    
   

    Serial.println();
}


void loop() {
  Serial.println("Looping");
  connect_MQTT();
  delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
  
  getBMEData(bmeData);
    delay(10); // This delay ensures that BME data upload is all good


  // MQTT can only transmit strings

  bmeHumid = bmeData[3];
  bmeTemp = bmeData[0];
  
  String bmeTempStr=": "+String((float)bmeTemp)+" % ";
  String bmeHumidStr=": "+String((float)bmeHumid)+" % ";
  String bmeAltStr=": "+String((float)bmeData[2])+" % ";
  String bmePressureStr=": "+String((float)bmeData[1])+" % ";


  // PUBLISH to the MQTT Broker (topic = Humidity, defined at the beginning)
  if (client.publish(temp_topic, String(bmeTemp).c_str())) {
    client.publish(humid_topic, String(bmeHumid).c_str());
    client.publish(altitude_topic, String(bmeAlt).c_str());
    client.publish(pressure_topic, String(bmePressure).c_str());

    Serial.println("Weather data sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Wifi Strength failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(temp_topic, String(bmeTemp).c_str());
    client.publish(humid_topic, String(bmeHumid).c_str());
    client.publish(altitude_topic, String(bmeAlt).c_str());
    client.publish(pressure_topic, String(bmePressure).c_str());
  }
  client.disconnect();  // disconnect from the MQTT broker
  
  WiFi.end(); //turn off wifi before sleep

  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  delay(1000);   
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);    
  Serial.println("Going to sleep");

  //float sleepTime = 900000; // Set to 15 minutes currently
  LowPower.sleep(900000);


  Serial.println("Wake up?");



}
