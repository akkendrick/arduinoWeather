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

// Define wifi parameters
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int INTERVAL = 1;

// Define weather variables
float bmeData[4];
#define BME_PWR 6
double US100_time;
double US100_distance;
#define US100_PWR 7
#define US100_TRIG 4
#define US100_ECHO 5

// Create an rtc object for tracking time
RTCZero rtc;

// MQTT properties
const char* mqtt_server = "192.168.50.100";  // IP address of the MQTT broker
const char* temp_topic = "outdoor/weather/temperature";
const char* humid_topic = "outdoor/weather/humidity";
const char* pressure_topic = "outdoor/weather/pressure";
const char* altitude_topic = "outdoor/weather/altitude";
const char* distance_topic = "outdoor/weather/distance";
const char* mqtt_username = MQTT_user; // MQTT username
const char* mqtt_password = MQTT_pass; // MQTT password
const char* clientID = "arduino"; // MQTT client ID

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;

// Connect to the computer running MQTT, 
// it is listening on port 1883 
PubSubClient client(mqtt_server, 1883, wifiClient);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;
unsigned int firstRun = 1;

// the US-100 module has the jumper cap on the back.
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
unsigned int Len_mm  = 0;


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
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void getBMEData(float bmeData[4]) {

    float bmeTemp;
    float bmeHumid;
    float bmeAlt;
    float bmePressure;

    bool BMEstarted = false;

    // Power ON the BME280 board
    pinMode(BME_PWR, OUTPUT);
    digitalWrite(BME_PWR, HIGH);
    delay(50);

    unsigned status;
    status = bme.begin();  

    int iter = 0;

    if (!status) {
      // Try restarting the BME 10 times
      while(iter < 10) {
        Serial.println("Stuck in BME Loop");
        // Try cycling BME280 power
        digitalWrite(BME_PWR, LOW);
        delay(50);
        digitalWrite(BME_PWR, HIGH);
        delay(50);
        iter++;

        // Update status each iteration
        status = bme.begin();
        if (status) {
          break;
        }
    }
 

    }
  
    // Check if the BME is connected
    if (status) {
        Serial.println("BME connected, getting measurements");

        bmeTemp = bme.readTemperature();
        bmeData[0] = bmeTemp;
  
        bmePressure = bme.readPressure() / 100.0F;
        bmeData[1] = bmePressure;
  
        bmeAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
        bmeData[2] = bmeAlt;
  
        bmeHumid = bme.readHumidity();
        bmeData[3] = bmeHumid;
      }
      else {
        Serial.println("BME not connected");

        bmeData[0] = 0;
        bmeData[1] = 0;
        bmeData[2] = 0;  
        bmeData[3] = 0;
      }

  

    
}

float getUS100Data()  {
  long timeHigh;

  // Power ON the US-100 board
  pinMode(US100_PWR, OUTPUT);
  digitalWrite(US100_PWR, HIGH);
  pinMode(US100_TRIG, OUTPUT);
  pinMode(US100_ECHO, INPUT);

  int val = 0;
  val = digitalRead(US100_PWR);   // read the  pin
  Serial.print("pin is:");
  Serial.println(val);
  delay(10);

  // Make sure this pin is low
  digitalWrite(US100_TRIG, LOW);
  delayMicroseconds(2);
  // Tell the US-100 to send an echo
  digitalWrite(US100_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US100_TRIG, LOW);
  
  // Measure how long it takes for the pulse to return
  // this is in microseconds
  timeHigh = pulseIn(US100_ECHO, HIGH);
  US100_time = timeHigh;

  return US100_time;
}

/****************************************
 * Main Functions
****************************************/
void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    
    rtc.begin(); // enable real time clock functionalities

    pinMode(LED_BUILTIN, OUTPUT);    
   

    Serial.println();

    pinMode(US100_PWR, OUTPUT);
    digitalWrite(US100_PWR, HIGH);
    pinMode(US100_TRIG, OUTPUT);
    pinMode(US100_ECHO, INPUT);

}


void loop() {

  float sleepTime;

  connect_MQTT();
  delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
  
  getBMEData(bmeData);
  delay(10); // This delay ensures that BME data is all good

  US100_time = getUS100Data();
  delay(10); // This delay ensures that US100 data is all good

  Serial.print("Time high is:");
  Serial.println(US100_time);

  // Use temperature to calculate speed of sound, this is in m/s
  double soundSpeed;
  soundSpeed = 331.3 + 0.606 * bmeData[0];
  Serial.print("Sound speed is:");
  Serial.println(soundSpeed);
  
  // Use US-100 measurement and speed of sound to calculate object distance
  // this is divided by two because the echo is bouncing off a surface
  // and returning to the sensor
  // soundSpeed is in m/s, US100 time in us, we want mm output
  US100_distance = soundSpeed * US100_time / 2000;

  Serial.print("The distance is:");
  Serial.println(US100_distance);



  // Publish all data to the MQTT Broker
  if (client.publish(temp_topic, String(bmeData[0]).c_str())) {
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(humid_topic, String(bmeData[3]).c_str());
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(altitude_topic, String(bmeData[2]).c_str());
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(pressure_topic, String(bmeData[1]).c_str());
    delay(10);
    client.publish(distance_topic, String(US100_distance).c_str());
    delay(10);

    // Sleep for 15 minutes if all is good
    sleepTime = 900000;

    Serial.println("Weather data sent!");
  }

  // client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will go to sleep, and try again in 5 minutes
  else {
    Serial.println("Weather data failed to send. Going to sleep and will just keep trying.");

    // Sleep for 5 minutes and try again
    sleepTime = 300000;
  }

  client.disconnect();  // disconnect from the MQTT broker
  
  WiFi.end(); //turn off wifi before sleep

  // Using built-in LED to indicate when it is awake
  digitalWrite(LED_BUILTIN, LOW);   

  Serial.println("Going to sleep");

  LowPower.sleep(int(sleepTime));
  
  digitalWrite(LED_BUILTIN, HIGH);    


}
