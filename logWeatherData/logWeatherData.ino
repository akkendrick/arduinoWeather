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

// the US-100 module has jumper cap on the back.
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
unsigned int Len_mm  = 0;

int US100_temp = 0;


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

void getUS100data(US100Data[2])  {
  Serial1.begin(9600);    
  while(!Serial1)

  while(Serial1.available())
   Serial1.read();    

  Serial1.write(0X55);                           // trig US-100 begin to measure the distance
    delay(500);                                   // delay 500ms to wait result
  
  if(Serial1.available() >= 2)                   // when receive 2 bytes 
    {
        HighLen = Serial1.read();                   // High byte of distance
        LowLen  = Serial1.read();                   // Low byte of distance
        Len_mm  = HighLen*256 + LowLen;            // Calculate the distance
        if((Len_mm > 1) && (Len_mm < 10000))       // normal distance should between 1mm and 10000mm (1mm, 10m)
        {
            Serial.print("Present Length is: ");   // output the result to serial monitor
            Serial.print(Len_mm, DEC);             // output the result to serial monitor
            Serial.println("mm");                  // output the result to serial monitor
        }
    }

    delay(500); 

    while(Serial1.available())
    Serial1.read();                   
                                       // wait 500ms
    Serial1.write(0X50);   // trig US-100 begin to measure the temperature
    delay(500);            //delay 500ms to wait result
    if(Serial1.available() >= 1)            //when receive 1 bytes 
    {
        US100_temp = Serial1.read();     //Get the received byte (temperature)
        if((US100_temp > 1) && (US100_temp < 130))   //the valid range of received data is (1, 130)
        {
            US100_temp -= 45;                           //Real temperature = Received_Data - 45
            Serial.print("Present Temperature is: ");          //output result
            Serial.print(US100_temp, DEC);             //output result
            Serial.println(" degree centigrade.");        //output result
        }
    }
    delay(500);          
}

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

    Serial1.begin(9600);    
    while(!Serial1);    

    while(Serial1.available())
    Serial1.read();    
    
}


void loop() {

  connect_MQTT();
  delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
  
  getBMEData(bmeData);
    delay(10); // This delay ensures that BME data upload is all good


  // MQTT can only transmit strings
 
  String bmeTempStr=": "+String((float)bmeData[0])+" % ";
  String bmeHumidStr=": "+String((float)bmeData[3])+" % ";
  String bmeAltStr=": "+String((float)bmeData[2])+" % ";
  String bmePressureStr=": "+String((float)bmeData[1])+" % ";


  // PUBLISH to the MQTT Broker (topic = Humidity, defined at the beginning)
  if (client.publish(temp_topic, String(bmeTemp).c_str())) {
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(humid_topic, String(bmeHumid).c_str());
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(altitude_topic, String(bmeAlt).c_str());
    delay(10); // This delay ensures that BME data upload is all good
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
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(humid_topic, String(bmeHumid).c_str());
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(altitude_topic, String(bmeAlt).c_str());
    delay(10); // This delay ensures that BME data upload is all good
    client.publish(pressure_topic, String(bmePressure).c_str());
  }
  client.disconnect();  // disconnect from the MQTT broker
  
  WiFi.end(); //turn off wifi before sleep

  delay(5000);   
  digitalWrite(LED_BUILTIN, LOW);   

  Serial.println("Going to sleep");

  //float sleepTime = 900000; // Set to 15 minutes currently
  LowPower.sleep(900000);


  digitalWrite(LED_BUILTIN, HIGH);    


}
