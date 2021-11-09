import paho.mqtt.client as mqtt
from prometheus_client import start_http_server, Summary, Gauge
import time
import datetime
import sqlite3
import os
from dotenv import load_dotenv

load_dotenv()

############ modify this for your prometheus variables #############
# There are different types of variables accepted by prometheus,
# since these are all point measurements in time, Gauge is the right choice
temp = Gauge('temperature', 'Weather Station Temperature')
hum = Gauge('humidity', 'Weather Station Humidity')
alt = Gauge('altitude', 'Weather Station Altitude')
pres = Gauge('pressure', 'Weather Station Pressure')
#####################################################################

############ modify this for your mqtt config ##############
MQTT_ADDRESS = os.environ.get("MQTT_ADDRESS") 
MQTT_USER = os.environ.get("MQTT_USER") 
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD") 
MQTT_TOPIC = 'outdoor/weather/temperature'
MQTT_REGEX = 'home/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'raspberrypi'
########################################################

def on_connect(client, userdata, flags, rc):
    """ Run the following when a client connects"""
    # There are various mqtt connect codes, only needed if debugging
    print('Connected with result code ' + str(rc)) 
    # Subscribe mqtt to the following variables
    client.subscribe([('outdoor/weather/temperature',1),('outdoor/weather/humidity',1),
                      ('outdoor/weather/altitude',1),('outdoor/weather/pressure',1)])

def process_request(msg):
    """A function to read the published data over mqtt."""
    timedat = datetime.datetime.now()
    # Print the timestep to make sure it is working now 
    print("Current Time:",datetime.datetime.now())
    # Print the message 
    print(msg.topic + ' ' + str(msg.payload))

    # Save the weather data into a database
    cur.execute('INSERT INTO weatherData (timedat, name, value) values (str(timedat), str(msg.payload),str(msg.payload))')
    conn.commit()

    # Make sure we associate prometheus logs with the correct mqtt variable
    # This publishes the mqtt variables to a prometheus gauge
    if msg.topic ==  'outdoor/weather/temperature':
        temp.set(msg.payload)
    elif msg.topic == 'outdoor/weather/humidity':
        hum.set(msg.payload)
    elif msg.topic == 'outdoor/weather/altitude':
        alt.set(msg.payload)
    elif msg.topic == 'outdoor/weather/pressure':
        pres.set(msg.payload)
    else:
        print('Incorrect topic')

def on_message(client, userdata, msg):
    """ Run the following command when a message is received""" 
    process_request(msg)

def setup_database():
    """Set up the database for storing the data sent by mqtt"""
    conn = sqlite3.connect('YOUR PATH')
    cur = conn.cursor()
    cur.execute('CREATE TABLE weatherData (timedat VARCHAR, name VARCHAR, value VARCHAR)')
    conn.commit()

def main():
    # Start the Prometheus server
    start_http_server(8000)
    # Setup the SQLlite database
    setup_database()

    # Start mqtt client
    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)

    # Specify what programs to call when mqtt conditions are met
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # Setup mqtt on a port
    mqtt_client.connect(MQTT_ADDRESS, 1883)
    # Keep running forever
    mqtt_client.loop_forever()


if __name__ == '__main__':
    main()