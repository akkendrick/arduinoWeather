mport paho.mqtt.client as mqtt
from prometheus_client import start_http_server, Summary, Gauge
import time
import datetime

temp = Gauge('temperature', 'Weather Station Temperature')
hum = Gauge('humidity', 'Weather Station Humidity')
alt = Gauge('altitude', 'Weather Station Altitude')
pres = Gauge('pressure', 'Weather Station Pressure')

############ modify this for your config ##############
MQTT_ADDRESS = 
MQTT_USER = ''
MQTT_PASSWORD = ''
MQTT_TOPIC = 'outdoor/weather/temperature'
MQTT_REGEX = 'home/([^/]+)/([^/]+)'
MQTT_CLIENT_ID = 'raspberrypi'

def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))
    client.subscribe([('outdoor/weather/temperature',1),('outdoor/weather/humidity',1),
                      ('outdoor/weather/altitude',1),('outdoor/weather/pressure',1)])

def process_request(msg):
    """A function to read wifi."""
    print("Current Time:",datetime.datetime.now())
    print(msg.topic + ' ' + str(msg.payload))

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
    """The callback for when a PUBLISH message is received from the server."""
    process_request(msg)

def main():
    start_http_server(8000)

    mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(MQTT_ADDRESS, 1883)
    mqtt_client.loop_forever()


if __name__ == '__main__':
    main()