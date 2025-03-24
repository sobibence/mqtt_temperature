import json
import time
import random
import paho.mqtt.client as mqtt
import logging
import math

# MQTT broker details
MQTT_BROKER_HOST = "localhost"  # Adjust if your broker hostname is different
MQTT_BROKER_PORT = 1883
MQTT_TOPIC = "home/sensors/1"

# Enable logging
logging.basicConfig(level=logging.DEBUG)

def onconnect(client, userdata, flags, reasonCode, properties=None):
    print(f"Connected with result code {reasonCode}")
    

# Create the MQTT client instance
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.enable_logger(logging.getLogger(__name__))

client.on_connect = onconnect

# Connect to the broker
client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)

# Start the network loop in a separate thread
client.loop_start()

def send_sensor_data():
    # check connection
    if not client.is_connected():
        print("Reconnecting to the broker...")
        result = client.reconnect()
        print(f"Reconnection result: {result}")
    
    # Simulate sensor data with sin and cos functions with a period of one hour
    data = {
        "temperature": math.sin(time.time() / 3600) * 10 + 20,
        "humidity": math.cos(time.time() / 1700) * 10 + 50,
        "co2":  math.cos(time.time() / 3000) * 100 + 500,
        "timestamp": time.time()
    }
    payload = json.dumps(data)
    # Publish the data to the topic
    errorcode = client.publish(MQTT_TOPIC, payload)
    
    if errorcode[0] != 0:
        print(f"Failed to publish message: {payload}, {errorcode}")
    else:
        print(f"Published: {payload}, {errorcode}")
    

print("Waiting for connection", end="")
try:
    while True:
        send_sensor_data()
        time.sleep(5)  # Wait 5 seconds between messages
except KeyboardInterrupt:
    print("Stopping publisher.")
finally:
    client.loop_stop()
    client.disconnect()
