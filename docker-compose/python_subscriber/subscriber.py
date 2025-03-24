import json
import os
import paho.mqtt.client as mqtt
import psycopg2
from datetime import datetime
import time
import logging
import queue

# Set up logging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

# Environment variables for configuration
MQTT_BROKER_HOST = os.environ.get("MQTT_HOST")
MQTT_BROKER_PORT = int(os.environ.get("MQTT_PORT"))
DB_HOST = os.environ.get("DB_HOST", "localhost")
DB_NAME = os.environ.get("DB_NAME", "")
DB_USER = os.environ.get("DB_USER", "")
DB_PASSWORD = os.environ.get("DB_PASSWORD", "")
# print mqtt 
queue = queue.Queue()


# if cant connect wait 50 seconds and try again
while True:
    try:
        conn = psycopg2.connect(dbname=DB_NAME, user=DB_USER,
                        password=DB_PASSWORD, host=DB_HOST)
        logger.info("Connected to DB")
        break
    except psycopg2.OperationalError:
        logger.info("Waiting for DB to be ready...")
        time.sleep(50)

def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        # get the last part of the topic and use it as the sensor id if can not be converted use 0
        sensor_id = int(topic.split("/")[-1]) if topic.split("/")[-1].isdigit() else 0
        data = json.loads(msg.payload.decode())
        temperature = data.get('temperature')
        humidity = data.get('humidity')
        co2 = data.get('co2')
        timestamp = datetime.fromtimestamp(data.get('timestamp'))
        if timestamp is None or temperature is None or humidity is None or co2 is None:
            logger.error(f"Invalid data: {data}")
            return
        logger.debug(f"Received data: {data}")
        queue.put({"temperature": temperature, "humidity": humidity, "co2": co2, "timestamp": timestamp, "sensor_id": sensor_id})
        logger.debug(f"Queue size: {queue.qsize()}")
        if queue.qsize() >= 50:
            for _ in range(queue.qsize()):
                insert_data(queue)
        
        
    except Exception as e:
        logger.error("Error processing message:" +str(e))
        
def on_connect(client, userdata, flags, rc, properties=None):
    logger.info(f"Connected with result code {rc}")
    client.subscribe("home/sensors/#")
    
# insert data to the database
def insert_data(q):
    try:
        with conn.cursor() as cur:
            for _ in range(q.qsize()):
                data = q.get()
                temperature = data.get('temperature')
                humidity = data.get('humidity')
                co2 = data.get('co2')
                timestamp = data.get('timestamp')
                sensor_id = data.get('sensor_id')
                cur.execute(
                    "INSERT INTO sensor_data (sensor_id, timestamp, temperature, humidity, co2) VALUES (%s, %s, %s, %s, %s)",
                    (sensor_id, timestamp, temperature, humidity, co2)
                )
                logger.debug(f"Inserted data: {data}")
            conn.commit()
            
    except Exception as e:
        logger.error("Error processing message:" + str(e))
        conn.rollback()

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.enable_logger(logging.getLogger(__name__))
client.on_connect = on_connect
client.on_message = on_message

result = client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
if result != 0:
    logger.error(f"Failed to connect to broker: {result}")
else:
    logger.info("Connected to broker")
try: 
    client.loop_forever()
except:
    logger.info("Stopping subscriber.")
finally:
    client.disconnect()
    if queue.qsize() > 0:
        insert_data(queue)
    conn.close()
    logger.info("Disconnected from broker and DB")