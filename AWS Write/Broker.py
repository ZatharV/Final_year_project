import paho.mqtt.client as mqtt
import time
global message
import requests
import time
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import datetime

payload_dict = { 'TIMESTAMP': 0,'TEMPERATURE': 0, 'HUMIDITY' : 0, 'BATTERY_VOLTAGE': 0, 'LIGHT_INTENSITY': 0, 'LPG_CONCENTRATION': 0, 'CARBON_MONOXIDE': 0, 'SMOKE': 0, 'MOTOR': 0}
count = 0


myMQTTClient = AWSIoTMQTTClient("ClientID")
myMQTTClient.configureEndpoint("a8vv11hxsa5yb-ats.iot.ap-south-1.amazonaws.com", 8883)

# myMQTTClient.configureCredentials("/home/pi/Documents/RPI/AmazonRootCA1.pem", "/home/pi/Documents/RPI/aaec224321f9ead8b32b85efab57780338f9a33e0aec1b2a83001f5ad3b9b10c-private.pem.key", "/home/pi/Documents/RPI/aaec224321f9ead8b32b85efab57780338f9a33e0aec1b2a83001f5ad3b9b10c-certificate.pem.crt")
 
myMQTTClient.configureCredentials("D:/nipun prev laptop/final_year/Project scripts/AWS Write/AmazonRootCA1.pem", "D:/nipun prev laptop/final_year/Project scripts/AWS Write/aaec224321f9ead8b32b85efab57780338f9a33e0aec1b2a83001f5ad3b9b10c-private.pem.key", "D:/nipun prev laptop/final_year/Project scripts/AWS Write/aaec224321f9ead8b32b85efab57780338f9a33e0aec1b2a83001f5ad3b9b10c-certificate.pem.crt")


print ('Initiating Realtime Data Transfer From Raspberry Pi...')

{"LPG_CONCENTRATION":{"S":"-1"},"HUMIDITY":{"S":"2.35294"},"TEMPERATURE":{"S":"-50.7498"},"BATTERY_VOLTAGE":{"S":"10.9455"},"MOTOR":{"S":"0"},"CARBON_MONOXIDE":{"S":"-1"},"SMOKE":{"S":"sensor_char7"},"LIGHT_INTENSITY":{"S":"0"}}

Myvar= myMQTTClient.connect()

broker_url = "broker.hivemq.com"
broker_port = "1883"



def on_connect(client, userdata, flags, rc):
    print(f"mqtt client Connected ...")
    client.subscribe("topic/temperature")
    client.subscribe("topic/humidity")
    client.subscribe("topic/battery")
    client.subscribe("topic/light")
    client.subscribe("topic/lpg")
    client.subscribe("topic/co")
    client.subscribe("topic/smoke")
    client.subscribe("topic/motor")



def on_message(client, userdata, msg):
    global payload_dict, count
    print("message received on topic: " + msg.topic +  msg.payload.decode())
    topic = msg.topic
    
    if topic == "topic/temperature":
        var1 = msg.payload.decode()
        myMQTTClient.publish("topic/temperature", var1, 1)
        print("Received payload for topic1: ", var1)
        payload_dict['TEMPERATURE'] = var1
        count +=1
    elif topic == "topic/humidity":
        var2 = msg.payload.decode()
        myMQTTClient.publish("topic/humidity", var2, 1)
        print("Received payload for topic2: ", var2)
        payload_dict['HUMIDITY'] = var2
        count +=1
    elif topic == "topic/battery":
        var3 = msg.payload.decode()
        myMQTTClient.publish("/topic/battery", var3, 1)
        print("Received payload for topic3: ", var3)
        payload_dict['BATTERY_VOLTAGE'] = var3
        count +=1
    elif topic == "topic/light":
        var4 = msg.payload.decode()
        myMQTTClient.publish("/topic/light", var4, 1)
        print("Received payload for topic4: ", var4)
        payload_dict['LIGHT_INTENSITY'] = var4
        count +=1
    elif topic == "topic/lpg":
        var5 = msg.payload.decode()
        myMQTTClient.publish("/topic/lpg", var5, 1)
        print("Received payload for topic5: ", var5)
        payload_dict['LPG_CONCENTRATION'] = var5
        count +=1
    elif topic == "topic/co":
        var6 = msg.payload.decode()
        myMQTTClient.publish("/topic/co", var6, 1)
        print("Received payload for topic6: ", var6)
        payload_dict['CARBON_MONOXIDE'] = var6
        count +=1
    elif topic == "topic/smoke":
        var7 = msg.payload.decode()
        myMQTTClient.publish("/topic/smoke", var7, 1)
        print("Received payload for topic7: ", var7)
        payload_dict['SMOKE'] = var7
        count +=1
    elif topic == "topic/motor":
        var8 = msg.payload.decode()
        myMQTTClient.publish("/topic/motor", var8, 1)
        print("Received payload for topic8: ", var8)
        payload_dict['MOTOR'] = var8
        count +=1
    else:
        print("Unknown topic: " + topic)
    
    if count == 8:
        timestamp = datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
        payload_dict['TIMESTAMP'] = timestamp
        payload = json.dumps(payload_dict)
        myMQTTClient.publish("/topic/alldata", payload, 1)
        print("all data json payload published: ", payload)
        count = 0


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker_url, 1883, 60)
client.loop_forever()

