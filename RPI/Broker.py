# subscriber.py
import paho.mqtt.client as mqtt
import time
import board
import adafruit_bme680
global message
import requests
import time
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

################################################################################333

myMQTTClient = AWSIoTMQTTClient("ClientID")
myMQTTClient.configureEndpoint("a8vv11hxsa5yb-ats.iot.ap-south-1.amazonaws.com", 8883)

myMQTTClient.configureCredentials("/home/pi/Documents/RPI/AmazonRootCA1.pem", "/home/pi/Documents/RPI/aaec224321f9ead8b32b85efab57780338f9a33e0aec1b2a83001f5ad3b9b10c-private.pem.key", "/home/pi/Documents/RPI/aaec224321f9ead8b32b85efab57780338f9a33e0aec1b2a83001f5ad3b9b10c-certificate.pem.crt")
 
print ('Initiating Realtime Data Transfer From Raspberry Pi...')



Myvar= myMQTTClient.connect()

# date = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
# print (f"Timestamp:{date}")

###################################################################################

# i2c = board.I2C()  
# bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)
# bme680.sea_level_pressure = 1013.25
# temperature_offset = -5

broker_url = "broker.hivemq.com"
broker_port = "1883"



def on_connect(client, userdata, flags, rc):
    print(f"mqtt client Connected ...")
    client.subscribe("/topic/temperature")
    client.subscribe("/topic/humidity")
    client.subscribe("/topic/battery")
    client.subscribe("/topic/light")
    client.subscribe("/topic/lpg")
    client.subscribe("/topic/co")
    client.subscribe("/topic/smoke")
    client.subscribe("/topic/motor")



def on_message(client, userdata, msg):
    print("message received on topic: " + msg.topic +  msg.payload.decode())
    topic = msg.topic
    if topic == "/topic/temperature":
        var1 = msg.payload.decode()
        myMQTTClient.publish("/topic/temperature", var1, 1)
        print("Received payload for topic1: ", var1)
    elif topic == "/topic/humidity":
        var2 = msg.payload.decode()
        myMQTTClient.publish("/topic/humidity", var2, 1)
        print("Received payload for topic2: ", var2)
    elif topic == "/topic/battery":
        var3 = msg.payload.decode()
        myMQTTClient.publish("/topic/battery", var3, 1)
        print("Received payload for topic3: ", var3)
    elif topic == "/topic/light":
        var4 = msg.payload.decode()
        myMQTTClient.publish("/topic/light", var4, 1)
        print("Received payload for topic4: ", var4)
    elif topic == "/topic/lpg":
        var5 = msg.payload.decode()
        myMQTTClient.publish("/topic/lpg", var5, 1)
        print("Received payload for topic5: ", var5)
    elif topic == "/topic/co":
        var6 = msg.payload.decode()
        myMQTTClient.publish("/topic/co", var6, 1)
        print("Received payload for topic6: ", var6)
    elif topic == "/topic/smoke":
        var7 = msg.payload.decode()
        myMQTTClient.publish("/topic/smoke", var7, 1)
        print("Received payload for topic7: ", var7)
    elif topic == "/topic/motor":
        var8 = msg.payload.decode()
        myMQTTClient.publish("/topic/motor", var8, 1)
        print("Received payload for topic8: ", var8)
    else:
        print("Unknown topic: " + topic)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.will_set('raspberry/status', b'{"status": "Off"}')
client.connect(broker_url, 1883, 60)
client.loop_forever()

# while True:
#     print("\nTemperature: %0.1f C" % (bme680.temperature + temperature_offset))
#     print("Gas: %d ohm" % bme680.gas)
#     print("Humidity: %0.1f %%" % bme680.relative_humidity)
#     print("Pressure: %0.3f hPa" % bme680.pressure)
#     print("Altitude = %0.2f meters" % bme680.altitude)

#     time.sleep(1)