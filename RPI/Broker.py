# subscriber.py
import paho.mqtt.client as mqtt
import time
import board
import adafruit_bme680

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)

# change this to match the location's pressure (hPa) at sea level
bme680.sea_level_pressure = 1013.25

# You will usually have to add an offset to account for the temperature of
# the sensor. This is usually around 5 degrees but varies by use. Use a
# separate temperature sensor to calibrate this one.
temperature_offset = -5

broker_url = "192.168.43.206"
broker_port = "1883"



def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code ")
    client.subscribe("raspberry/topic")
    client.subscribe("/topic/qos1")


def on_message(client, userdata, msg):
    print("message received on topic " +  msg.payload.decode())


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message


client.will_set('raspberry/status', b'{"status": "Off"}')


client.connect(broker_url, 1883, 60)


client.loop_forever()

while True:
    print("\nTemperature: %0.1f C" % (bme680.temperature + temperature_offset))
    print("Gas: %d ohm" % bme680.gas)
    print("Humidity: %0.1f %%" % bme680.relative_humidity)
    print("Pressure: %0.3f hPa" % bme680.pressure)
    print("Altitude = %0.2f meters" % bme680.altitude)

    time.sleep(1)