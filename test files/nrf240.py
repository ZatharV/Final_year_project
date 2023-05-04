import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev

# Set the pins for the NRF24L01 module
GPIO.setmode(GPIO.BCM)
pipes = "Node1"

# Initialize the NRF24L01 module
radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)

# Set the radio frequency to 2.4GHz
radio.setChannel(0x76)

# Set the data rate to 1Mbps and the power amplifier to the maximum level
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MAX)

# Open the receive pipe and start listening for data
radio.openReadingPipe(0, pipes)
radio.startListening()

while True:
    # Check if there is any data available
    while not radio.available(0):
        time.sleep(1)

    # Read the data and print it to the console
    received_message = []
    radio.read(received_message, radio.getDynamicPayloadSize())
    print("Received: {}".format(received_message))