import RPi.GPIO as GPIO
import time
import math

GPIO.setmode(GPIO.BCM)
GPIO.setup(8, GPIO.IN)  # DSM501A input D8

sample_time_ms = 30000
low_pulse_occupancy = 0
starttime = time.time()

while True:
    start_pulse = time.time()
    GPIO.wait_for_edge(8, GPIO.FALLING)
    end_pulse = time.time()
    duration = end_pulse - start_pulse
    low_pulse_occupancy += duration
    endtime = time.time()
    if ((endtime - starttime) > sample_time_ms/1000):
        ratio = (low_pulse_occupancy - endtime + starttime + sample_time_ms)/(sample_time_ms*10.0)  # Integer percentage 0=>100
        concentration = 1.1*math.pow(ratio, 3) - 3.8*math.pow(ratio, 2) + 520*ratio + 0.62  # using spec sheet curve
        print("low_pulse_occupancy: {}".format(low_pulse_occupancy))
        print("ratio: {}".format(ratio))
        print("DSM501A: {}".format(concentration))
        print("\n")
        low_pulse_occupancy = 0
        starttime = time.time()

GPIO.cleanup()
