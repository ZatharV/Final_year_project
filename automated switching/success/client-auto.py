#!/usr/bin/env python3

import socket
import time
import json
import os
import subprocess
import random
import argparse
import psutil
from command_runner import command_runner_threaded

wait_time = 10
data_sent = False
wait_end_feed = False
camera_id = 0

def trigger_function():
    global data_sent
    if data_sent==True:
        data_sent=False
        return True

def feed_ended():
    global wait_end_feed
    wait_end_feed = False

def main():
    parser = argparse.ArgumentParser(description='Slip interface setup arguments for client')
    parser.add_argument('--baudrate', type=str, default='115200', help='Baudrate at which the jetson communicates with the radio using usb to ttl')
    parser.add_argument('--usb_port', type=str, default='/dev/ttyUSB0', help='port at which the the radio has been connected')
    parser.add_argument('--sender_ip', type=str, default='10.0.0.1', help='ip address of the sender rover')
    parser.add_argument('--receivers_ip', type=str, default='10.0.0.2', help='ip address of the receiver control station')
    parser.add_argument('--netmask', type=str, default='255.255.255.0', help='netmask for ip address')
    parser.add_argument('--mtu', type=str, default='200', help='mtu of the slip interface')
    parser.add_argument('--txqueuelen', type=str, default='1024', help='transmission queue length')
    parser.add_argument('--control_port', type=int, default=8000, help='port at which control messgaes to be sent')
    parser.add_argument('--video_port', type=str, default=6000, help='port at which video to be sent')
    parser.add_argument('--width', type=str, default='640', help='width of the displayed video')
    parser.add_argument('--height', type=str, default='480', help='height of the displayed video')
    args = parser.parse_args()

    cmd1 = 'cd /home/nvidia/Documents && sudo insmod slip.ko'

    try:
        # Run the command and raise an exception if it returns a non-zero status code
        subprocess.run(cmd1, shell=True, check=True, stderr=subprocess.DEVNULL)
    except:
        # Handle the exception
        print("slip file already exixts")

    while True:
        result = subprocess.run('ls /dev/ttyUSB*', stdout=subprocess.PIPE, shell=True)
        output = result.stdout.decode('utf-8').strip()
        try:
            cmd2= f'sudo ldattach -s 115200 SLIP {output}'
            subprocess.run(cmd2, shell=True, check=True)
        except:
            print("not connected")
            continue
        break
    print("done")
    time.sleep(1)

    cmd3 = f'sudo ifconfig sl0 {args.receivers_ip} netmask {args.netmask} mtu {args.mtu} pointopoint {args.sender_ip} txqueuelen {args.txqueuelen}'
    os.system(cmd3)
    time.sleep(1) 
    
    os.nice(-15)
    my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    my_host = args.sender_ip
    my_port = args.control_port

    command_runner_threaded(f'gst-launch-1.0 udpsrc port={args.video_port} ! application/x-rtp,encoding-name=H265,payload=127 ! rtph265depay ! h265parse ! queue ! omxh265dec ! videoconvert ! videoscale ! \'video/x-raw, format=I420, width={args.width}, height={args.height}\' ! nveglglessink window-x=0 window-y=0 max-lateness=-1 qos=False enable-last-sample=False', stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low',shell=False)

    try:
        my_socket.connect((my_host, my_port))
    except:
        pass
    while True:
        global data_sent, wait_end_feed, camera_id
        camera_id = camera_id + 1
        time.sleep(wait_time) #CAMERA SWITCHING TIME
        if camera_id > 3:
            camera_id = 0
        test_json = json.dumps({"ID":camera_id})
        try:
            my_socket.send(test_json.encode('utf-8'))
            data_sent = True
            wait_end_feed = True
            print("sent",camera_id)
            while wait_end_feed == True:
                pass
            if (camera_id == 0):
                command_runner_threaded(f'gst-launch-1.0 udpsrc port={args.video_port} ! application/x-rtp,encoding-name=H265,payload=127 ! rtph265depay ! h265parse ! queue ! omxh265dec ! videoconvert ! videoscale ! \'video/x-raw, format=I420, width={args.width}, height={args.height}\' ! nveglglessink window-x=0 window-y=0 max-lateness=-1 qos=False enable-last-sample=False', stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low',shell=False)
            elif (camera_id == 1):
                command_runner_threaded(f'gst-launch-1.0 udpsrc port={args.video_port} ! application/x-rtp,encoding-name=H265,payload=127 ! rtph265depay ! h265parse ! queue ! omxh265dec ! videoconvert ! videoscale ! \'video/x-raw, format=I420, width={args.width}, height={args.height}\' ! nveglglessink window-x=1280 window-y=0 max-lateness=-1 qos=False enable-last-sample=False', stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low',shell=False)
            elif (camera_id == 2):
                command_runner_threaded(f'gst-launch-1.0 udpsrc port={args.video_port} ! application/x-rtp,encoding-name=H265,payload=127 ! rtph265depay ! h265parse ! queue ! omxh265dec ! videoconvert ! videoscale ! \'video/x-raw, format=I420, width={args.width}, height={args.height}\' ! nveglglessink window-x=0 window-y=600 max-lateness=-1 qos=False enable-last-sample=False', stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low',shell=False)
            elif (camera_id == 3):
                command_runner_threaded(f'gst-launch-1.0 udpsrc port={args.video_port} ! application/x-rtp,encoding-name=H265,payload=127 ! rtph265depay ! h265parse ! queue ! omxh265dec ! videoconvert ! videoscale ! \'video/x-raw, format=I420, width={args.width}, height={args.height}\' ! nveglglessink window-x=1280 window-y=600 max-lateness=-1 qos=False enable-last-sample=False', stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low',shell=False)
            print("cmd executed")
        except socket.error:
            print("Socket error occurred. Retrying...")
            my_socket.close()
            my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            while True:
                try:
                    my_socket.connect((my_host, my_port))
                    break
                except socket.error:
                    time.sleep(1)
                    continue
        except KeyboardInterrupt:
            data_sent       = True      #Signal end of feed
            wait_end_feed   = True      #Wait end of feed flag
            while(True == wait_end_feed):
                pass
            exit()

if __name__ == "__main__":
    main()
