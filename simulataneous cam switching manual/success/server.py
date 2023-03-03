#!/usr/bin/env python3

import socket
import time
import json
import os
import subprocess
import argparse
import signal
import psutil
from command_runner import command_runner_threaded

prev_id = None
default_cam = 0
wait_end_feed = False
kill_feed = False
def trigger_function():
    global kill_feed
    rc = False
    if (True == kill_feed):
        rc = True
        kill_feed = False
    return rc
    
def feed_ended():
    global wait_end_feed
    wait_end_feed = False

def main():
    parser = argparse.ArgumentParser(description='Slip interface setup arguments')
    parser.add_argument('--baudrate', type=str, default='115200', help='Baudrate at which the jetson communicates with the radio using usb to ttl')
    parser.add_argument('--usb_port', type=str, default='/dev/ttyUSB0', help='port at which the the radio has been connected')
    parser.add_argument('--sender_ip', type=str, default='10.0.0.1', help='ip address of the sender rover')
    parser.add_argument('--receivers_ip', type=str, default='10.0.0.2', help='ip address of the receiver control station')
    parser.add_argument('--netmask', type=str, default='255.255.255.0', help='netmask for ip address')
    parser.add_argument('--mtu', type=str, default='200', help='mtu of the slip interface')
    parser.add_argument('--txqueuelen', type=str, default='1024', help='transmission queue length')
    parser.add_argument('--control_port', type=int, default=8000, help='port at which control messgaes to be sent')
    parser.add_argument('--video_port', type=str, default=6000, help='port at which video to be sent')
    # parser.add_argument('--width', type=str, default='160', help='width of the captured video')
    # parser.add_argument('--height', type=str, default='120', help='height of the captured video')
    parser.add_argument('--fps', type=str, default='10', help='fps of the captured video')
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

    cmd3 = f'sudo ifconfig sl0 {args.sender_ip} netmask {args.netmask} mtu {args.mtu} pointopoint {args.receivers_ip} txqueuelen {args.txqueuelen}'
    os.system(cmd3)
    time.sleep(1) 
    
    command_runner_threaded('gst-launch-1.0 v4l2src device="/dev/video0" ! \'video/x-raw, width=160, height=120, framerate=10/1\' ! videoconvert ! \'video/x-raw, format=I420, width=160, height=120\' ! omxh265enc bitrate=40 iframeinterval=5 EnableStringentBitrate=true control-rate=4 temporal-tradeoff=4 EnableTwopassCBR=true ! \'video/x-h265, stream-format=byte-stream\' ! h265parse ! queue ! rtph265pay pt=127 ! udpsink host=10.0.0.2 port=6000 sync=false', stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low', shell=False)
    
    os.nice(-15)
    my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    my_host = "0.0.0.0"
    my_port = args.control_port
    print(my_port)
    my_socket.setsockopt(socket.SOL_SOCKET,socket.SO_PRIORITY,11)
    my_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    my_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
    my_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
    my_socket.bind((my_host, my_port))
    my_socket.listen(5)
    my_socket.settimeout(2)
    
    while True:
        global camera_id1, camera_id2, cam_width, cam_height
        try:
            cl, myaddr = my_socket.accept()
            print('Got connection from', myaddr)
            while True:
                data=cl.recv(1024)
                print("data recvd or not?",data)
                if data:
                    global wait_end_feed, kill_feed
                    print(data.decode('utf-8'))
                    string_data = data.decode('utf-8')
                    received_dict=json.loads(string_data)
                    camera_id1 = str(received_dict['ID1'])
                    camera_id2 = str(received_dict['ID2'])
                    cam_width = str(received_dict['WIDTH'])
                    cam_height = str(received_dict['HEIGHT'])
                    print("check this received camera id1 == ", camera_id1)
                    print("check this received camera id2 == ", camera_id2)
                    print("check this received camera width == ", cam_width)
                    print("check this received camera height == ", cam_height)
                    
                    kill_feed       = True      #Signal end of feed
                    wait_end_feed   = True      #Wait end of feed flag
                    while(True == wait_end_feed):
                        pass
                    command_runner_threaded(f"gst-launch-1.0 v4l2src device=\"/dev/video{camera_id1}\" ! video/x-raw, width={cam_width}, height={cam_height}, framerate=10/1 ! videoconvert ! 'video/x-raw, format=I420, width={cam_width}, height={cam_height}' ! omxh265enc bitrate=40 iframeinterval=5 EnableStringentBitrate=true control-rate=4 temporal-tradeoff=4 EnableTwopassCBR=true ! 'video/x-h265, stream-format=byte-stream' ! h265parse ! queue ! rtph265pay pt=127 ! udpsink host=10.0.0.2 port=6000 sync=false v4l2src device=\"/dev/video{camera_id2}\" ! video/x-raw, width={cam_width}, height={cam_height}, framerate=10/1 ! videoconvert ! 'video/x-raw, format=I420, width={cam_width}, height={cam_height}' ! omxh265enc bitrate=40 iframeinterval=5 EnableStringentBitrate=true control-rate=4 temporal-tradeoff=4 EnableTwopassCBR=true ! 'video/x-h265, stream-format=byte-stream' ! h265parse ! queue ! rtph265pay pt=127 ! udpsink host=10.0.0.2 port=6001 sync=false", stop_on=trigger_function, on_exit=feed_ended, priority='low', io_priority='low',shell=False)
                    print("running this...")
                else:
                    print("no data recvd running same camera feed")
                    break
        except KeyboardInterrupt:
            kill_feed       = True      #Signal end of feed
            wait_end_feed   = True      #Wait end of feed flag
            while(True == wait_end_feed):
                pass
            exit()
        except:
            print("error connecting to control station retrying...")
            pass
    
if __name__ == "__main__":
    main()



