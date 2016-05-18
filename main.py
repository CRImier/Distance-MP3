#!/usr/bin/env python
import os 
import json 
import threading
from datetime import datetime
from copy import copy
import subprocess
import shlex
import time

import RPi.GPIO as GPIO
import smbus
import serial
import termios
from pygame import mixer

"""
This code queries an external device connected through serial that presumably sends a distance in centimeters. 
Once this distance is between some bounds (configurable), it starts playing a music file. 

Program's behaviour is modifiable using a configuration file (expected to be found at config_path)
"""

config_path = "/etc/mp3_distance.json"
#config_path = "mp3_distance.json"

def read_config(config_path):
    try:
        f = open(config_path, 'r')
        data = json.load(f)
        f.close()
    except (IOError, ValueError) as e:
        raise
    else:
        return data

def scan_partitions():
    partitions = []
    dbu_dir = "/dev/disk/by-uuid/"
    parts_by_uuid = os.listdir(dbu_dir)
    for uuid in parts_by_uuid:
        path = os.path.realpath(os.path.join(dbu_dir, uuid))
        details_dict = {"uuid":uuid, "path":path}
        partitions.append(details_dict)
        #partitions is now something like 
        #[{"uuid":"5OUU1DMUCHUNIQUEW0W", "path":"/dev/sda1"}, {"label":"label1", "uuid":"MANYLETTER5SUCH1ONGWOW", "path":"/dev/mmcblk0"}]
    mtab_file = "/etc/mtab" 
    f = open(mtab_file, "r")
    lines = f.readlines()
    f.close()
    mounted_partitions = {}
    for line in lines:
        line = line.strip().strip("\n")
        if line: #Empty lines? Well, who knows what happens... =)
            elements = shlex.split(line) #Avoids splitting where space character is enclosed in ###########
            if len(elements) != 6:
                break
            path = elements[0] 
            #mtab is full of entries that aren't any kind of partitions we're interested in - we're interested in physical&logical partitions of disk drives
            #That's why we need to filter entries by path
            if path.startswith("/dev"):
                #Seems to be a legit disk device. It's either /dev/sd** or a symlink to that. If it's a symlink, we resolve it.
                mountpoint = elements[1]
                dev_path = os.path.realpath(path)
                mounted_partitions[dev_path] = mountpoint
    for entry in partitions:
         if entry["path"] in mounted_partitions:
             entry["mounted"] = True
             entry["mountpoint"] = mounted_partitions[entry["path"]]
         else:
             entry["mounted"] = False
             entry["mountpoint"] = None
    return partitions
         

class Player():
    """ A music player class. Has 2 simple methods to control the playback. Supports MP3 sound. """
    def __init__(self, file_path):
        self.path = file_path
        mixer.init()
        self.playing = False
 
    def play(self):
        if self.playing:
            return True
        self.playing = True
        print("Starting to play music...")
        mixer.music.load(self.path)
        mixer.music.play()
        time.sleep(1)
        
    def stop(self):
        self.playing = False
        print("Stopping music playback.")
        mixer.music.stop()

    def fadeout(self, fadeout_time):
        self.playing = False
        print("Fading out and stopping music playback.")
        mixer.music.fadeout(fadeout_time)

last_time_triggered = None

def pir_scanning(pin_num):
    global last_time_triggered
    GPIO.setmode(GPIO.BCM)      
    GPIO.setup(pin_num, GPIO.IN)
    state = False
    while True: 
        current_state = GPIO.input(pin_num)
        if state != current_state:
            if current_state:
                print('Sensor triggered at {}'.format(datetime.now()))
                last_time_triggered = datetime.now()
            state = current_state
        time.sleep(0.5)

def pir_sensor_active(max_diff_seconds):
    if last_time_triggered is None:
         return False
    now = datetime.now()
    ltt = copy(last_time_triggered)
    delta = now - ltt #Possible time shift issues (think DST and such), just won't work
    return delta.total_seconds() < max_diff_seconds 

"""
def get_distance(bus, address):
    distance = bus.read_byte_data(address, 0x00)
    return distance
"""

"""
def get_distance(port):
    port.reset_input_buffer()
    timeout_counter = 0
    while timeout_counter < 20:
       if not port.in_waiting:
           sleep(0.1)
       else:
           print(port.in_waiting)
           distance = port.readline()
           return int(distance.strip('\n'))
       timeout_counter += 1
    raise IOError
"""

def get_distance(port):
    port.flushInput()
    timeout_counter = 0
    while timeout_counter < 20:
       if not port.inWaiting():
           time.sleep(0.1)
       else:
           distance = port.readline()
           return int(distance.strip('\n'))
       timeout_counter += 1
    raise IOError

def get_audio_path(config):
    if "audio_path" in config:
        audio_path = config["audio_path"]
        return audio_path
    else:
        filename = config["filename"]
        partition_filter_startswith = "/dev/mmcblk"
        audio_dirs = [partition["mountpoint"] for partition in scan_partitions() if partition["mountpoint"] and not partition["path"].startswith(partition_filter_startswith)]
        for dir in audio_dirs:
            if filename in os.listdir(dir):
                return os.path.join(dir, filename)

def start():
    config = read_config(config_path)
    #bus = smbus.SMBus(config["bus"]) #I2C
    serial_timeout = config["serial_timeout"]
    port = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=serial_timeout)
    address = int(config["arduino_address"], 0) #Address can be stored as decimal, hexadecimal or even octal if you're perverted enough
    upper_range = config["upper_range"]
    lower_range = config["lower_range"]
    pir_gpio = config["pir_gpio_pin"] if "pir_gpio_pin" in config else None
    pir_interval = config["pir_max_interval"] if "pir_max_interval" in config else None
    iter_count = config["iter_count"] if "iter_count" in config else 1
    sleep_time = config["sleep_time"] if "sleep_time" in config else 10
    fadeout_time = config["fadeout_time"] if "fadeout_time" in config else None
    distance_ok = False #A flag to indicate we're in the state where distance is ok
    stop_triggered = False #A flag to indicate state when 2 seconds are passing
    player = None
    audio_path = None
    if iter_count > 1:
        print("Taking {} values and filtering".format(iter_count))
    if pir_gpio:
        t = threading.Thread(target=pir_scanning, args = [pir_gpio]) #Adding threads to such a simple program you need refactois a signring
        t.daemon = True
        t.start()
    while True:
        try:
            prev_path = audio_path
            audio_path = get_audio_path(config)
            if prev_path != audio_path:
                print("Path changed from {} to {}".format(prev_path, audio_path))
                player = Player(audio_path)
        except:
            raise
            print("Can't find audio!")
            sleep(sleep_time)
            audio_path = None
            continue
        try: #Data acquisition
            #distance = get_distance(bus, address) #I2C
            distances = []
            if iter_count > 1:
                for i in range(iter_count):
                    try:
                        distances.append(get_distance(port))
                    except (IOError, termios.error):
                        pass #Just not appending anything
                if not distances or iter_count/len(distances) > 1: #More than half of attempts weren't successful, we can signal that comms weren't successful
                    raise IOError
                #Now continuing onto filtering values.
                print("Distances: {}".format(distances))
                average = sum(distances)/iter_count
                print("Average: {}".format(average))
                if average in range(lower_range): 
                    distance = 0
                else:
                    filtered_distances = []
                    for distance in distances: #More advanced filtering
                        if float(distance)/average > 1.5:
                            print("Filtered out value: {}".format(distance))
                        else:
                            filtered_distances.append(distance)
                    distance = sum(filtered_distances)/iter_count
            else:
                distance = get_distance(port)
            print("Distance: {}".format(distance))
        #except (IOError, termios.error, OSError): #Uncomment for debugging
        except:
            #print("Can't connect to device at address {}!".format(str(hex(address)))) #I2C
            print("Can't connect to device or communication error occured!")
            time.sleep(1)
        else: #Decision making
            time_left_to_sleep = sleep_time-(iter_count*serial_timeout)
            if lower_range <= distance <= upper_range: #In range of the sensor, need to start playing
                if pir_gpio and not distance_ok and not pir_sensor_active(pir_interval): #This check should only work when starting playback
                    print("Ultrasonic sensor triggered but PIR sensor not active")
                    continue
                distance_ok = True
                stop_triggered = False
                player.play()
                time.sleep(time_left_to_sleep)
            else:
                if distance_ok: #Giving 2 seconds of delay in case distance is restored during those 2 seconds
                    print("Giving {} seconds to come back...".format(sleep_time))
                    time.sleep(time_left_to_sleep)
                    stop_triggered = True
                    distance_ok = False
                elif stop_triggered: #Going through this one the second time, two seconds have passed and no response so far
                    print("{} seconds passed!".format(sleep_time))
                    if fadeout_time:
                        player.fadeout(fadeout_time)
                        time_left_to_sleep = time_left_to_sleep - fadeout_time 
                        if time_left_to_sleep < 0: time_left_to_sleep = 0
                    else:
                        player.stop()
                    time.sleep(time_left_to_sleep)
                    stop_triggered = False
                else:
                    time.sleep(time_left_to_sleep) #Normal state

    print("Job well done!")

if __name__ == "__main__":
    start()


