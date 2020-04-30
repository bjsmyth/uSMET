#!/usr/bin/python3.6

import serial
from crc16 import crc16xmodem
from collections import namedtuple
from struct import *
import pandas as pd
import argparse
from os import path, mkdir
from inputs import get_gamepad
import threading
from signal import signal, SIGINT
import sys
import time


prevError = 0.0
output = 0.0
KI = 5
KP = 3
sendVal = 0

def exit_handler(signal_received, frame):
    global df
    global filePath
    global t1
    print('Exiting')
    df.to_pickle(filePath)
    t1.do_run = False
    t1.join()
    sys.exit()


def save_func():
    global df
    global filePath
    while True:
        df.to_pickle(filePath)
        time.sleep(5 - time.time() % 5)


def gamepad_func():
    global sendVal
    t = threading.currentThread()
    
    joystick = 0
    btn = 0
    setattr(t, "do_run", True)

    while getattr(t, "do_run", True):
        events = get_gamepad()
        for event in events:
            if event.code == 'ABS_X':
                joystick = event.state
            elif event.code == 'BTN_TL':
                btn = event.state
                
            sendVal = map_val(joystick, -32768, 32767, -90.0, 90.0)
        if btn != 1:
            sendVal = 0
                
def map_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


parser = argparse.ArgumentParser(description='Serial control for uSMET')
parser.add_argument('-d', default='/dev/ttyACM0', help='UART tty device name')
args = parser.parse_args()

ser = serial.Serial(args.d, 1000000)
t1 = threading.Thread(target=gamepad_func)
t1.start()

oneShot = True

columns = 'flags steering_current_pos steering_motor_set_rpm steering_motor_current_rpm reartrack_pos reartrack_duty vehicle_speed accel_x' \
          ' accel_y accel_z gyro_x gyro_y gyro_z vehicle_roll'
Data = namedtuple('Data', columns)
columns += '  steering_set_pos vehicle_set_speed reartrack_set_angle'
df = pd.DataFrame(columns=columns.split())
df = df.astype({"reartrack_pos": 'int64', "steering_set_pos": 'float64'})
#df = pd.read_pickle('test_frame.pkl')


crcErrors = 0
validRx = 0
packet = b'\x00'

if not path.exists("smet_logs"):
    mkdir("smet_logs")

i = 0
filePath = f"smet_logs/smet_data_{i}.pkl"
while path.exists(filePath):
    print(filePath)
    i += 1
    filePath = f"smet_logs/smet_data{i}.pkl"


signal(SIGINT, exit_handler)
ser.flushInput()
while True:
    #events = get_gamepad()
    #for event in events:
    #    if event.code == 'ABS_X':
    #           joystick = event.state

        
    packet = ser.read(1)
    #print(f'Received packet: {packet}')
    numBytes = int.from_bytes(packet, byteorder='big')
    ser.write(packet)


    packet = ser.read(numBytes)
    crc = crc16xmodem(packet)
    
    data = Data._make(unpack('>xBfifHfffffffffxx', packet))

    
    if crc != 0:
        crcErrors += 1
        ser.flushInput()
        continue
    
    
    #df1['steering_set_pos'][0] = sendVal
        
    #df1 = pd.DataFrame([[steer_pos, reartrack_pos, reartrack_duty, vehicle_speed, vehicle_roll]], columns=columns)
    #validRx += 1

    pack2 = b'\x04'
    ser.write(pack2)
    pack3 = ser.read(1)
    #if pack2 != pack3:
    #    continue
    currError = 0 - data.vehicle_roll
    #print(currError)
    #print(output)
    output += (KP + KI * 0.05) * currError - KP * prevError
    #output = 90 if output > 90 else output
    #output = -90 if output < 90 else output
    
    sendOutput = output * 0.216
    toSend = pack('>fff', sendOutput, 0, 0)
    sendcrc = crc16xmodem(toSend)
    toSend = pack('>fffH', sendOutput, 0, 0, sendcrc)
    ser.write(toSend)
    

    df1 = pd.DataFrame(data=[data])
    df1.insert(0, 'steering_set_pos', sendOutput)
    df = df.append(df1, ignore_index=True)
    prevError = currError
    
    print(sendOutput)
    #print(packet)
    #print(f'crc: {crc}')
    #print(f'Flags: {data.flags}')
    #print(f'ADC: {data.reartrack_pos}')
    #print(f'Degrees: {data.reartrack_duty}')
    #print(df1)
    #print(f'steer_pos: {steer_pos}')
    #print(f'reartrack_pos: {reartrack_pos}')
    #print(f'reartrack_duty: {reartrack_duty}')
    #print(f'vehicle_speed: {vehicle_speed}')
    #print(f'vehicle_roll: {data.vehicle_roll}\n')
    #print(f'Steering pos: {data.steering_current_pos}')
    #print(f'Control RPM: {data.steering_motor_set_rpm}')
    #print(f'CRC Errors: {crcErrors}')
    #print(df)
    #df.to_pickle(filePath)
    #df.to_csv('test.csv', index=None, header=True)
    #df.to_hdf('test_frame.h5', key='df', mode='a')
    

