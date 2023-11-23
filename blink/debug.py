#!/bin/python

import serial
from bitarray import bitarray
import time
import hashlib

def printArrayHexBinary(arr):
    for i in arr:
        print("hexadecimal: ", hex(i), "\tbinary: ", bin(i))

    print("\n")


seri = serial.Serial(
    "/dev/ttyUSB0", baudrate=300, bytesize=7, parity="E", stopbits=1, timeout=2
)


md5_hash = hashlib.md5()

meeting_message = bytearray(b"/?!\r\n")
seri.write(meeting_message)
time.sleep(0.25)
meeting_response = bytearray(seri.readline())
time.sleep(0.25)
print(meeting_response)

max_baud_rate = b"\x35"
mbr_str = max_baud_rate.decode("utf-8")
mbr_int = int(mbr_str)

baud_rates = [300, 600, 1200, 2400, 4800, 9600]

if meeting_response[0] == 47 and len(meeting_response) > 5:
    information_message = bytearray(b"\x0604\r\n")
    information_message[2:2] = max_baud_rate
    print(information_message)

    seri.write(information_message)
    time.sleep(0.25)

    seri.close()
    seri = serial.Serial(
        "/dev/ttyUSB0",
        baudrate=baud_rates[mbr_int],
        bytesize=7,
        parity="E",
        stopbits=1,
        timeout=2,
    )
    print("port opened again. new baudrate is: ", seri.baudrate)

    while(1):
        res = seri.readline()
        if(len(res)==0):
            break
        print(res)
