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

meeting_message = bytearray(b"/?60616161!\r\n")
seri.write(meeting_message)
time.sleep(0.25)
meeting_response = bytearray(seri.readline())
time.sleep(0.25)
print(meeting_response)
print(len(meeting_response))

max_baud_rate = b"\x35"
mbr_str = max_baud_rate.decode("utf-8")
mbr_int = int(mbr_str)

baud_rates = [300, 600, 1200, 2400, 4800, 9600]

if meeting_response[0] == 47 and len(meeting_response) > 5:
    information_message = bytearray(b"\x0601\r\n")
    # information_message[2:2] = b"6"
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

    if (information_message[3] == 0x30 or information_message[3] == 0x36):
        print("entered readout mode")

        for i in range(10):
            res = bytearray(seri.readline())
            print(res)

    else:
        print("entered programming mode")

        information_response = bytearray(seri.readline())
        print(information_response)
        print(len(information_response))
        bcc_received = information_response.pop()

        bcc = 0x01
        for i in information_response:
            bcc ^= i

        print("information response bcc result: ", hex(bcc))

        if bcc_received == bcc:
            # bcc = 0x01
            # # tüketim sorgusu
            # loadFormat = bytearray(
            #     b"\x01\x52\x32\x02\x50\x2E\x30\x31\x2823-11-18,10:00;23-11-18,13:00\x29\x03"
            # )
            
            bcc = 0x01
            # tüketim sorgusu
            loadFormat = bytearray(
                b"\x01\x52\x32\x02\x50\x2E\x30\x31\x28;\x29\x03"
            )

            for b in loadFormat:
                bcc ^= b
            loadFormat.append(bcc)
            print("data to send device: ", loadFormat)
            seri.write(loadFormat)

            xor_check = 0x02
            for i in range(500):
                data = bytearray(seri.readline())
                if len(data) == 0:
                    break
                print(data)
                if len(data) != 3:
                    for b in data:
                        xor_check ^= b
                else:
                    xor_check ^= data[0]
                    xor_check ^= data[1]
                    print("xor check:", hex(xor_check))

            # loadFormat2 = bytearray(b'\x01\x52\x32\x02\x50\x2E\x30\x31\x282308102315;2308110100\x29\x03')
            # bcc2= 0x01
            # for b in loadFormat2:
            #     bcc2 ^= b
            # loadFormat2.append(bcc2)
            # print(loadFormat2)
            # seri.write(loadFormat2)
            # for i in range(20):
            #     print(seri.readline())

            # bcc_password = 0x01
            # passwordMessage = bytearray(b'\x01\x50\x31\x02\x2812345678\x29\x03')
            # for b in passwordMessage:
            #     bcc_password ^= b
            # passwordMessage.append(bcc_password)
            # print(passwordMessage)
            # seri.write(passwordMessage)
            # print(bytearray(seri.readline()))

            # bcc_time = 0x01
            # timeSet = bytearray(b'\x01\x57\x32\x02\x30\x2E\x39\x2E\x31\x2808:18:20\x29\x03')
            # for b in timeSet:
            #     bcc_time ^=b
            # timeSet.append(bcc_time)
            # seri.write(timeSet)
            # print(seri.readline())

            # bcc_pw_hint = 0x01
            # pw_hint_message = bytearray(
            #     b"\x01\x52\x32\x02\x39\x36\x2e\x39\x35\x2e\x31\x28\x29\x03"
            # )
            # for b in pw_hint_message:
            #     bcc_pw_hint ^= b
            # pw_hint_message.append(bcc_pw_hint)
            # print(pw_hint_message)
            # seri.write(pw_hint_message)
            # print(bytearray(seri.readline()))

            # bcc_date = 0x01
            # dateSet = bytearray(
            #     b"\x01\x57\x32\x02\x30\x2E\x39\x2E\x32\x2823-11-23\x29\x03"
            # )
            # for b in dateSet:
            #     bcc_date ^= b
            # dateSet.append(bcc_date)
            # seri.write(dateSet)
            # print(seri.readline())

            # production = bytearray(b"\x01R2\x0296.1.3()\x03")
            # bcc = 0x01
            # for b in production:
            #     bcc ^= b
            # production.append(bcc)
            # print(production)
            # seri.write(production)
            # time.sleep(0.25)
            # while True:
            #     res = bytearray(seri.readline())
            #     if len(seri.readline()) == 0:
            #         break
            #     else:
            #         print("message received: ",res)
