#!/bin/python

import struct
import serial
from bitarray import bitarray
import time
import hashlib


def calculateBCC(data, xor):
    for i in data:
        xor ^= i

    return xor & 0x7F


def bitArrayToByte(lsb_arr):
    lsb_arr_list = lsb_arr.tolist()
    lsb_byte = 0

    for i in range(len(lsb_arr)):
        lsb_byte = (lsb_byte << 1) | lsb_arr_list[i]

    return lsb_byte


def conversionTo7Bit(data_arr):
    lsb_buffer = bitarray()
    dest_arr = bytearray()

    for index, byte in enumerate(data_arr):
        lsb = byte & 0x01
        byte = byte >> 1

        lsb_buffer.append(lsb)
        dest_arr.append(byte)

        if len(lsb_buffer) == 7 or index == len(data_arr) - 1:
            lsb_buffer = bitarray(reversed(lsb_buffer))
            lsb_byte = bitArrayToByte(lsb_buffer)

            dest_arr.append(lsb_byte)
            lsb_buffer = bitarray()

    return dest_arr


def messageLengthConversion(msg_len):
    lsb_buffer = bitarray()
    message_len_array = bytearray()

    for byte in msg_len.to_bytes(3, byteorder="big"):
        lsb = byte & 0x01
        byte = byte >> 1

        lsb_buffer.append(lsb)
        message_len_array.append(byte)

    lsb_buffer = bitarray(reversed(lsb_buffer))
    len_lsb_byte = bitArrayToByte(lsb_buffer)

    message_len_array.append(len_lsb_byte)

    return message_len_array


def printArrayHexBinary(arr):
    for i in arr:
        print("hexadecimal: ", hex(i), "\tbinary: ", bin(i))

    print("\n")


seri = serial.Serial(
    "/dev/ttyUSB0", baudrate=300, bytesize=7, parity="E", stopbits=1, timeout=2
)

md5_hash = hashlib.md5()

# dt = bytearray(b"Hello World!")
# md5_csum = hashlib.md5(dt).digest()

# csum_bytearray = bytearray(md5_csum)
# # md5_hash.update(dt)
# # checksum = md5_hash.digest()

# print("MD5 CHECKSUM: ", csum_bytearray)
# print(type(csum_bytearray))


meeting_message = bytearray(b"/?60616161!\r\n")
seri.write(meeting_message)
time.sleep(0.2)
meeting_response = bytearray(seri.readline())
time.sleep(0.2)
print(meeting_response)

max_baud_rate = b"\x35"
mbr_str = max_baud_rate.decode("utf-8")
mbr_int = int(mbr_str)

baud_rates = [300, 600, 1200, 2400, 4800, 9600]

if meeting_response[0] == 47 and len(meeting_response) > 5:
    information_message = bytearray(b"\x0601\r\n")
    information_message[2:2] = max_baud_rate
    print(information_message)

    seri.write(information_message)
    time.sleep(0.2)

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

    if information_message[3] == 0x30:
        print("entered readout mode")

        for i in range(4):
            res = bytearray(seri.readline())
            print(res)

    else:
        print("entered programming mode")

        information_response = bytearray(seri.readline())
        print(information_response)
        bcc_received = information_response.pop()

        bcc = 0x01
        for i in information_response:
            bcc ^= i

        print("information response bcc result: ", hex(bcc))

        if bcc_received == bcc:
            # initlialize the program info
            reprogram_info = bytearray(b"!!!!")

            print("repogram info : ", reprogram_info)
            # send to reporgram info message and wait for ack message
            seri.write(reprogram_info)
            rpg_res = bytearray(seri.readline())
            time.sleep(0.2)

            if rpg_res[0] == 0x06:
                print("ACK message received.")

                # open the file
                file = open(
                    "/home/yavuz/pico/freertos-pico/blink/build/blink.bin", "rb"
                )
                message = bytearray(file.read())

                # message = bytearray(b"\x01\x23\x45")
                # print("\nmessage: ")
                # printArrayHexBinary(message)

                # inilialize the reprogram array and convert it to 7 bit message
                reprogram_msg = bytearray()
                reprogram_msg = conversionTo7Bit(message)
                print("\nreprogram_msg: ")
                printArrayHexBinary(reprogram_msg)

                # get the converted program len
                message_len = len(reprogram_msg)

                # debug
                print("\nmessage len:")
                printArrayHexBinary(message_len.to_bytes(3, byteorder="big"))

                # convert the message len to 7-bit version
                converted_prg_len = bytearray(
                    reversed(messageLengthConversion(message_len))
                )

                # debug
                print("\nmessage len converted:")
                printArrayHexBinary(converted_prg_len)

                # initialize the program header and add the converted message len (4 Bytes)
                program_header = bytearray()
                program_header = converted_prg_len + program_header
                print(
                    "\nprogram header after adding converted program len: ",
                    program_header,
                )

                # calculate the md5 checksum for NOT CONVERTED program and add the program header array (16 Bytes)
                md5_csum = hashlib.md5(message).digest()
                csum_bytearray = bytearray(md5_csum)
                print("\nMD5 CHECKSUM: ", csum_bytearray)
                program_header = csum_bytearray + program_header
                print("\nprogram header after adding md5 checksum: ", program_header)

                # add the NOT CONVERTED PPROGRAM'S program len to the program header array (4 Bytes)
                program_len = len(message)
                program_header = (
                    bytearray(reversed(bytearray(program_len.to_bytes(4, "big"))))
                    + program_header
                )
                print("\nNOT CONVERTED program's program len: ", program_len)
                print("\nNOT CONVERTED program's program len as bytes:")
                printArrayHexBinary(program_len.to_bytes(4, "big"))
                print(program_header)

                # add epoch unix time value (4 Bytes)
                current_epoch = int(time.time())
                print("\nepoch value: ", current_epoch)
                program_header = (
                    bytearray(reversed(current_epoch.to_bytes(4, "big")))
                    + program_header
                )
                print("\nprogram header after adding epoch: ", program_header)

                # add padding to program header (should be 256 bytes)
                program_header = program_header + bytearray(256 - len(program_header))
                print("\nprogram header after padding: ")
                printArrayHexBinary(program_header)
                print("\nlength of program header", len(program_header))

                message = program_header + message
                print("\nprogram header + message: ")
                # printArrayHexBinary(message)

                reprogram_msg2 = conversionTo7Bit(message)
                print("\nconverted program header + message : ")
                # printArrayHexBinary(reprogram_msg2)

                print("sending data...")
                seri.write(reprogram_msg2)
                print("data sent.")

            # if res[0] == 0x06:
            #     error_count = 0
            #     while len(reprogram_msg) != 0:
            #         msg_pack = reprogram_msg[:9]

            #         print("sent msg pack bytes: ", len(msg_pack))
            #         # printArrayHexBinary(msg_pack)

            #         del reprogram_msg[:9]

            #         print("remaining reporgram msg bytes: ", len(reprogram_msg))
            #         # printArrayHexBinary(reprogram_msg)

            #         seri.write(msg_pack)
            #         res = bytearray(seri.readline(1))
            #         print(res)

            #         print("1")

            #         if not reprogram_msg:
            #             break
            #         print("2")
            #         if res[0] == 0x06:
            #             print("3")
            #             continue
            #         elif res[0] == 0x15:
            #             print("4")
            #             error_count += 1
            #             print("error occured")
            #             reprogram_msg = msg_pack + reprogram_msg
            #             if error_count == 3:
            #                 print("5")
            #                 break
            #     print("6")
            # print("7")

            # bcc = 0x01
            # # tüketim sorgusu
            # loadFormat = bytearray(
            #     b"\x01\x52\x32\x02\x50\x2E\x30\x31\x2823-10-17,14:00;23-10-18,15:00\x29\x03"
            # )
            # for b in loadFormat:
            #     bcc ^= b
            # loadFormat.append(bcc)
            # print("data to send device: ", loadFormat)
            # seri.write(loadFormat)

            # xor_check = 0x02
            # for i in range(500):
            #     data = bytearray(seri.readline())
            #     if len(data) == 0:
            #         break
            #     print(data)
            #     if len(data) != 3:
            #         for b in data:
            #             xor_check ^= b
            #     else:
            #         xor_check ^= data[0]
            #         xor_check ^= data[1]
            #         print("xor check:", hex(xor_check))
            #         print(xor_check)

            # loadFormat2 = bytearray(b'\x01\x52\x32\x02\x50\x2E\x30\x31\x282308102315;2308110100\x29\x03')
            # bcc2= 0x01
            # for b in loadFormat2:
            #     bcc2 ^= b
            # loadFormat2.append(bcc2)
            # print(loadFormat2)
            # seri.write(loadFormat2)
            # for i in range(20):
            #     print(seri.readline())

            # bcc_time = 0x01
            # timeSet = bytearray(b'\x01\x57\x32\x02\x30\x2E\x39\x2E\x31\x2813:00:40\x29\x03')
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

            # bcc_password = 0x01
            # passwordMessage = bytearray(b'\x01\x50\x31\x02\x28\x29\x03')
            # for b in passwordMessage:
            #     bcc_password ^= b
            # passwordMessage.append(bcc_password)
            # print(passwordMessage)
            # seri.write(passwordMessage)
            # print(bytearray(seri.readline()))

            # bcc_date = 0x01
            # dateSet = bytearray(
            #     b"\x01\x57\x32\x02\x30\x2E\x39\x2E\x32\x2823-10-17\x29\x03"
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
            # time.sleep(0.2)
            # while True:
            #     res = bytearray(seri.readline())
            #     if len(seri.readline()) == 0:
            #         break
            #     else:
            #         print("message received: ",res)
