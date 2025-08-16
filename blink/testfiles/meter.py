import serial
import time

def calculateBCC(data, xor):
    for i in data:
        xor ^= i

    return xor


seri = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=300,
    bytesize=7,
    parity="E",
    stopbits=1,
    timeout=2
)

while True:
    line = bytearray(seri.readline())

    if line:
        print(line)
        break

time.sleep(0.25)
res = bytearray(b"/MSY5<2>MAVIALP\r\n")
seri.write(res)

while True:
    line = bytearray(seri.readline())

    if line:
        print(line)
        break

time.sleep(0.25)

seri.close()
seri = serial.Serial(
    "/dev/ttyUSB0",
    baudrate=9600,
    bytesize=7,
    parity="E",
    stopbits=1,
    timeout=2,
)
time.sleep(0.25)

ro_data = bytearray(b'\x020.0.0(612400088)\r\n0.2.0(V1.1.0)\r\n0.8.4(15*min)\r\n0.9.1(00:03:10)\r\n0.9.2(00-01-01)\r\n96.1.3(24-03-13)\r\nT.V.1(005)\r\n32.7.0(0.00)\r\n52.7.0(0.00)\r\n72.7.0(0.00)\r\n!\r\n\x03')
ro_data.append(calculateBCC(ro_data, ro_data[0]))
seri.write(ro_data)