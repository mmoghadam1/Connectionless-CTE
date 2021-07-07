import serial
import struct
s = serial.Serial('/dev/tty.usbmodem0004401673681');
s.flush()
"""
PACKET_SIZE = 179  # 164 + 4 +2 + 1 + 1 + 4 + 1 + 2
found = False
while not found:
    # Look for 4 0xFFs in a row...
    msg = s.readline()
    index = msg.find(b'\xff'*4)
    if index >=0:
        found = True
        #  Figure how much to read to get to next 0xFFs
        amount_xferred = len(msg) - index
        s.read(PACKET_SIZE - amount_xferred)
"""
NUM_PACKETS = 10
onefound = False
while not onefound:
    # Look for 4 0xFFs in a row...
    msg = s.readline()
    index = msg.find(b'\xff'*4)
    if index >=0:
        onefound = True
PACKET_SIZE = len(msg) - index
found = False
msg2 = b''
while not found:
    msg2 += s.readline()
    index = msg2.find(b'\xff'*4)
    if index >=0:
        found = True
PACKET_SIZE += index
print(PACKET_SIZE)
amount_xferred = len(msg2) - index
s.read(PACKET_SIZE - amount_xferred)

filename = 'connectionless_20ms_chmap.dat'
with open(filename, 'wb') as f:

    for count in range(NUM_PACKETS):
        packet = s.read(PACKET_SIZE)
        f.write(packet)
        f.flush()
        data = struct.unpack('<4sHBBIB164p2s', packet)
        print(count, data[1], data[2], -data[3])

s.close()
