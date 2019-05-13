import serial
from time import sleep


com = serial.Serial('/dev/ttyUSB0',
                     baudrate=3000000,
                     bytesize=serial.EIGHTBITS,
                     parity=serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=1,
                     xonxoff=0,
                     dsrdtr=1,
                     rtscts=0
                     )
#com.setDTR(False)

if com.isOpen():
  try:
    print('start0')
    msgs = []
    msgs.append(bytearray([255, 255, 253, 0, 1, 6, 0, 3, 50, 2, 0, 53, 108]))
    msgs.append(bytearray([255, 255, 253, 0, 1, 6, 0, 3, 11, 0, 1, 71, 99]))
    msgs.append(bytearray([255, 255, 253, 0, 1, 7, 0, 3, 92, 2, 0, 0, 105, 237]))
    msgs.append(bytearray([255, 255, 253, 0, 1, 6, 0, 3, 50, 2, 1, 48, 236]))
    msgs.append(bytearray([255, 255, 253, 0, 2, 6, 0, 3, 50, 2, 0, 5, 111]))
    msgs.append(bytearray([255, 255, 253, 0, 2, 6, 0, 3, 11, 0, 1, 119, 96]))
    msgs.append(bytearray([255, 255, 253, 0, 2, 7, 0, 3, 92, 2, 0, 0, 99, 221]))
    msgs.append(bytearray([255, 255, 253, 0, 2, 6, 0, 3, 50, 2, 1, 0, 239]))
    msgs.append(bytearray([255, 255, 253, 0, 3, 6, 0, 3, 50, 2, 0, 22, 238]))
    msgs.append(bytearray([255, 255, 253, 0, 3, 6, 0, 3, 11, 0, 1, 100, 225]))
    msgs.append(bytearray([255, 255, 253, 0, 3, 7, 0, 3, 92, 2, 0, 0, 101, 205]))
    msgs.append(bytearray([255, 255, 253, 0, 3, 6, 0, 3, 50, 2, 1, 19, 110]))
    msgs.append(bytearray([255, 255, 253, 0, 4, 6, 0, 3, 50, 2, 0, 101, 105]))
    msgs.append(bytearray([255, 255, 253, 0, 4, 6, 0, 3, 11, 0, 1, 23, 102]))
    msgs.append(bytearray([255, 255, 253, 0, 4, 7, 0, 3, 92, 2, 0, 0, 119, 189]))
    msgs.append(bytearray([255, 255, 253, 0, 4, 6, 0, 3, 50, 2, 1, 96, 233]))
    msgs.append(bytearray([255, 255, 253, 0, 5, 6, 0, 3, 50, 2, 0, 118, 232]))
    msgs.append(bytearray([255, 255, 253, 0, 5, 6, 0, 3, 11, 0, 1, 4, 231]))
    msgs.append(bytearray([255, 255, 253, 0, 5, 7, 0, 3, 92, 2, 0, 0, 113, 173]))
    msgs.append(bytearray([255, 255, 253, 0, 5, 6, 0, 3, 50, 2, 1, 115, 104]))
    msgs.append(bytearray([255, 255, 253, 0, 6, 6, 0, 3, 50, 2, 0, 70, 235]))
    msgs.append(bytearray([255, 255, 253, 0, 6, 6, 0, 3, 11, 0, 1, 52, 228]))
    msgs.append(bytearray([255, 255, 253, 0, 6, 7, 0, 3, 92, 2, 0, 0, 123, 157]))
    msgs.append(bytearray([255, 255, 253, 0, 6, 6, 0, 3, 50, 2, 1, 67, 107]))
    msgs.append(bytearray([255, 255, 253, 0, 7, 6, 0, 3, 50, 2, 1, 67, 107]))

#    while True:
#      ret = com.read()
    print(len(msgs))
    for s in msgs:
      com.write(s)
      sleep(0.01)
  finally:
    com.close()
else:
  print('failed')

