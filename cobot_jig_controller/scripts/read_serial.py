import serial
from time import sleep


com = serial.Serial('/dev/ttyUSB1',
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

    c = 0
    a = []
    b = []
    while True:
      ret = com.read()
      if ret:
        r = ord(ret)
        b.append(r)
        if len(b)>=3 and b[-3:]==[255, 255, 253]:
          print(a)
          a = [hex(c) for c in b[-3:]]
          b = []
        elif len(a)>0:
          a.append(hex(r))
  finally:
    com.close()
else:
  print('failed')

