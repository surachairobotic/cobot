import serial
from time import sleep


com = serial.Serial('/dev/ttyACM0',
                     baudrate=9600,
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

    while True:
        com.write("$id#")
        sleep(0.01)
        print("READ : %s" % com.read())
  finally:
    com.close()
else:
  print('failed')
