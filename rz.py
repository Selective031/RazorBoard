
import serial
import time
import blynklib
import Adafruit_DHT

BLYNK_AUTH = "xxxxxx"

blynk = blynklib.Blynk(BLYNK_AUTH)

RAZ = serial.Serial('/dev/ttyUSB0', 115200)

time.sleep(3)

RAZ.write(b'debug on\r\n')

timecount = 0

@blynk.handle_event('write V*')
def write_handler(pin, value):

    if pin == 0 :
     if value[0] == '1':
       RAZ.write(b'DISABLE\r\n')
       print ("DISABLE")
     if value[0] == '0':
       RAZ.write(b'ENABLE\r\n')
       print ("ENABLE")

    if pin == 3:
      RAZ.write(b'track perimeter\r\n')

while True:
  time.sleep(1)

  blynk.run()

  timecount += 1
  if timecount == 10:
    humidity, temperature = Adafruit_DHT.read(11, 7)
    if humidity is not None and temperature is not None:
      blynk.virtual_write(15, temperature)
      blynk.virtual_write(18, temperature)
      blynk.virtual_write(16, humidity)
      blynk.virtual_write(19, humidity)


    timecount = 0

  while RAZ.in_waiting:
    try:
      pass
      RAZ_CMD = RAZ.readline().decode().strip()
      print (RAZ_CMD)

      List = RAZ_CMD.split()

      if List[0] == "V1:":
        blynk.virtual_write(1, List[1])
        blynk.virtual_write(17, List[1])
      if List[0] == "M1:":
        blynk.virtual_write(5, List[1])
      if List[0] == "M2:":
        blynk.virtual_write(6, List[1])
      if List[0] == "C1:":
        blynk.virtual_write(7, List[1])
      if List[0] == "IN->":
        if List[1] == "BWF1:":
         blynk.virtual_write(9, List[2])
        if List[3] == "BWF2:":
         blynk.virtual_write(10, List[4])

      if List[0] == "Magnitude":
         blynk.virtual_write(12, List[3])
         blynk.virtual_write(13, List[5])

      if List[0] == "Security":
         blynk.virtual_write(14, List[1])
      if List[0] == "State":
         blynk.virtual_write(20, List[1])

      if List[0] == "Movement:":
        blynk.virtual_write(4, List[1])
      if List[0] == "Movement" and List[1] == "Verdict:":
        blynk.virtual_write(11, List[2])
      if List[0] == "Charger" and List[1] == "Connected:":
        if List[2] == "1":
          blynk.virtual_write(8, 255)
        else: 
          blynk.virtual_write(8, 0)


    except:
      pass

