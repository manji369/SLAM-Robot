import serial, time
importRPi.GPIO as GPIO
ser = serial.Serial('/dev/ttyAMA0', 9600)
pin = 12
GPIO.setmode(GPIO.BOARD)
# set up GPIO output channel
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin,GPIO.HIGH)
time.sleep(0.5)
GPIO.output(pin,GPIO.LOW)
time.sleep(0.1)
GPIO.output(pin,GPIO.HIGH)
time.sleep(3)
out = 0
while(out<20):
ser.write("G")
print("printed G")
out = ser.readline()
out = float(out)
	print(out)
time.sleep(1)
time.sleep(3)
ser.write("G")
print("printed G")
out = ser.readline()
out = float(out)
print(out)
gyr1 = out
ser.write("E")
out = ser.readline()
front_sensor = int(out)
print("front sensor = "+out+"\n")
ser.write("F")
out = ser.readline()
side_sensor = int(out)
print("side sensor = "+out+"\n")
spd1 = 200
spd2 = 200
while(front_sensor>30):
	time.sleep(1)
	ser.write("J")
	time.sleep(0.1)
	ser.write("E")
	out = ser.readline()
	front_sensor = int(out)
	#print("front sensor = "+out+"\n")
#	ser.write("F")
#	out = ser.readline()
#	side_sensor = int(out)
	#print("side sensor = "+out+"\n")
	ser.write("G")
out = ser.readline()
gyr = float(out)
print("gyro = "+str(gyr)+" Fr = "+str(front_sensor)+"\n")
	comp = (gyr-gyr1)*(0.1)*(5)
	if(gyr>gyr1+2):
		spd11 = int(spd1/2)
		spd21 = int(spd2/2)
		ser.write("B\n")
	ser.write(str(spd11)+"\n")
	ser.write(str(spd21))
		spd1 = int(spd1 - comp)
		spd2 = int(spd2 + comp)
		if(spd2>255):
			spd2 = 255
		if(spd1<0):
			spd1 = 0
	elif(gyr<gyr1-2):
		spd11 = int(spd1/2)
                spd21 = int(spd2/2)
		ser.write("B\n")
ser.write(str(spd11)+"\n")
ser.write(str(spd21))
		spd1 = int(spd1 - comp)
		spd2 = int(spd2 + comp)
	if(spd1>255):
		spd1 = 255
	if(spd2<0):
		spd2 = 0
	if(spd2>255):
                spd2 = 255
if(spd1<0):
                spd1 = 0
	print("spd1 = "+str(spd1)+" spd2 = "+str(spd2)+"\n")
	ser.write("B\n")
	ser.write(str(spd1)+"\n")
	ser.write(str(spd2))
#	ser.write("I")
#	time.sleep(0.05)
ser.write("J")
ser.close