import serial

connection = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout = 10)

connection.write('m10010')

connection.write('s')
test = connection.readline()

print test

