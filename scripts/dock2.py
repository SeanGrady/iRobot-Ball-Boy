#!/usr/bin/env python
import struct
import sys, glob # for listing serial ports
import roslib
roslib.load_manifest('create2_dock')
import sys, os, os.path
import rospy
#import numpy as np
try:
    import serial
except ImportError:
    print 'Import error', 'Please install pyserial.'
    raise

connection = None


class dock:
	def onConnect(self):
	    	global connection

	    	if connection is not None:
	      		print 'Oops Youre already connected!'
	      		return

	    	port = '/dev/ttyUSB0'
	    	if port is not None:
	      		print "Trying " + str(port) + "... "
	      		try:
				connection = serial.Serial(port, baudrate=115200 , timeout=1)
				print "Connected!"
	      		except:
				print "Failed."



	def sendCommandRaw(self, command):
	    global connection

	    try:
	      if connection is not None:
		connection.write(command)
	      else:
		print "Not connected."
	    except serial.SerialException:
	      print "Lost connection"
	      connection = None


	def sendCommandASCII(self, command):
	      cmd = ""
	      for v in command.split():
		  cmd += chr(int(v))

	      self.sendCommandRaw(cmd)

	def selfDrive(self):
		self.onConnect()
		self.sendCommandASCII('128')
		self.sendCommandASCII('131')
		#self.sendCommandASCII('135')
		self.sendCommandASCII('143')
		self.sendCommandASCII('135')
		rate = rospy.Rate(0.2)


def main(args):
  dock_bot = dock()
  rospy.init_node('dock', anonymous=True)
  dock_bot.selfDrive()
  
  try:
    rospy.spin();
  except KeyboardInterrupt:
    self.sendCommandASCII('135')

if __name__ == '__main__':
  main(sys.argv)
