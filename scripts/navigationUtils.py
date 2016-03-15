class collisionDetection():
	def __init__(self, connection):
		# Initialize all sensor readings to 0
		self.readingFront = 0
		self.readingLeft = 0
		self.readingRight = 0
		self.connection = connection

	# This function gets the sensor readings for all sensors
	def getSensorReadings(self):
		# connect to Arduino
		self.connection.write('u1')
		self.readingFront = int(connection.readline())

		self.connection.write('u2')
		self.readingLeft = int(connection.readline())

		self.connection.write('u3')
		self.readingRight = int(connection.readline())

	# This returns sensor reading only for a given sensor
	def getSensorReading(self, sensor):
		if sensor == 1:
			self.connection.write('u1')
			self.readingFront = int(connection.readline())

		elif sensor == 2:
			self.connection.write('u2')
			self.readingLeft = int(connection.readline())

		elif sensor == 3:
			self.connection.write('u3')
			self.readingRight = int(connection.readline())
		else:
			print "ERROR : wrong sensor id passed"


#########################################################################
# Should be in the controller.py

	def moveAroundObjectDetected(self):
		# Object is detected turn left 90 degrees
		rotateLeft90Degress()

		# Now move robot till right ultrasound sensor does not
                # detect any object
		while(this.readingRight < 20):
			# move the controller a little bit forward at
                        # say 1 second step
			self.driveRobot(100,0)
			rospy.sleep(1)
			self.driveRobot(0,0)
			# Update the right sensor reading
			getSensorReading(3)

		# So now the right sensor reading is clear, move a bit
                # more forward so that we account for robot`s body length
                # as well
		self.driveRobot(100,0)
		rospy.sleep(2)
		self.driveRobot(0,0)

		# Now we are clear of the object, rotate to the right 90
                # degress so that we are in the same direction
		rotateRight90Degrees()

	def rotateLeft90Degress(self):
		current_angle = self.odom_estimate.angle
		target_angle = current_angle + 90
		self.driveRobot(0, 90)
		while(current_angle < target_angle):
			rospy.sleep(0.25)
			current_angle = self.odom_estimate.angle
		self.driveRobot(0,0)

	def rotateRight90Degrees(self):
		current_angle = self.odom_estimate.angle
		target_angle = current_angle - 90
		self.driveRobot(0, 90)
		while(current_angle > target_angle):
			rospy.sleep(0.25)
			current_angle = self.odom_estimate.angle
		self.driveRobot(0,0)

	def rotateLeft90DegreesRandom(self, maxTime):
		current_angle = self.odom_estimate.angle
		self.driveRobot(0, 90)
		# Sleep for a random time between 0s to 4s
		rospy.sleep(random.random() * maxTime)
		self.driveRobot(0,0)

	def driveRobotForwardRandom(self, maxTime):
		self.driveRobot(100,0)
		rospy.sleep(random.random() * maxTime)
		self.driveRobot(0,0)

	def navigateRandomly(self):
		# Initial state , move within 180 degress first
		# import random should be done
		# Move randomly till the camera detects a ball
		while self.front_cam.see_ball not True:
			# Sleep for a random time between 0s to 4s
			rotateLeft90DegressRandom(4);
			driveRobotForwardRandom(4);
