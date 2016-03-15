
class grab_ball():
	def __init__(self):
		self.initiate_pickup():
	
	def initiate_pickup():
		self.lower_arm()
		while(True):
		image = self.get_image()
			if(image == center):
				self.stop()
				break;
			elif(image == right):
				move_to_side(left)
			elif(image == left):
				move_to_side(right)
		self.lower_arm()
		self.grab_ball()
		self.raise_arm()
					
	def pickup_ball():
		self.lower_arm()
		self.grab_ball()
		self.raise_arm()

	def stop():
		return 'q'	

	def center_arm():
		return 'c'  # writing c to serial will center arm. 

	def lower_arm():
		return 'm10100'	# lowers the arm all the way down
	
	def raise_arm():
		return 'm11100'

	def raise_arm_over_ball():
		return 'm11003'

	def move_to_side(direction):
		
		if(direction == 'right'):
			return 'm40100'
		elif(direction =='left':
			return 'm41100'
	
	def grab_ball():
		return 'm31100'

	def drop_ball():
		return 'm30100'

	def get_image():
		# call get image from the top camera
