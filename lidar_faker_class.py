

import random
import thread
import time 




class lidar_faker():
	def __init__(self,resolution=1):
		self.x_degree = 0
		self.y_degree = 0
		self.dist = 0
		self.quality = 0
		self.rpm = 0
		self.resolution = resolution
		self.clockwise = True
		self.th = thread.start_new_thread(self.run, ())
	
	def run(self):
		while True:
			time.sleep(.00001)
			self.read_lidar()
			
	def read_lidar(self):
		self.y_degree += self.resolution
		if self.y_degree > 360:
			self.y_degree = 0
			if self.clockwise == True:
				self.x_degree += self.resolution
			else:
				self.x_degree -= self.resolution
			if self.x_degree > 180:
				self.clockwise = False
			if self.x_degree < 0:
				self.clockwise = True
	
		self.dist = random.randint(950,1000)
		#self.dist = 1000
		self.quality = random.randint(0,100)
		self.rpm = random.randint(281 ,295)
