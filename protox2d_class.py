#!/usr/bin/python

import time
import serial

class protox():
	def __init__(self, com_port):
		# serial port
		self.com_port = com_port
		self.baudrate = 115200
		self.ser = serial.Serial(self.com_port, self.baudrate)
	
	def run(self):
		while True:
			if (self.ser.isOpen() == False):
				self.ser = serial.Serial(self.com_port, self.baudrate)
				time.sleep(0.1)
		
	def read_lidar(self):
			data = []
			self.x_degree = 0
			self.y_degree = 0
			self.dist = 0
			data = self.ser.readline().strip('\r\n').split(',')
			return data


if __name__== "__main__":

	lidar = protox("/dev/ttyUSB0")
	
	while True:
		lidar_data = lidar.read_lidar()
		print lidar_data
		
		
    
