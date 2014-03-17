#!/usr/bin/python

import time, serial, sys

class protox():
	def __init__(self, com_port):
		# setup serial port
		self.baudrate = 115200
		self.serial_com = serial.Serial(com_port, self.baudrate)
	
	def run(self):
		while True:
			if (self.ser.isOpen() == False):
				self.ser = serial.Serial(self.com_port, self.baudrate)
				time.sleep(0.1)
		
	def read_lidar(self):
			data = []
			data = self.serial_com.readline().strip('\r\n').split(',')
			return data


if __name__== "__main__":

	com_port = "/dev/ttyUSB0"
	if len(sys.argv) > 1:
		com_port = sys.argv[1]
	lidar = protox(com_port)
	
	while True:
		lidar_data = lidar.read_lidar()
		print lidar_data
		
		
    
