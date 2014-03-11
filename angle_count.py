from lidar3d_class import *
import time

lidar = mobot_lidar3d("/dev/ttyUSB0", 115200, 3)

angle_count = [0] * 360
print type( angle_count)
max_count = (360 * 3)
count = 0
angle_zero_dist = [0] * 360

while True:
	lidar_data = lidar.read_lidar()
	

	if (len(lidar_data) > 0):
		#print lidar_data
		try:
			x_degree = int(lidar_data[1])
			x_degree = 0
			#print self.y_degree 
		except:
			x_degree = 0
			pass
		try:
			y_degree = int(lidar_data[2])
			#y_degree = y_degree #+ 90
			#print x_degree
		except:
			y_degree = 0
			pass
		try:
			dist = int(lidar_data[3])
			#print dist
		except:
			dist = 0 
			pass
		try:
			quality = int(lidar_data[4])
			#print dist
		except:
			quality = 0 
			pass
		#reset_dist_data()
		if dist > 0:
			angle_zero_dist[y_degree] = angle_zero_dist[y_degree] + 1
		if quality > 0:
		#	#print y_degree
			angle_count[y_degree] = angle_count[y_degree] + 1
		count += 1
		
		if count >= max_count:
			count = 0
			print "-" * 50
			num = 0
			zero_dist_num = 0
			for i in range (360):
				if angle_count[i] < 1:
					num += 1
					print "angle[", i, "]:", angle_count[i]
				if angle_zero_dist[i] < 1:
					zero_dist_num += 1
			print "number angle with zero dist:", zero_dist_num		
			print "number angle with no data:", num
			#angle_count = [0] * 360
			#angle_zero_dist = [0] * 360
				 
		#time.sleep(0.1)
		#reset_data()
	else:
		#time.sleep(3)
		print "no data"
		#print lidar.x_degree, lidar.y_degree, lidar.dist, lidar.quality
