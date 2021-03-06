

from lidar_faker_class import *
import time
from visual import *
from visual.text import *

lidarData = [[] for i in range(360)] #A list of 360 elements Angle, Distance , quality

offset = 140
init_level = 0
index = 0
fps_rate = 60
visualization = True
resolution = 3
lidar = lidar_faker(resolution)

#setup the window, view, etc
scene =  display(title='PROTOX1 LIDAR Data',
     x=0, y=0, width=1200, height=800,
     center=(5,0,0), background=(0,1,1))

scene.forward = (1, -.5,0)
scene.background = (0.1, 0.1, 0.2)
scene.title = "ProtoX1 Lidar Distance"
scene.show_rendertime = 1

class grid:
    """A graphical grid, with two level of subdivision.

    The grid can be manipulated (moved, showed/hidden...) by acting on self.frame.
    """
    
    def __init__(self, size=100, small_interval=10, big_interval=50):
        self.frame = frame(pos=(0,0,0))
        for i in range(-size, size+small_interval, small_interval):
            if i %big_interval == 0:
                c = color.gray(0.65)
            else:
                c = color.gray(0.25)
            curve(frame=self.frame, pos=[(i,0,size),(i,0,-size)], color=c)
            curve(frame=self.frame, pos=[(size,0,i),(-size,0,i)], color=c)

#grid where major intervals are 1m, minor intervals are 10cm
my_grid = grid(size=4000, small_interval = 100, big_interval=1000)
my_grid.frame.pos.y=-5


# sample and intensity points
point = points(pos=[(0,0,0) for i in range((129600/resolution))], size=5, color=(0 , 1, 0))
#lines=[curve(pos=[(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0)),(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0))], color=[(0.1, 0.1, 0.2),(1,0,0)]) for i in range(360)]
#zero_intensity_ring = ring(pos=(0,0,0), axis=(0,1,0), radius=offset-1, thickness=1, color = color.yellow)

#label_speed = label(pos = (0,-500,0), xoffset=1, box=False, opacity=0.1)
#label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)

use_points = True
use_outer_line = False
use_lines = True
use_intensity = True

def update_view( x_degree, y_degree, dist, quality ):
	"""Updates the view of a sample.

	Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
	"""
	global resolution, offset, use_outer_line, use_line

	x_degree_rad = x_degree * math.pi / 180.0
	y_degree_rad = y_degree * math.pi / 180.0
	c = math.cos(x_degree_rad)
	cy = math.cos(y_degree_rad)
	s = -math.sin(x_degree_rad)
	sy = -math.sin(y_degree_rad)
	dist_mm = dist

	#quality = x2 | (x3 << 8) # quality is on 16 bits
	#lidarData[x_degree] = [dist_mm,quality]
	dist_x = dist_mm*c
	dist_y = dist_mm*cy
	dist_z = dist_mm*s
	dist_zy = dist_mm*sy
	#print "degrees:", x_degree, y_degree
	#print "distance:", dist_x , dist_y, dist_z
	#print c, cy, s, sy
	
	#display point 	
	num = ((360/resolution*x_degree) + y_degree)
	#print "num:", num
	v1 = vector(dist_y, 0, dist_zy)
	v2 = rotate(v1, (math.radians(90)), axis=(1,0,0))
	v3 = rotate(v2, (math.radians(90-x_degree)), axis=(0,1,0))
	#print "vectors:", v1, v2
	point.pos[num] = v3
	#pointb.pos[x_degree] = v2
	#print len(point.pos)
	#print point.pos[x_degree]
	#raw_input()
'''
    if visualization:

        if not use_lines : lines[ x_degree].pos[1]=(offset*c,0,offset*s)
        if not use_outer_line :
            outer_line.pos[ x_degree]=(offset*c,0,offset*s)
            outer_line.color[ x_degree] = (0.1, 0.1, 0.2)
        
        
        # display the sample
        if quality < 1: # is the flag for "bad data" set?
            # yes it's bad data
            lines[ x_degree].pos[1]=(offset*c,0,offset*s)
            outer_line.pos[ x_degree]=(offset*c,0,offset*s)
            outer_line.color[ x_degree] = (0.1, 0.1, 0.2)
        else:
            # no, it's cool
            if quality > 10 :
                # X+1:6 not set : quality is OK
                if use_points : point.pos[ x_degree] = vector( dist_x,0, dist_y)
                if use_intensity : point2.pos[ x_degree] = vector( (quality + offset)*c,0, (quality + offset)*s)
                if use_lines : lines[ x_degree].color[1] = (1,0,0)
                if use_outer_line : outer_line.color[ x_degree] = (1,0,0)
            else:
                # Warning, the quality is not as good as expected
                if use_points : pointb.pos[ x_degree] = vector( dist_x,0, dist_y)
                if use_intensity : point2b.pos[ x_degree] = vector( (quality + offset)*c,0, (quality + offset)*s)
                if use_lines : lines[ x_degree].color[1] = (0.4,0,0)
                if use_outer_line : outer_line.color[ x_degree] = (0.4,0,0)
            if use_lines : lines[ x_degree].pos[1]=( dist_x, 0, dist_y)
            if use_outer_line : outer_line.pos[ x_degree]=( dist_x, 0, dist_y)
'''     
message = text(pos=(0,0,0), string='PROTOX1 DATA', justify='center',
               color=color.yellow, axis=(0,0,1), height=100,
                depth=0.3, up=(10,10,-0.3))
time.sleep(5)
while True:

	print lidar.x_degree, lidar.y_degree, lidar.dist, lidar.quality
 	update_view (lidar.x_degree, lidar.y_degree, lidar.dist, lidar.quality )
	#rate(60) # synchonous repaint at 60fps
	if scene.kb.keys: # event waiting to be processed?
		s = scene.kb.getkey() # get keyboard info
		if s == "q": # stop motor
			print "hi"
			#sys.exit(-1)
