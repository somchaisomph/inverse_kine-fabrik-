from sys import exit
from fabrik.vector import *
from fabrik.chain import *
import random
from fabrik.utils import VectorUtils
import numpy as np

def solve_internal_angle(points):	
	d1 = distance(points[0],points[1])
	d2 = distance(points[1],points[2])
	d3 = distance(points[2],points[0])
	unit_v = Vector2D((1,0))
	cos_beta = (np.power(d1,2) +np.power(d2,2) - np.power(d3,2))/(2*d1*d2)	
	beta = np.degrees(np.arccos(cos_beta))
	alpha = unit_v.angle(Vector2D(points[1]))

	
	return(np.degrees(alpha),beta)

 
#create chain
ikc = IKCHAIN()

# add first joint with origin type at coordinate of 400,0
ikc.add_joint([0,0],JOINT_DICT['origin'])

# add 2nd joint with normal type at coordinate of 400,250
ikc.add_joint([0,450],JOINT_DICT['normal'])

# add last joint with end_effector type at coordinate of 400,550
ikc.add_joint([0,850],JOINT_DICT['end_effector'])

ikc.update_linkage()

#push given chain into fabrik instance to create solution
fabrik = FBRIK(ikc)

# first target point
target_point = [245,280]
	
#turn target point to tempory joint 
temp_joint = vt.JOINT(target_point,JOINT_DICT['target'])
	
# start solving
fabrik.solve(temp_joint)
	
#get solutions for each joint 
coord = fabrik.get_resolution(type='point') # return in coordinate
print("Resolution to reach target at {}".format(target_point))
print("In coordinates : {}".format(coord ))
alpha,beta = solve_internal_angle(coord)
print(alpha,beta)
