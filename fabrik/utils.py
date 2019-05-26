import numpy as np

class VectorUtils():
	
	def distance(p1,p2):
		#print(p1,p2)
		dif = (p2[0] - p1[0] , p2[1] - p1[1])
		return np.sqrt(dif[0]*dif[0] + dif[1]*dif[1])
	
	def get_projection2d(a,b,c):
		# a,b,c are points 
		# this function try to find projection point C' on
		# the line run through a,b
		# 1) split them out
		xa,ya = a
		xb,yb = b
		xc,yc = c
	
		px = xb - xa
		py = yb - ya
		delta_ab = px * px + py * py
		u0 = (xc - xa ) * px + (yc - ya) * py
		u = u0 / delta_ab
		x = xa + u * px
		y = ya + u * py
		return [x,y]
		
	def intersect(L1,L2)	:
		# L1 = [(x1,y1),(x2,y2)]
		# L2 = [(x3,y3),(x4,y4)]
		(x1,y1),(x2,y2) = L1
		(x3,y3),(x4,y4) = L2
		
		d = (x1-x2) * (y3-y4) - (x3-x4) * (y1-y2)
		a = (x1*y2-y1*x2)
		b = (x3*y4 - y3*x4)
		
		x = a * (x3-x4) - b * (x1-x2)
		y = a * (y3-y4) - b * (y1-y2)
		
		x = x / d
		y = y / d
		
		return (x,y)
		
	def quadrant(P1,P2):
		# what quadrant P2 is compare to P1
		x1,y1 = P1
		x2,y2 = P2
		qx = []
		qy = []
	
		if x2 < x1 :
			qx = [-1,0]
		else :
			qx = [1, 0]
			
		if y2 < y1 :
			qy = [0,-1]
		else :
			qy = [0,1]
	
		qd = [qx[0] + qy[0],qx[1] + qy[1]]
		
		return(qd)
    
	def load(filename):        
		return np.load(filename)
	
	
class GeoUtils():
	def law_of_cosine(p1,p2) :
		try:
			delta_x = abs(p1[0] - p2[0])
			delta_y = abs(p1[1] - p2[1])
			magnitude = distance(p1,p2)
			cos_theta = (np.power(delta_y,2) + np.power(magnitude,2) - np.power(delta_x,2)) / (2 * delta_y * magnitude)
			theta = np.degrees(np.arccos(cos_theta))
			return theta
		except ZeroDivisionError :
			raise 'Divided by zero.'	
