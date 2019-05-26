import math
import numpy as np

def distance(v1,v2):
	#if isinstance(v1,VectorNP) and isinstance(v2,VectorNP) :
	dif = v2.get() - v1.get()
	return np.sqrt(dif[0]*dif[0] + dif[1]*dif[1])

def perspective_project(vector3, d):
	x, y, z = vector3.get()
	return (x * d/z, -y * d/z)

def calculate_viewing_distance(fov, screen_width): 
	d = (screen_width/2.0) / np.tan(fov/2.0)
	return d



class MyError(Exception):
	def __init__(self,value):
		self.value = value
		
	def __str__(self):
		return repr(self.value)

class VectorNP():
	def __init__(self,elements):
		self._v = np.array(elements)
				
	def magnitude(self):
		#sqrt(x**2 + y**2)
		return np.linalg.norm(self._v)
		
	def get_unitvector(self):
		mag = self.magnitude()
		return VectorNP(self._v / mag	)	
	
	def scale(self,scale_v):
		#scale_v is scale vector or matrix in term of numpy array
		if self._v.shape == scale_v.shape :
			self._v *= scale_v
			return True
		else :
			return False
	
	def translate2d(self,tm):
	#	# tm is translation vector or matrix
		self._v += tm	
		
	def T(self):
		self._v = self._v.T
		
	def get_transpose(self):
		return self._v.T	
			
	def size(self):
		return self.get().shape
	
	#def rotate2d(self,degree):
	#	rad = np.radians(degree)
	#	rm = np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
	#	res = np.dot(rm,self._v.T)
	#	self._v = res
	
	#def __call__(self):
	#	return self._v
	'''	
	def __truediv__(self,v):
		if isinstance(v,VectorNP):
			return VectorNP(self()/v())
		elif isinstance(v,int) or isinstance(v,float):
			return VectorNP(self.get()/v)
	'''
	'''
	def __mul__(self,v):
		if isinstance(v,VectorNP):
			if self.size() == v.size() :
				return VectorNP(self.get() * v())
			else :
				return None
		elif isinstance(v,int) or isinstance(v,float):
			return VectorNP(self.get() * v)
	'''
	'''
	def __add__(self,v):
		if isinstance(v,VectorNP):
			if self.size() == v.size() :
				return VectorNP(self.get() + v.get())
			else :
				return None
		elif isinstance(v,int) or isinstance(v,float):
			return VectorNP(self.get() - v)
	'''
	'''		
	def __sub__(self,v):
		if self.size() != v.size() : 
			return None
		if isinstance(v,int) or isinstance(v,float):
			return VectorNP(self() - v())
		return VectorNP(self() - v())
	'''	
	'''		
	def __sub__(self,v):
		if self.size() != v.size() : 
			return None
		diff = self.get() - v.get()	
		if isinstance(v,int) or isinstance(v,float):
			return VectorNP(self.get() - v.get())
		return VectorNP(self.get() - v.get())
	'''
					
	def __neg__(self):
			self._v *= -1	
			
	def set(self,new_element):
		self._v = np.array(new_element)
		
	def get(self):
		return self._v	
		
	def distance(self,v):
		dif = self.get() - v.get()
		return np.sqrt(dif[0]*dif[0] + dif[1]*dif[1])

	
	
					
#---------------------------------------------------------------------------------------
class Vector2D(VectorNP):
	def dot(self,v):
		# return scalar 
		x1,y1 = self.get()
		x2,y2 = v.get()
		return x2 * x1 + y2 *y1
		
	def size(self):
		return self.get().shape	
		
	def angle(self,v):
		# in radians
		d = self.dot(v)
		m1 = self.magnitude()
		m2 = v.magnitude()
		return np.arccos(d/(m1*m2))
		
	def perp_dot(self,v):
		#perpendicular dot 
		#v∗w|=|v||w|sin(α).
		# return scalar
		x1,y1 = self.get()
		x2,y2 = v.get()
		return x1*y2 - y1*x2	
	
	def project_on(self,w):
		# projection of self on w = vector with same size of self and  direction of w
		mw = w.magnitude()
		a = self.dot(w)/(mw * mw )
		proj_on_w = w * a
		return proj_on_w
			
	def perp_on(self,w):
		# find perpendicular on w
		mw = w.magnitude()
		a = self.perp_dot(w) / (mw * mw)		
		p = w.perp_vect()
		return (p * a) * -1
		#the alternate method
		#return self - self.project_on(w)
			
	def perp_vect(self):
		# return the same size vector with direction is perpendicular
		x,y = self.get()
		return Vector2D([-y,x])
		
	
	def __sub__(self,v):		
		if isinstance(v,int) or isinstance(v,float):
			return Vector2D(self.get() - v)
		elif isinstance(v,Vector2D):
			diff = self.get() - v.get()	
			return Vector2D([diff[0],diff[1]])	
		else :
			return None
		
	def __add__(self,v):
		if isinstance(v,int) or isinstance(v,float):
			return Vector2D(self.get() + v)	
		elif isinstance(v,Vector2D):
			return Vector2D(self.get() + v.get())
		else :
			return None	
		
			
	def __truediv__(self,v):
		if isinstance(v,int) or isinstance(v,float):
			return Vector2D(self.get()/v)
		elif isinstance(v,Vector2D):	
			return Vector2D(self.get()/v.get())
		else :
			return None	
		
	
	def __mul__(self,v):
		if isinstance(v,int) or isinstance(v,float):
			return Vector2D(self.get() * v)		
		elif isinstance(v,Vector2D):	
			return Vector2D(self.get() * v.get())
		else :
			return None
	
	def rotate(self,degree):
		# 1 turn degree to radian
		rad = np.radians(degree)		
		# 2. create rotation matrix
		# see https://somchaisom.blogspot.com/2018/03/computer-graphics-2-d-rotation.html
		rm = np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
		res = np.dot(rm,self._v.T)
		self._v = res
		
	def tusi_couple(self,degree,v2):
		# see https://en.wikipedia.org/wiki/Tusi_couple
		rad = np.radians(degree)		
		rm = np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
		center1 = np.dot(rm,self._v.T)
		
		#inside rotation
		rad = np.radians(-degree)
		rm = np.array([[np.cos(rad),-np.sin(rad)],[np.sin(rad),np.cos(rad)]])
		center2 = np.dot(rm,v2)
		v3 = center1 + center2	
		self._v = v3	
		
	def eclipse(self,degree,a,b):
		rad = np.radians(degree)	
		
		r = (a * b )/ np.sqrt(np.power(b * np.cos(rad),2) + np.power(a * np.sin(rad),2))
		
		x = r * np.cos(rad)
		y =	r * np.sin(rad)
		self.set([x,y])
		
#---------------------------------------------------------------------------------------		
class Vector3D(VectorNP):
	def dot(self,v):
		x1,y1,z1 = self.get()
		x2,y2,z2 = v.get()
		return x2*x1  + y1*y2 +  z2*z1	
		
	def angle(self,v):
		# in radians
		d = self.dot(v)
		m1 = self.magnitude()
		m2 = v.magnitude()
		return np.arccos(d/(m1*m2))	
		
	def __sub__(self,v):
		if self.size() != v.size() : 
			return None
		diff = self.get() - v.get()	
		if isinstance(v,int) or isinstance(v,float):
			return Vector3D(diff)
		return Vector3D(diff)	
		
	def __sub__(self,v):
		if self.size.get() != v.size() : 
			return None
		diff = self.get() - v.get()	
		if isinstance(v,int) or isinstance(v,float):
			return Vector3D(diff)
		return Vector3D(diff)	
		
	def __add__(self,v):
		if isinstance(v,int) or isinstance(v,float):
			return Vector3D(self() - v)	
		if self.size() == v.size() :
			return Vector3D(self.get() + v.get())
		else :
			return None

			
	def __truediv__(self,v):
		if isinstance(v,int) or isinstance(v,float):
			return Vector3D(self()/v)
		return Vector3D(self.get()/v.get())
		
	
	def __mul__(self,v):
		if isinstance(v,int) or isinstance(v,float):
			return Vector3D(self._v * v)		
		else :
			if self.size() == v.size() :
				return Vector3D(self() * v())
			else :
				return None		
#---------------------------------------------------------------------------------------
class JOINT(Vector2D):
	def __init__(self,element,type):
		super().__init__(element)
		self.type = type
		self.limit_angles = []
		
	def set_type(self,type):
		self.type = type
		
	def set_limit_angles(self,angles):
		#constrain = [theta1,theta2, theta3, theta4]
		#up,down,left,right
		self.limit_angles = angles
		
#---------------------------------------------------------------------------------------

def test1():
	v1 = VectorNP([2,2])
	v2 = VectorNP([1,5])
	v3d = VectorNP([2,5,3])
	v3 = v1 + v2
	v4 = v1 - v2
	#x,y,z = v3d()
	d = calculate_viewing_distance(600,np.radians(60))
	print(perspective_project(v3d,d))
	v1.rotate2d(45)
	print(v1())	

def test2():
	v1 = Vector2D([2,2])
	#v2 = v1.perp_vect()
	print(v2())

def test3():
	a = Vector2D([1,2])
	b = Vector2D([3,-2])
	c = Vector2D([2,0])
	d = Vector2D([-1,3])
	e = Vector2D([2,4])
	f = Vector2D([5,3])
	print(a.project_on(b).get())
	print(a.perp_dot(b))
	print(d.perp_dot(c))
	print(e.perp_dot(f))
	print(f.perp_vect().get())
	print(e.perp_on(f).get())
	
def test4():
	p1=[5,5]
	p2 =[15,5]
	p3 = [10,7]
	#a = Vector2D(p1)
	#b = Vector2D(p2)
	#c = Vector2D([(p3[0]-p1[0]),(p3[1]-p1[1])])
	#d = c.project_on((b-a))
	#c.translate2d(np.array([0,10]))
	#print((b-a)())
	#x,y = d.get()
	#print("{},{}".format(x+p1[0],y+p1[1]))
	
	v1 = Vector2D(p1)
	v2 = Vector2D(p2)
	v3 = Vector2D(p3)
	
	v2 = v2 - v1
	v3 = v3 - v1
	v4 = v3.project_on(v2)
	print((v4+v1).get())
	
if __name__ == "__main__" :
	test4()
	
	
