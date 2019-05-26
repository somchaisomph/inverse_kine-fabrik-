import numpy as np
import fabrik.vector as vt
from fabrik.utils import VectorUtils
import math

JOINT_DICT={"origin":0,"sub_base":1,'end_effector':2,'normal':3,'target':4,'projection':5}

def law_of_cosine(p1,p2) :
	try:
		delta_x = abs(p1[0] - p2[0])
		delta_y = abs(p1[1] - p2[1])
		magnitude = distance(p1,p2)
		cos_theta = (np.power(delta_y,2) + np.power(magnitude,2) - np.power(delta_x,2)) / (2 * delta_y * magnitude)
		theta = np.degrees(np.arccos(cos_theta))
		return theta
	except ZeroDivisionError :
		raise MyError('Divided by zero.')

def distance(p1,p2):
	d = (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2
	return math.sqrt(d)	
	
class MyError(Exception):
	def __init__(self,value):
		#accept error message from outside.
		self.value = value
		
	def __str__(self):
		# show error message to the outside		.
		return repr(self.value)
#---------------------------------------------------------------
	
		
class IKCHAIN():
	def __init__(self):
		#self.origin = None
		#self.end_effector = None
		self.linkage = []
		self.joints = []		
		self.end_effector_index = -1
		self.origin_index = -1
		
	def add_joint(self,element,type=None):
		if isinstance(element,vt.JOINT):
			self._add_joint(element)
		elif isinstance(element,list) or isinstance(element,tuple) :
			j = vt.JOINT(element,type)
			self.joints.append(j)
			if type == JOINT_DICT['origin']  or  type == JOINT_DICT['sub_base'] :
				if self.origin_index ==-1:
					#self.origin = j
					self.origin_index = self.joints.index(j)
					#print(self.origin_index)
				else :
					raise(MyError("Origin is already set."))		
			if type == JOINT_DICT['end_effector'] :
				if self.end_effector_index == -1 :
					#self.end_effector = j	
					self.end_effector_index = self.joints.index(j)
					#print(self.end_effector_index)
				else :
					raise (MyError('End Effecor is already set.'))
				
	def _add_joint(self,j):
		self.joints.append(j)
		if j.type == JOINT_DICT['origin']  or j.type == JOINT_DICT['sub_base'] :
			self.origin_index = self.joints.index(j)
		if j.type == JOINT_DICT['end_effector'] :
			self.end_effector_index = self.joints.index(j)
			
		
	def size(self):
		#size of chain or number of joints
		return len(self.joints)
		
	def update_linkage(self):
		self.linkage = []
		for i in range(0,len(self.joints)-1,1):
			J1 = self.joints[i]
			J2 = self.joints[i+1]
			L = vt.distance(J1, J2)
			self.linkage.append(L)
	
	def __call__(self,i):
		if i < len(self.joints) and i >=0:
			return self.joints[i]	
		else :
			return None
		
	def set_origin(self,position):
		# fine origin , should be the first element		
		'''
		for i in range(0,len(self.joints),1) :
			if self.joints[i].type ==  JOINT_DICT['origin'] :
				self.joints[i].set(position)
				#self.origin = self.joints[i]
				break
		'''
		self.joints[self.origin_index].set(position)

	def translate(self,tm):		
		#tm is a numpy narray represents translation vector or matrix
		for i in range(0,len(self.joints),1) :
			self.joints[i].translate2d(tm)
			
	def set_origin(self,tm):		
		#tm is a numpy narray represents translation vector or matrix
		for i in range(0,len(self.joints),1) :
			self.joints[i].translate2d(tm)
				
	def get_origin(self):
		return self(self.origin_index )
		
	def get_end_effector(self):
		return self(self.end_effector_index)

	
	def get_total_length(self):
		s = 0
		for l in self.linkage:
			s +=l
		return s
			
	def get_max_length(self):
		mx = self.linkage[0]
		for i in range(1,len(self.linkage),1) :
			if self.linkage[i] > mx :
				mx = self.linkage[i]
		return mx			
		
	def is_good(self):
		if len(self.linkage ) < 2  : return True
		mx_len = self.get_max_length()
		total_len = self.get_total_length()
		if mx_len > (total_len - mx_len) :
			return False
		else :
			return True	
			
	def from_linkage(self,linkage,dim=2,sub_base=False):
		#create from assigned linkage 
		if dim == 2 :
			j = [0,0]
			# add origin or sub base 
			if not sub_base :
				self.add_joint(j,JOINT_DICT['origin'])
			else :
				self.add_joint(j,JOINT_DICT['sub_base'])	
				
			# add normal 	
			for i in range(len(linkage)-1):
				j = (j[0],j[1]+linkage[i])
				self.add_joint(j,JOINT_DICT['normal'])
			j = (j[0],j[1]+linkage[i])
			
			# add end effector
			self.add_joint(j,JOINT_DICT['end_effector'])	
			
		elif dim == 3 :
			j = [0,0,0]
			if not sub_base :
				self.add_joint(j,JOINT_DICT['origin'])
			else :
				self.add_joint(j,JOINT_DICT['sub_base'])	
			for i in range(len(linkage)-1):
				j = [j[0],j[1]+linkage[i],j[2]]
				self.add_joint(j,JOINT_DICT['normal'])
			j = [j[0],j[1]+linkage[i],j[2]]
			self.add_joint(j,JOINT_DICT['end_effector'])			
		self.update_linkage()		
		
	def print_joint(self):
		for i in range(len(self.joints)) :
			print(self(i)())
			
	
	def find_target_projection(self,j1,j2,target):
		proj = VectorUtils.get_projection2d(j1.get(),j2.get(),target.get())
		return proj
				


		
class FBRIK():
	def __init__(self,ik_chain):		
		self.chain = ik_chain
		self.terolance = 0.1
		
	def solve_unreachable(self, target):
		# target is an instance of JOINT
		n = self.chain.size()
		end_eff_ind = self.chain.end_effector_index
		origin_ind = self.chain.origin_index
		#for i in range(0,n-1 ,1)  :
		for i in range(origin_ind,end_eff_ind ,1)  :
			try:
				j1 = self.chain(i)
				ri = vt.distance(target, j1)
				ld = self.chain.linkage[i] / ri
				v1 = j1*(1-ld) 
				v2 = target * ld
				self.chain(i+1).set((v1+v2).get())
				
			except  ZeroDivisionError:
				raise MyError("Divide by zero is not allowed")
			
		return 
		
	def forward(self,target):
		# target is an instance of JOINT
		b = None
		end_eff_ind = self.chain.end_effector_index
		origin_ind = self.chain.origin_index
		
		# target is reachable , set b as the root position		
		#if self.chain(origin_ind).type == JOINT_DICT['origin'] :
		#	b = self.chain(origin_ind)() # keep value of origin position
		if self.chain(origin_ind).type == JOINT_DICT['origin'] :
			b = self.chain(origin_ind).get() # keep value of origin position	
		# start by move the end effector to target
		self.chain(end_eff_ind).set(target.get())
		
		# then the rest of joints downward to root
		try:
			for i in range(end_eff_ind - 1,origin_ind - 1,-1): # include origin_ind 
				# find the distance ri between the new joint position pj[i+1] and joint pi
				j1 = self.chain(i)
				j2 = self.chain(i+1)
				ri = vt.distance(j1,j2)
				ld = self.chain.linkage[i] / ri
				v1 = j2 * (1-ld) 
				v2 = j1 * ld
				self.chain(i).set((v1+v2).get())
		except ZeroDivisionError :
			#raise MyError("Divide by zero is not allowed")
			pass
		
		if b is not None:
			# set it back 
			self.chain(origin_ind).set(b)
		return
		
	def backward(self):
		end_eff_ind = self.chain.end_effector_index
		origin_ind = self.chain.origin_index
		
		#start from origin to end effector -1 
		for i in range(origin_ind, end_eff_ind ,1) :
			try:
				# find distance ri, between new position  pj[i] and pj[i+1]
				j1 = self.chain(i)
				j2 = self.chain(i+1)
				ri = vt.distance(j1,j2)
				ld = self.chain.linkage[i] / ri
				
				# find new position of p[i+1]
				v1 = j1*(1-ld) 
				v2 = j2 * ld
				self.chain(i+1).set((v1+v2).get())
			except ZeroDivisionError :
				raise MyError("Divide by zero is not allowed")	
		
	def solve_reachable(self,target):
		# target is reachable , set b as the root position
		# Check whether the distance between the end effector pn 
		# and the target t is greater than a tolerance. (tol)

		end_eff_ind = self.chain.end_effector_index
		dif_a = vt.distance(target, self.chain(end_eff_ind))
		while dif_a > self.terolance :
			# stage 1 : forward reaching
			self.forward(target)
			# stage 2 : backward reaching
			self.backward()					
			dif_a = vt.distance(target, self.chain(end_eff_ind))		
		return		
		
	def solve(self,target):	
		#find distance from origin to target
		#origin_ind = self.chain.origin_index
		#dist = vt.distance(self.chain(origin_ind),target ) 
		#print(dist)
		#if dist > self.chain.get_total_length():
		#	self.solve_unreachable(target)
		#else :
		#	self.solve_reachable(target)
		#return
		if self.is_reachable(target) :
			#self.solve_reachable(target)
			# stage 1 : forward reaching
			self.forward(target)
			# stage 2 : backward reaching
			self.backward()	
		else:
			self.solve_unreachable(target) 			
	
	def get_resolution(self,type='point'):
		# type could be 'point' or 'angle'
		res=[]
		end_eff_ind = self.chain.end_effector_index
		origin_ind = self.chain.origin_index
		if type =='point':
			for i in range(origin_ind, end_eff_ind+1,1) :
				res.append(self.chain(i).get())
			return res	
		elif type =='angle':
			#for i in range(origin_ind, end_eff_ind-1,1) :
			for i in range(origin_ind, end_eff_ind,1) :
				j1 = self.chain(i).get()
				j2 = self.chain(i+1).get()
				ang = law_of_cosine(j1,j2)
				res.append(ang)
			return res		
		
	def is_reachable(self,target):
		origin_ind = self.chain.origin_index
		dist = VectorUtils.distance(self.chain(origin_ind).get(),target.get() ) 
		return dist < self.chain.get_total_length()
			
#-----------------------------------------------------------------------
