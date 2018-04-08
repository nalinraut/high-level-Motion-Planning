
import pkg_resources
pkg_resources.require("Klampt>=0.7.0")
import stability 
import polytope
import hand_models 
import moving_base
import math
from klampt.math import *
from klampt import RobotPoser,ObjectPoser,PointPoser,TransformPoser
from klampt import vis
from klampt.vis import *
from klampt.vis import glcommon
from klampt.model import coordinates
from klampt.model.trajectory import SE3Trajectory
from klampt.io import resource
from klampt.vis import editors
import sys
import numpy as np
import scipy.spatial
import itertools

object_file = "objects/curve1.obj"
#object_file = "objects/curve2.obj"


def mkdir_p(path):
    import os, errno
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise


#torque balance
#with Wi the wrench matrix of the i'th contact on the object
#Ji the jacobian of the i'th contact on the robot
#G the generalized gravity force on the robot
#Gobj the net wrench of gravity on the object
#
#find t,f1,..,fn s.t. 
#  G = t - sum[i=1 to n] Ji(q)^T fi
#  sum[i=1 to n] Wi fi = Gobj
#  tmin <= t <= tmax
#  fi in FCi
#
# set up constraints on F:
# tmin <= G + sum[i=1 to n] Ji(q)^T fi <= tmax
# if tmax = -tmin = inf, then can ignore.
# if tmin = tmax, then add equality constraint.
#
# In the case of spring-loaded hand, let qref be the reference
# configuration and qact be the actual configuration.  Let K be the
# elasticity matrix.  Then t = K(qref - qact) is known and we have case 2.

def refine_hull(sphere):
	def tri_normal(a,b,c):
		n = vectorops.cross(vectorops.sub(b,a),vectorops.sub(c,a))
		d = vectorops.norm(n)
		if d > 0:
			return vectorops.div(n,d)
		return n
	#now see if there are any edge flips that can make the sphere more convex
	edgeTriLeft = dict()
	edgeTriRight = dict()
	for i,s in enumerate(sphere.simplices):
		#orient to be outward facing
		normal = tri_normal(sphere.points[s[0]],sphere.points[s[1]],sphere.points[s[2]])
		if vectorops.dot(sphere.points[s[0]],normal) < 0:
			s[1],s[2] = s[2],s[1]
		edgeTriLeft[s[0],s[1]] = i
		edgeTriLeft[s[1],s[2]] = i
		edgeTriLeft[s[2],s[0]] = i
		edgeTriRight[s[1],s[0]] = i
		edgeTriRight[s[2],s[1]] = i
		edgeTriRight[s[0],s[2]] = i
	def flipscore(i,j):
		if (i,j) not in edgeTriLeft:
			return -1
		if (i,j) not in edgeTriRight:
			return -1
		#find the two triangles with i and j
		it1,it2 = edgeTriLeft[(i,j)],edgeTriRight[(i,j)]
		t1 = [v for v in sphere.simplices[it1]]
		t2 = [v for v in sphere.simplices[it2]]
		m,n = -1,-1
		m = [v for v in t1 if v not in [i,j]][0]
		n = [v for v in t2 if v not in [i,j]][0]
		assert (t1.index(j)+1)%3 == t1.index(m),"incorrect edge ? "+str(t1)+" should be "+str([i,j,m])
		assert (t2.index(i)+1)%3 == t2.index(n),"incorrect edge ? "+str(t2)+" should be "+str([j,i,n])
		if m == n:
			#flipped to inversion?
			return -1
		assert m != i
		assert m != j
		assert n != i
		assert n != j
		#consider edge swap to mn
		midpt = vectorops.interpolate(sphere.points[i],sphere.points[j],0.5)
		altmidpt = vectorops.interpolate(sphere.points[m],sphere.points[n],0.5)
		n1 = tri_normal(sphere.points[i],sphere.points[j],sphere.points[m])
		n2 = tri_normal(sphere.points[i],sphere.points[n],sphere.points[j])
		n1alt = tri_normal(sphere.points[i],sphere.points[n],sphere.points[m])
		n2alt = tri_normal(sphere.points[m],sphere.points[n],sphere.points[j])
		angle = vectorops.dot(n1,n2)
		altangle = vectorops.dot(n1alt,n2alt)
		#use distance to origin as a proxy for convexity
		#score = vectorops.norm(midpt)
		#altscore = vectorops.norm(altmidpt)
		#use convexity as score
		score = -(vectorops.dot(n1,vectorops.sub(sphere.points[n],sphere.points[m])) + vectorops.dot(n2,vectorops.sub(sphere.points[m],sphere.points[n])))
		altscore = -(vectorops.dot(n1alt,vectorops.sub(sphere.points[j],sphere.points[i])) + vectorops.dot(n2alt,vectorops.sub(sphere.points[i],sphere.points[j])))
		#if vectorops.dot(n1alt,sphere.points[i]) <= 0 or vectorops.dot(n2alt,sphere.points[i]) <= 0:
		#	return -1
		return altscore - score 
	flipscores = dict()
	for e in edgeTriLeft.iterkeys():
		(i,j) = e
		if i > j:
			continue
		assert i != j
		score = flipscore(i,j)
		#print "score for edge",i,j,":",score
		if score > 0:
			flipscores[(i,j)] = score
	while len(flipscores) > 0:
		(score,edge) = max((v,k) for (k,v) in flipscores.iteritems())
		if score <= 1:
			break
		del flipscores[edge]
		i,j = edge
		it1,it2 = edgeTriLeft[(i,j)],edgeTriRight[(i,j)]
		t1 = [v for v in sphere.simplices[it1]]
		t2 = [v for v in sphere.simplices[it2]]
		m,n = -1,-1
		m = [v for v in t1 if v not in [i,j]][0]
		n = [v for v in t2 if v not in [i,j]][0]
		#print "Flipping edge",i,j,m,n
		#print "Triangles",it1,it2
		#print "Quality score",score
		assert (t1.index(j)+1)%3 == t1.index(m)
		assert (t2.index(i)+1)%3 == t2.index(n)
		if m == n:
			"refine_hull(): Uh... both sides of a triangle are in the polyhedron"
			continue
		assert m != n
		assert m != i
		assert m != j
		assert n != i
		assert n != j
		del edgeTriLeft[(i,j)]
		del edgeTriLeft[(j,i)]
		del edgeTriRight[(i,j)]
		del edgeTriRight[(j,i)]
		sphere.simplices[it1] = [i,n,m]
		sphere.simplices[it2] = [m,n,j]
		edgeTriLeft[(i,n)] = it1
		edgeTriRight[(n,i)] = it1
		edgeTriLeft[(n,m)] = it1
		edgeTriRight[(m,n)] = it1
		edgeTriLeft[(m,i)] = it1
		edgeTriRight[(i,m)] = it1
		edgeTriLeft[(m,n)] = it2
		edgeTriRight[(n,m)] = it2
		edgeTriLeft[(n,j)] = it2
		edgeTriRight[(j,n)] = it2
		edgeTriLeft[(j,m)] = it2
		edgeTriRight[(m,j)] = it2
		#need to update flip scores for all adjacent edges
		for e in zip(t1,t1[1:3]+[t1[0]]) + zip(t2,t2[1:3]+[t2[0]]):
			a,b = e
			if a > b:
				a,b = b,a
			if (a,b) != (i,j):
				if (a,b) in edgeTriRight and (a,b) in edgeTriLeft:
					#print "Revising",a,b
					#print "Triangles",sphere.simplices[edgeTriLeft[(a,b)]],sphere.simplices[edgeTriRight[(a,b)]]
					score = flipscore(a,b)
					if score > 0:
						flipscores[(a,b)] = score


class GLHandStabilityPlugin(GLPluginInterface):
	def __init__(self,h):
		GLPluginInterface.__init__(self)
		self.hand = h
		h.load_frames()
		self.world = WorldModel()
		#load robot
		robot = moving_base.make_moving_base_robot(h.robot_file,self.world,floating=True)
		#load object
		id = self.world.loadElement(object_file)
		if id < 0:
			raise IOError("Unable to load "+object_file)
		assert self.world.numRigidObjects() > 0
		object = self.world.rigidObject(0)
		object.setTransform(so3.rotation([1,0,0],math.pi/2),[0,0,0.2])

		#set up unit frames
		coordinates.setWorldModel(self.world)
		coordinates.listItems()
		hand_coordinates = coordinates.manager().subgroups[robot.getName()]
		hand_coordinates.listItems()
		self.unit_frames = []
		for i,(key,unit) in enumerate(h.all_units):
			self.unit_frames.append(hand_coordinates.addFrame("unit"+str(i),parent=hand_coordinates.frame(key),relativeCoordinates=unit.localTransform))

		#set up  visualization
		vis.add("robot",robot)
		vis.edit("robot")
		vis.add("rigidObject",object)
		vis.edit("rigidObject")
		print "COM",object.getMass().getCom()
		vis.add("COM",se3.apply(object.getTransform(),object.getMass().getCom()))
		vis.add("origin",se3.identity())
		#vis.add("coordinates",coordinates.manager())

		self.config_name = None

		#set up friction solver
		folder="FrictionCones/"
		self.friction_factories = {}
		for key,unit in h.all_units:
			if unit.type not in self.friction_factories:
				self.friction_factories[unit.type] = stability.GeneralizedFrictionConeFactory()
				self.friction_factories[unit.type].load(folder + unit.type)
		self.contact_units = []
		self.wrenches = None
		self.torques = None
		self.feasible = None
		self.angle = 0
		self.wrenchSpace = None
		self.wrenchSpace6D = None

		self.frame_gl_objects = []
		self.wrench_space_gl_object = None

	def display(self):
		robot = self.world.robot(0)
		object = self.world.rigidObject(0)
		GLPluginInterface.display(self)

		#draw unit friction cones
		glEnable(GL_LIGHTING)
		glDisable(GL_DEPTH_TEST)
		while len(self.frame_gl_objects) < len(self.unit_frames):
			self.frame_gl_objects.append(glcommon.CachedGLObject())
		for i in self.contact_units:
			f = self.unit_frames[i]
			obj = self.frame_gl_objects[i]
			key,unit = self.hand.all_units[i]
			T = f.worldCoordinates()
			try:
				obj.draw(self.friction_factories[unit.type].forceTemplate.drawGL,T,args=(unit.patchSize/25,))
			except:
				import traceback
				traceback.print_exc()
				exit(-1)
		glEnable(GL_DEPTH_TEST)
		#draw wrenches
		if self.wrenches is not None:
			#this line shows the force inside the friction cone
			#forcescale = self.contactPatchSize/25/4
			forcescale = object.getMass().mass/9.8
			torquescale = forcescale
			glDisable(GL_LIGHTING)
			glDisable(GL_DEPTH_TEST)
			glBegin(GL_LINES)
			for w,frame in zip(self.wrenches,self.last_solver_frames):
				normal_amt = se3.apply(se3.inv(frame.worldCoordinates()),w[:3])[2]
				if normal_amt > 0.0:
					glColor3f(1,0.5,0)
				else:
					glColor3f(1,0.5,1)
				p = frame.worldCoordinates()[1]
				glVertex3fv(p)
				glVertex3fv(vectorops.madd(p,(w[0],w[1],w[2]),forcescale))
			glEnd()
			glBegin(GL_LINES)
			for w,frame in zip(self.wrenches,self.last_solver_frames):
				normal_amt = se3.apply(se3.inv(frame.worldCoordinates()),w[:3])[2]
				if normal_amt > 0.0:
					glColor3f(0,0.5,0)
				else:
					glColor3f(0,0.5,1)
				p = frame.worldCoordinates()[1]
				f = (w[0],w[1],w[2])
				m = (w[3],w[4],w[5])
				m = vectorops.sub(m,vectorops.cross(p,f))
				glVertex3fv(p)
				glVertex3fv(vectorops.madd(p,m,torquescale))
			glEnd()
			glEnable(GL_DEPTH_TEST)
		if self.torques is not None:
			#TODO: draw torques
			glDisable(GL_DEPTH_TEST)
			for i in xrange(6,robot.numLinks()):
				T = robot.link(i).getTransform()
				alocal = robot.link(i).getAxis()
				aworld = so3.apply(T[0],alocal)
				torque = self.torques[i]
			glEnable(GL_DEPTH_TEST)
		if self.wrenchSpace is not None:
			if self.wrench_space_gl_object == None:
				self.wrench_space_gl_object = glcommon.CachedGLObject()
			cm = se3.apply(self.world.rigidObject(0).getTransform(),self.world.rigidObject(0).getMass().getCom())
			if hasattr(self,'volumeTexture'):
				glBindTexture(GL_TEXTURE_3D,self.volumeTexture[0])
			else:
				self.volumeTexture = glGenTextures(2)

				m,n,p = 1,512,1
				data = [0]*(m*n*p*4)

				glBindTexture(GL_TEXTURE_3D,self.volumeTexture[1])
				for i in range(n):
					data[i*4] = max(0,255-i)
					data[i*4+1] = 255-i if i >= 256 else i
					data[i*4+2] = max(i-255,0)
					data[i*4+3] = 255
				glTexImage3D(GL_TEXTURE_3D,0,GL_RGBA8,n,m,p,0,GL_RGBA,GL_UNSIGNED_BYTE,data)
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_MIN_FILTER,GL_LINEAR)
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_S,GL_CLAMP);
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_T,GL_CLAMP);
				#glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_U,GL_CLAMP);

				glBindTexture(GL_TEXTURE_3D,self.volumeTexture[0])
				data = [0]*(m*n*p*4)
				for i in range(n):
					data[i*4+0] = 0
					data[i*4+1] = 0
					data[i*4+2] = 0
					data[i*4+3] = 0
					if (i+50) % 100 == 0:
						data[i*4+0] = 0
						data[i*4+1] = 0
						data[i*4+2] = 0
						data[i*4+3] = 255 * (512-i/2) / 512
				glTexImage3D(GL_TEXTURE_3D,0,GL_RGBA8,m,n,p,0,GL_RGBA,GL_UNSIGNED_BYTE,data)
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_MIN_FILTER,GL_LINEAR)
				#glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_S,GL_REPEAT)
				#glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_T,GL_REPEAT)
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_S,GL_CLAMP);
				glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_T,GL_CLAMP);
				#glTexParameteri(GL_TEXTURE_3D,GL_TEXTURE_WRAP_U,GL_CLAMP);

			glEnable(GL_TEXTURE_3D)
			glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE)
			glDisable(GL_CULL_FACE)
			glDisable(GL_DEPTH_TEST)
			#glEnable(GL_LIGHTING)
			glDisable(GL_LIGHTING)
			glEnable(GL_BLEND)
			glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
			#draw lines
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(1,1,1,1))
			self.wrench_space_gl_object.draw(stability.draw_hull,transform=(so3.identity(),cm),args=(self.wrenchSpace,0.01))
			glEnable(GL_CULL_FACE)
			glEnable(GL_DEPTH_TEST)
			#draw volume
			glEnable(GL_LIGHTING)
			glBindTexture(GL_TEXTURE_3D,self.volumeTexture[1])
			#glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(1,0.5,0,0.8))
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(1,1,1,0.8))
			self.wrench_space_gl_object.draw(stability.draw_hull,transform=(so3.identity(),cm),args=(self.wrenchSpace,0.01))
			glDisable(GL_BLEND)
			glDisable(GL_TEXTURE_3D)

	def motionfunc(self,x,y,dx,dy):
		res = GLPluginInterface.motionfunc(self,x,y,dx,dy)
		coordinates.updateFromWorld()
		object = self.world.rigidObject(0)
		vis.setItemConfig("COM",se3.apply(object.getTransform(),object.getMass().getCom()))
		return res

	def edit_contacting_links(self):
		contact_links = set()
		for i,(name,unit) in enumerate(self.contact_units):
			contact_links.add(name)
		ok_pressed,links = editors.run(editors.SelectionEditor("Contacting links",value=list(contact_links),description="Select all links in contact",world=self.world,robot=self.world.robot(0)))
		if not ok_pressed:
			return
		#if self.config_name != None:
		#	print "Saving to",self.config_name+".array"
		#	resource.set(self.config_name+".array",type="IntArray",value=links)
		robot = self.world.robot(0)
		linkNames = set([robot.link(l).getName() for l in links])
		self.contact_units = []
		for i,(name,unit) in enumerate(self.hand.all_units):
			if name in linkNames:
				self.contact_units.append(i)
		return self.contact_units

	def edit_contacting_units(self):
		tempWorld = WorldModel(self.world)
		patches = {}
		selected = []
		for i,(link,unit) in enumerate(self.hand.all_units):
			obj = tempWorld.makeRigidObject(unit.name)
			patch = GeometricPrimitive()
			patch.setAABB([-unit.patchSize/2,-unit.patchSize/2,-0.001],[unit.patchSize/2,unit.patchSize/2,0.001])
			obj.geometry().set(Geometry3D(patch))
			obj.appearance().setColor(0,0,0,0.5)
			obj.setTransform(*self.unit_frames[i].worldCoordinates())
			patches[obj.getID()] = i
			if i in self.contact_units:
				selected.append(obj.getID())
		ok_pressed,objects = editors.run(editors.SelectionEditor("Contacting units",value=selected,description="Select all units in contact",world=tempWorld))
		if not ok_pressed:
			return
		self.contact_units = []
		for oid in objects:
			if oid in patches:
				self.contact_units.append(patches[oid])
		for u in self.contact_units:
			assert u >= 0 and u < len(self.unit_frames),"Invalid unit? "+str(u)
		return self.contact_units


	def make_solver(self,units,origin,useTorqueLimits=False,gravity=(0,0,-9.8)):
		"""Makes a GeneralizedStabilitySolver according to the robot's current configuration, without any
		external forcing specified.

		Return value: (solver,J) where solver is a GeneralizedStabilitySolver instance and J are the
		jacobian matrices for each contact point.

		If useTorqueLimits = True, adds torque constraints corresponding to the current configuration
		and given gravity.
		"""
		self.last_solver_frames = []

		robot = self.world.robot(0)
		solver = stability.GeneralizedStabilitySolver(self.friction_factories)
		unitlinks = []
		for i in units:
			assert i >= 0 and i < len(self.hand.all_units)
			link,unit = self.hand.all_units[i]
			frame = self.unit_frames[i]
			#print "Adding unit on link",link
			#print link,frame.worldCoordinates()
			self.last_solver_frames.append(frame)
			T = frame.worldCoordinates()
			solver.addContactPatch((T[0],vectorops.sub(T[1],origin)),unit.patchSize,unit.type)
			unitlinks.append(link)

		J = []
		for i in xrange(len(solver.contactPoints)):
			patch = solver.pointIndices[i]
			ci = vectorops.add(solver.contactPoints[i],origin)
			link = robot.link(unitlinks[patch])
			assert link.index >= 0
			ciloc = link.getLocalPosition(ci)
			Ji = link.getPositionJacobian(ciloc)
			#print Ji
			#raw_input()
			J.append(np.array(Ji))

		if useTorqueLimits:
			#Torque balance constraints
			tmax = robot.getTorqueLimits()
			tmin = [-v for v in tmax]
			q = robot.getConfig()
			qmin,qmax = robot.getJointLimits()
			#assume infinite bounds on first 6 links
			inf = float('inf')
			for i in xrange(6):
				tmin[i] = -inf
				tmax[i] = inf
			for i in xrange(robot.numLinks()):
				if q[i] == qmin[i]:
					#print "Joint",robot.link(i).getName(),"at minimum, assuming infinite positive torque limit"
					tmax[i] = inf
				if q[i] == qmax[i]:
					#print "Joint",robot.link(i).getName(),"at maximum, assuming infinite negative torque limit"
					tmin[i] = -inf
			#torque balance testing
			#t = G - sum Ji^T (-fi), tmin <= t <= tmax
			G = np.array(robot.getGravityForces(gravity))
			for i in xrange(robot.numLinks()):
				if tmin[i] > -inf or tmax[i] < inf:
					row = np.hstack([Ji.T[i,:] for Ji in J])
					rhs = G[i]
					solver.addLinearConstraint(row,tmin[i]-rhs,tmax[i]-rhs)
		return (solver,J)

	def solve(self,units,mass=None,wext=None,mode='equilibrium'):
		"""mode can be 'equilibrium', 'energy_minimization', 'wrench_maximization', or 'wrench_dot_maximization'
		"""
		if not units:
			print "No contact units specified?"
			return False

		#create and configure the solver
		t0 = time.time()
		robot = self.world.robot(0)
		if mass == None:
			mass = self.world.rigidObject(0).getMass().mass
		#center the origin at the object's COM
		cm = se3.apply(self.world.rigidObject(0).getTransform(),self.world.rigidObject(0).getMass().getCom())
		#print "Cm",cm
		gravity = (0,0,-9.8)
		origin = cm
		if mode in ['wrench_maximization','wrench_dot_maximization']  or wext != None:
			gravity = (0.0,0.0,0.0)
		(solver,J) = self.make_solver(units,origin,useTorqueLimits=True,gravity=gravity)

		#set up the external forcing
		if wext is not None:
			solver.setExternalWrench(wext)
		else:
			solver.setGravityWrench(mass,[0,0,0],gravity)
		if mode == 'energy_minimization':
			#TODO: mass and inertia matrix
			solver.setWrenchObjective(1,1)
		elif mode == 'wrench_maximization':
			solver.setExternalWrench(wext)
			solver.maximizeWrenchMagnitude = True
		elif mode == 'wrench_dot_maximization':
			solver.setExternalWrench(wext)
			solver.maximizeWrenchDot = True
		else:
			#TESTING
			#solver.maximizeWrenchMagnitude = True
			pass

		t1 = time.time()
		print "Problem setup time",t1-t0
		t0 = time.time()
		self.torques = None
		#perform the solve
		try:
			self.wrenches = solver.solve()
		except ValueError as e:
			print e
			raise
			print "Probably don't have a valid static equilibrium solution"
			print "Equality constraints:"
			print solver.A_eq_extra
			self.wrenches = None
		t1 = time.time()
		self.last_solver = solver
		self.feasible = (self.wrenches is not None)
		print
		print "Solve time",t1-t0,"Feasible:",self.feasible
		if self.feasible:
			assert len(solver.forces) == len(J)
			#assert len(self.wrenches) == len(units)
			self.torques = np.array(robot.getGravityForces(gravity)) + sum(np.dot(Ji.T,fi) for fi,Ji in zip(solver.forces,J))
			return True
			print "Contact patch wrenches"
			for w in self.wrenches:
				print "   ",w
			#print "Contact patch forces",solver.forces
			#print solver.wrenchMatrices
			#print "Gravity torques",G
			w = sum(self.wrenches)
			if mode not in ['wrench_maximization','wrench_dot_maximization']:
				w += solver.wext
			#print "Resultant joint torques:"
			#for i in range(6,robot.numLinks()):
			#	print " ",robot.link(i).getName(),":",self.torques[i]
			print "Resultant body wrench:",w
			f = w[0:3]
			t = w[3:6]
			if mode == 'energy_minimization':
				#TODO: look at the object's inertia matrix?
				Tobj = self.world.rigidObject(0).getTransform()
				v,w = vectorops.div(f,mass),vectorops.div(t,mass)
				dR = vectorops.mul(w,1.0/mass)
				dt = vectorops.mul(v,1.0/mass)
				Tnew = se3.mul((so3.from_moment(dR),dt),Tobj)
				traj = SE3Trajectory([0,1],[Tobj,Tnew])
				vis.animate("rigidObject",traj)
			#raw_input()
			if solver.minimize:
				return solver.objectiveValue
				self.feasible = (solver.objectiveValue < 1e-5)
				return self.feasible
			return True
		else:
			#try again, without torque bounds
			solver.A_ineq_extra = []
			solver.b_ineq_extra = []
			self.wrenches = solver.solve()
			if self.wrenches is not None:
				print "Limiting factor is joint torques"
				self.torques = G + sum(np.dot(Ji.T,fi) for fi,Ji in zip(solver.forces,J))
				print "Resultant joint torques:",self.torques[6:]
		return False

	def rotate(self,radians):
		robot = self.world.robot(0)
		object = self.world.rigidObject(0)
		robot.setConfig(vis.getItemConfig("robot"))
		T = vis.getItemConfig("rigidObject")
		object.setTransform(T[:9],T[9:])

		self.angle += radians
		T = (so3.rotation((0,1,0),radians),[0.0]*3)
		Tbase = moving_base.get_moving_base_xform(robot)
		moving_base.set_moving_base_xform(robot,*se3.mul(T,Tbase))
		Tobj = object.getTransform()
		object.setTransform(*se3.mul(T,Tobj))
		#update visualization
		vis.setItemConfig("robot",robot.getConfig())
		vis.setItemConfig("rigidObject",sum(object.getTransform(),[]))
		vis.setItemConfig("COM",se3.apply(object.getTransform(),object.getMass().getCom()))
		coordinates.updateFromWorld()
		self.refresh()

	def keyboardfunc(self,c,x,y):
		if c == 'h':
			print "Help: "
			print "- s: save grasp configuration to disk"
			print "- l: load grasp configuration from disk"
			print "- <: rotate left"
			print "- >: rotate right"
			print "- [space]: solve for stability"
			print "- f: solve for the feasible force volume"
			print "- w: solve for the feasible wrench space"
			print "- f: solve for the robust force volume"
			print "- m: save the current volume to a mesh"
			print "- M: load the current volume from a mesh"
			print "- 6: calculates the 6D wrench space"
			print "- 7: loads the 6D wrench space"
			print "- 0: zeros the robot's base transform"
			print "- 9: hides the widgets"
		if c == 's':
			name = raw_input("Enter a name for this configuration > ")
			if name.strip() == '':
				print "Not saving..."
			else:
				self.save_config(name)
				self.config_name = name
		elif c == 'l':
			vis.animate("rigidObject",None)
			name = raw_input("Enter a name for the configuration to load > ")
			if name.strip() == '':
				print "Not loading..."
			else:
				self.load_config(name)
		elif c == ',' or c == '<':
			vis.animate("rigidObject",None)
			self.rotate(math.radians(1))
			print "Now at angle",math.degrees(self.angle)
		elif c == '.' or c == '>':
			vis.animate("rigidObject",None)
			self.rotate(-math.radians(1))
			print "Now at angle",math.degrees(self.angle)
		elif c == ' ':
			self.solve(self.contact_units)
		elif c == 'f':
			if self.wrenchSpace6D != None:
				cm = se3.apply(self.world.rigidObject(0).getTransform(),self.world.rigidObject(0).getMass().getCom())
				self.wrenchSpace = self.wrenchSpace6D.getSlice(cm).convex_decomposition[0]
				self.wrench_space_gl_object = None
			else:
				self.wrenchSpace = self.calculate_force_volume((0,0,0))
				self.wrenchSpace6D = None
				self.wrench_space_gl_object = None
		elif c == 'w':
			self.wrenchSpace = self.calculate_3D_wrench_space()
			self.wrenchSpace6D = None
			self.wrench_space_gl_object = None
		elif c == 'r':
			dropout = raw_input("How much to drop out? (default 0) > ")
			if dropout.strip() == '':
				dropout = 0
			else:
				dropout = float(dropout.strip())
			perturb = raw_input("How much to perturb? (default 0.1) > ")
			if perturb.strip() == '':
				perturb = 0.1
			else:
				perturb = float(perturb.strip())
			N = raw_input("How many samples? (default 100) > ")
			if N.strip() == '':
				N = 100
			else:
				N = int(N.strip())
			self.wrenchSpace = self.calculate_robust_force_volume((0,0,0),dropout=dropout,configurationPerturbation=perturb,maxVolumes=N)
			self.wrenchSpace6D = None
			self.wrench_space_gl_object = None
		elif c == 'm':
			assert self.wrenchSpace != None
			from PyQt4.QtGui import QFileDialog
			savedir = "data/"+self.world.robot(0).getName()+'/'+self.config_name
			name = QFileDialog.getSaveFileName(caption="Mesh file (*.obj, *.off, etc)",directory=savedir,filter="Wavefront OBJ (*.obj);;Object File Format (*.off);;All files (*.*)")
			g = polytope.hull_to_klampt_geom(self.wrenchSpace)
			g.saveFile(str(name))
		elif c == 'M':
			from PyQt4.QtGui import QFileDialog
			savedir = "data/"+self.world.robot(0).getName()+'/'+self.config_name
			name = QFileDialog.getOpenFileName(caption="Mesh file (*.obj, *.off, etc)",directory=savedir,filter="Wavefront OBJ (*.obj);;Object File Format (*.off);;All files (*.*)")
			g = Geometry3D()
			g.loadFile(str(name))
			self.wrenchSpace = polytope.klampt_geom_to_hull(g)
			self.wrench_space_gl_object = None
			#decomp = stability.approximate_convex_decomposition(g)
			#print "Convex decomposition into",len(decomp),"pieces"
			print len(polytope.pockets(g)),"pocket faces"
		elif c == '6':
			N = raw_input("How many samples? (default 100) > ")
			if N.strip() == '':
				N = 100
			else:
				N = int(N.strip())
			self.wrenchSpace6D = self.calculate_6D_wrench_space(N)
			self.wrenchSpace = None
			self.wrench_space_gl_object = None
		elif c == '7':
			self.wrenchSpace6D = self.load_6D_wrench_space()
			self.wrenchSpace = None
			self.wrench_space_gl_object = None
		elif c == '0':
			robot = self.world.robot(0)
			q = robot.getConfig()
			q[0:6] = [0]*6
			robot.setConfig(q)
			vis.setItemConfig("robot",robot.getConfig())
			coordinates.updateFromWorld()
		elif c == '9':
			vis.edit("robot",False)
			vis.edit("rigidObject",False)
			#vis.hide("COM")
			#object = self.world.rigidObject(0)
			#vis.edit("COM")
		else:
			return False
		return True

	def load_config(self,name):
		object = self.world.rigidObject(0)
		self.config_name = name
		print "Loading from",name+".config,",name+".xform","and",name+".array"
		qrob = resource.get(name+".config",type="Config",doedit=False)
		Tobj = resource.get(name+".xform",type="RigidTransform",doedit=False)
		self.contact_units = resource.get(name+"_units.array",type="IntArray",default=[],doedit=False)
		if len(self.contact_units) == 0:
			print "COULDNT LOAD "+name+"_units.array, trying "+name+".array"
			contact_links = resource.get(name+".array",type="IntArray",default=[],doedit=False)
			if len(contact_links) > 0:
				robot = self.world.robot(0)
				contact_links = [robot.link(l).getName() for l in contact_links]
				self.contact_units = []
				for i,(link,unit) in enumerate(self.hand.all_units):
					if link in contact_links:
						self.contact_units.append(i)
				print "UNITS",self.contact_units
		object.setTransform(*Tobj)
		qobj = vis.getItemConfig("rigidObject")
		vis.setItemConfig("robot",qrob)
		vis.setItemConfig("rigidObject",qobj)
		vis.setItemConfig("COM",se3.apply(object.getTransform(),object.getMass().getCom()))
		coordinates.updateFromWorld()

	def save_config(self,name):
		self.config_name = name
		print "Saving to",name+".config,",name+".xform","and",name+".array"
		resource.set(name+".config",type="Config",value=vis.getItemConfig("robot"))
		resource.set(name+".xform",type="RigidTransform",value=self.world.rigidObject(0).getTransform())
		resource.set(name+"_units.array",type="IntArray",value=self.contact_units)

	def calculate_robust_force_volume(self,p,w=20,h=10,dropout=1,configurationPerturbation=0,maxVolumes=100):
		"""Calculates the set of force volumes robust to unit drop-outs.and configuration perturbations.

		If configurationPerturbation = 0, then O(n choose dropout) volumes will be computed.
		Otherwise, maxVolumes volumes will be computed
		"""
		volumes = []
		if configurationPerturbation == 0:
			for units in itertools.combinations(self.contact_units,len(self.contact_units)-dropout):
				v = self.calculate_force_volume(p,w,h,units)
				volumes.append(v)
		else:
			from math import factorial
			from operator import mul
			import random
			robot = self.world.robot(0)
			def n_choose_k(n, k):
				if k < n-k:
					return reduce(mul, xrange(n-k+1, n+1), 1) // factorial(k)
				else:
					return reduce(mul, xrange(k+1, n+1), 1) // factorial(n-k)
			numCombinations = n_choose_k(len(self.contact_units),dropout)
			#TODO: sample combinations when (n choose k) is greater than maxVolumes
			combs = [links for links in itertools.combinations(self.contact_units,len(self.contact_units)-dropout)]
			q0 = robot.getConfig()
			for sample in xrange(maxVolumes):
				qpert = [0]*6 + [random.uniform(-configurationPerturbation,configurationPerturbation) for i in xrange(6,robot.numLinks())]
				robot.setConfig(vectorops.add(q0,qpert))
				v = self.calculate_force_volume(p,w,h,random.choice(combs))
				volumes.append(v)

		#take the minimum in each direction
		for i in range(volumes[0].points.shape[0]):
			dmin = np.linalg.norm(volumes[0].points[i])
			for v in volumes[1:]:
				assert v.points.shape == volumes[0].points.shape
				d = np.linalg.norm(v.points[i])
				if d < dmin:
					volumes[0].points[i] = v.points[i]
					dmin = d
		refine_hull(volumes[0])
		return volumes[0]

	def calculate_force_volume(self,p,w=20,h=10,units=None):
		"""Calculates the force volume at the point p. The sphere is discretized with
		w divisions in the latitude direction and h in the longitude direction.
		Return value is a Geometry3D.

		If units is not provided, self.contact_units is used as the set of contacting units.
		"""
		if units is None:
			units = self.contact_units
		spheremat = []
		for i in range(h+1):
			phi = float(i)/float(h)*math.pi
			z = math.cos(phi)
			xy = math.sin(phi)
			nw = int(math.floor(xy*w))+1
			#nw = w
			for j in range(nw):
				theta = float(j)/float(nw)*math.pi*2
				x = xy * math.cos(theta)
				y = xy * math.sin(theta)
				spheremat.append((x,z,y))
		sphere = scipy.spatial.ConvexHull(spheremat)

		solver_stats = dict()
		solver_stats['time'] = []
		solver_stats['num_lp_solves'] = []
		solver_stats['feasible'] = []
		for i in range(len(sphere.points)):
			fext = sphere.points[i]
			mext = np.cross(p,fext)
			wext = np.hstack((fext,mext))
			t0 = time.time()
			try:
				res = self.solve(units,wext=wext,mode='wrench_maximization')
			except KeyboardInterrupt:
				q = raw_input("Do you want to quit? (y/n)> ")
				if q == 'y':
					break
				else:
					res = False
					self.wrenches = None
			t1 = time.time()
			solver_stats['time'].append(t1-t0)
			solver_stats['num_lp_solves'].append(self.last_solver.numConvexSolves)
			if self.last_solver.numConvexSolves > 1000:
				raw_input("Excessive number of convex solves... press enter to continue")

			if self.wrenches == None:
				w = np.zeros((6,))
				solver_stats['feasible'].append(0)
			else:
				w = -sum(self.wrenches,np.zeros(6,))
				solver_stats['feasible'].append(1)

			print "****** Direction",wext,"value",w,"amount",np.dot(wext,w),"******"
			row = [v for v in wext] + [np.dot(wext,w)]
			if self.wrenches is not None:
				for wi in self.wrenches:
					row += [v for v in wi]
				if self.torques is not None:
					row += [v for v in self.torques]
			assert np.dot(wext,w) >= -1e-5
			#if np.linalg.norm(w) > 0:
			#	print "  Cos angle",np.dot(wext,w)/(np.linalg.norm(w)*np.linalg.norm(wext))
			f = w[0:3]
			t = w[3:6]
			sphere.points[i] = [f[0],f[1],f[2]]

		print "STATS"
		for s,values in solver_stats.iteritems():
			if s == 'time':
				print "%s: %dms [%dms,%dms]"%(s,int(float(sum(values))/len(values)*1000),int(min(values)*1000),int(max(values)*1000))
			else:
				print "%s: %f [%f,%f]"%(s,float(sum(values))/len(values),min(values),max(values))
			
		ch = scipy.spatial.ConvexHull(sphere.points)
		if len(ch.vertices) == sphere.points.shape[0]:
			return ch
		#otherwise begin a refining process to make this polyhedron look a bit better
		refine_hull(sphere)
		return sphere

	def calculate_6D_wrench_space(self,numPoints):
		solver,J = self.make_solver(self.contact_units,[0,0,0])
		pthull = scipy.spatial.ConvexHull(solver.contactPoints)
		if numPoints < len(pthull.vertices):
			print "Requested number of points lower than # of convex hull vertices",numPoints,"vs",len(pthull.vertices)
			raw_input("Press enter to continue > ")
		pts = []
		volumes = []

		savedir = "data/"+self.world.robot(0).getName()+'/'+self.config_name
		out = open("%s/wrench_slices.csv"%(savedir,),'w')
		out.write("#index,x,y,z\n")

		centroid = np.dot(pthull.points[pthull.vertices[i],:],np.ones(3))/len(pthull.vertices)
		numPoints = max(0,numPoints-len(pthull.vertices))
		numNonConvex = 0
		for i in range(len(pthull.vertices)):
			pt = pthull.points[pthull.vertices[i],:]
			#pull the point toward the centroid
			pt += (centroid-pt)/np.linalg.norm(centroid-pt)*1e-3
			pt = pt.tolist()
			geom = polytope.hull_to_klampt_geom(self.calculate_force_volume(pt))
			V = polytope.Volume()
			try:
				V.setGeom(geom)
				if len(V.convex_decomposition) > 1:
					print "Encountered a non-convex force volume on CH point",i
					numNonConvex += 1
				out.write("%d,%f,%f,%f\n"%(i+1,pt[0],pt[1],pt[2]))
				geom.saveFile("%s/wrench_slice_%d.obj"%(savedir,i+1))
			except Exception:
				out.write("%d,%f,%f,%f,FAILED\n"%(i+1,pt[0],pt[1],pt[2]))
				geom.saveFile("%s/wrench_slice_%d.obj"%(savedir,i+1))

			pts.append(pt)
			volumes.append(V)
		out.close()

		print "# of non-convex force volumes:",numNonConvex
		W = polytope.GeneralizedWrenchPolytope()
		W.setFromForceSamples(volumes,pts)
		#W.saveForceSamples(savedir,volumes,pts)
		W.test(100)
		return W

	def load_6D_wrench_space(self):
		savedir = "data/"+self.world.robot(0).getName()+'/'+self.config_name
		W = polytope.GeneralizedWrenchPolytope()
		W.loadForceSamples(savedir)
		W.test(100)
		return W

	def calculate_3D_wrench_space(self):
		"""Returns a ConvexHull object in fx fz ty wrench space"""
		#method = 'fx_fy_fz'
		method = 'fx_fz_ty'
		savedir = "data/"+self.world.robot(0).getName()+'/'+self.config_name
		mkdir_p(savedir)

		solver_stats = dict()
		solver_stats['time'] = []
		solver_stats['num_lp_solves'] = []
		solver_stats['feasible'] = []

		spheremat = []
		h = 10
		w = 20
		for i in range(h+1):
			phi = float(i)/float(h)*math.pi
			z = math.cos(phi)
			xy = math.sin(phi)
			nw = int(math.floor(xy*w))+1
			#nw = w
			for j in range(nw):
				theta = float(j)/float(nw)*math.pi*2
				x = xy * math.cos(theta)
				y = xy * math.sin(theta)
				spheremat.append((x,z,y))
		sphere = scipy.spatial.ConvexHull(spheremat)
		cm = se3.apply(self.world.rigidObject(0).getTransform(),self.world.rigidObject(0).getMass().getCom())
		out = open("%s/wrench_data_%s.csv"%(savedir,method),'w')
		suffixes = ['fx','fy','fz','tx','ty','tz']
		row = suffixes + ['amount']
		robot = self.world.robot(0)
		for unit in self.contact_units:
			n = self.hand.all_units[unit][0]
			row += [n+'_'+suff for suff in suffixes]
		row += ['t_'+robot.link(link).getName() for link in range(robot.numLinks())]
		out.write(",".join(str(v) for v in row) + '\n')
		for i in range(len(sphere.points)):
			wext = sphere.points[i]
			
			if method == 'fx_fy_fz':
				#x y z force
				f = [wext[0],wext[1],wext[2]]
				m = [0,0,0]
			else:
				#force in x-z plane and moment about y
				f = [wext[0],0,wext[2]]
				m = [0,wext[1]/10,0]
			
			wext = f + m
			t0 = time.time()
			try:
				res = self.solve(self.contact_units,wext=wext,mode='wrench_maximization')
			except KeyboardInterrupt:
				q = raw_input("Do you want to quit? (y/n)> ")
				if q == 'y':
					break
				else:
					res = False
					self.wrenches = None
			t1 = time.time()
			solver_stats['time'].append(t1-t0)
			solver_stats['num_lp_solves'].append(self.last_solver.numConvexSolves)
			if self.last_solver.numConvexSolves > 1000:
				raw_input("Excessive number of convex solves... press enter to continue")

			if self.wrenches == None:
				w = np.zeros((6,))
				solver_stats['feasible'].append(0)
			else:
				w = -sum(self.wrenches,np.zeros(6,))
				solver_stats['feasible'].append(1)

			print "****** Direction",wext,"value",w,"amount",np.dot(wext,w),"******"
			row = [v for v in wext] + [np.dot(wext,w)]
			if self.wrenches is not None:
				for wi in self.wrenches:
					row += [v for v in wi]
				if self.torques is not None:
					row += [v for v in self.torques]
			out.write(",".join(str(v) for v in row) + '\n')
			assert np.dot(wext,w) >= -1e-5
			#if np.linalg.norm(w) > 0:
			#	print "  Cos angle",np.dot(wext,w)/(np.linalg.norm(w)*np.linalg.norm(wext))
			f = w[0:3]
			t = w[3:6]
			if method == 'fx_fy_fz':
				#x y z force
				sphere.points[i] = [f[0],f[1],f[2]]
			else:
				#force in x-z plane and moment about y -- scaled by 10
				sphere.points[i] = [f[0],t[1]*10,f[2]]
			#debugging
			"""if np.dot(wext,w) >= 1e-3:
				solvedLarger = self.solve(self.contact_units,wext=w * 1.1)
				solvedSmaller = self.solve(self.contact_units,wext=w * 0.9)
				print "Bigger wrench",solvedLarger
				print "Smaller wrench",solvedSmaller
				assert not solvedLarger and solvedSmaller
			"""
		out.close()

		ch = scipy.spatial.ConvexHull(sphere.points)
		if len(ch.vertices) == sphere.points.shape[0]:
			sphere = ch
		else:
			#now begin a refining process to make this polyhedron look a bit better
			refine_hull(sphere)

		print
		print "STATS"
		for s,values in solver_stats.iteritems():
			if s == 'time':
				print "%s: %dms [%dms,%dms]"%(s,int(float(sum(values))/len(values)*1000),int(min(values)*1000),int(max(values)*1000))
			else:
				print "%s: %f [%f,%f]"%(s,float(sum(values))/len(values),min(values),max(values))

		return sphere

	def save_wrench_space(self,fn):
		fn = '%s/wrench_space_%s.off'%(savedir,method)
		f = open(fn,'w')
		f.write("OFF\n")
		f.write('%d %d 0\n'%(len(sphere.points),len(sphere.simplices)))
		for p in sphere.points:
			f.write('%f %f %f\n'%(p[0],p[1],p[2]))
		for s in sphere.simplices:
			f.write('3 %d %d %d\n'%(s[0],s[1],s[2]))
		f.close()
		print "Written to",fn


if __name__ == '__main__':
	h = hand_models.spiny_hand()
	#h hand_models.spiny_twofinger()
	#h = hand_models.spiny_opposing()
	plugin = GLHandStabilityPlugin(h)
	doedit = True
	if len(sys.argv) > 1:
		plugin.load_config(sys.argv[1])
		doedit = False
	if len(sys.argv) > 2:
		if sys.argv[2] == 'edit':
			doedit = True
	if doedit:
		#links = plugin.edit_contacting_links()
		#print "Using the following links:",links
		units = plugin.edit_contacting_units()
		print "Using the following units:",units

		vis.setWindowTitle("Editing hand and object configuration")
		vis.pushPlugin(plugin)
		vis.dialog()
		vis.setWindowTitle("Klamp't visualization")

		plugin.save_config("last")
		vis.kill()
		exit(0)
	else:
		units = plugin.contact_units

	coordinates.updateFromWorld()

	times = []
	angle = 0
	dtheta = 5
	angles = []
	feasible = []
	plugin.rotate(math.radians(angle))
	for i in range(360/dtheta):
		angles.append(angle)
		t0 = time.time()
		try:
			feasible.append(plugin.solve(units))
		except KeyboardInterrupt:
			q = raw_input("Do you want to quit? (y/n)> ")
			if q == 'y':
				break
			else:
				feasible.append('fail')
		t1 = time.time()
		times.append(t1-t0)
		plugin.rotate(math.radians(dtheta))
		angle += dtheta
	print "Angle, Feasibility:"
	for (a,f) in zip(angles,feasible):
		print a,f
	print "Solve time %dms [%dms,%dms]"%(int(sum(times)/len(times)*1000),int(min(times)*1000),int(max(times)*1000))
