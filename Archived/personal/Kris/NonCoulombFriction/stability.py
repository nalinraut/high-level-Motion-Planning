import pkg_resources
pkg_resources.require("Klampt>=0.7.0")
from klampt.math import *
from klampt import PointPoser,TransformPoser
import klampt.vis
from klampt import vis
from klampt.vis.glcommon import *
from klampt.vis import gldraw
from polytope import *
import scipy.optimize
import cvxopt
import numpy as np
import sys
import math
import time
import heapq
import random

class GeneralizedFrictionConeFactory:
	"""
	- forceTemplate is a 3D Volume stored in the local frame of a contact patch
	- contactAreaPolicy: not implemented yet
	- hysteresisPolicy: not implemented yet
	"""
	def __init__(self):
		self.forceTemplate = None
		self.contactAreaPolicy = None
		self.hysteresisPolicy = None
	def load(self,folder):
		self.forceTemplate = Volume()
		self.forceTemplate.load(folder)
	def make(self,Tcontact,contactSize=None):
		"""For a contact interface with size s, center t, outward facing normal z,
		and two orthogonal directions x and y describing the orientation of the contact
		interface, yields (c,W,V) where c is a list of contact points c1,...,ck,
		W is a list of 6 x 3 wrench matrices W1,..,Wk, and V is a 3-D volume  describing the 
		friction cone.  Here k is either 1 or 4, depending on the contact patch size.

		The contact transform is given by Tcontact=(R,t) where R = [x|y|z] is its 
		orientation.

		contactSize is either None, a number, or a pair. In the second case, it assumes
		the contact patch is a square.  In the second, it's a rectangle. Axes are aligned in the
		x,y directions.

		The total wrench (i.e., the (force,torque) pair about the origin) is W1*f1+...Wk*fk.
		The forces must satisfy the constraints f1,...,fk in V.  Specifically the fi variables
		are the forces in the local frames of each contact patch.
		"""
		R,t = Tcontact
		F = self.forceTemplate.transform(np.array(so3.matrix(R)),scale=(None if contactSize == None else 0.25),orthogonal=True)
		#F = self.forceTemplate.transform(np.array(so3.matrix(R))*(1.0 if contactSize == None else 0.25))
		contacts = [t]
		if contactSize != None:
			if not hasattr(contactSize,'__iter__'):
				contactSize = (contactSize,contactSize)
			a1 = (contactSize[0]*0.5,contactSize[1]*0.5,0)
			a2 = (contactSize[0]*0.5,-contactSize[1]*0.5,0)
			a3 = (-contactSize[0]*0.5,-contactSize[1]*0.5,0)
			a4 = (-contactSize[0]*0.5,contactSize[1]*0.5,0)
			contacts = [se3.apply(Tcontact,a) for a in [a1,a2,a3,a4]]
		W = []
		R = np.array(so3.matrix(R))
		for i,p in enumerate(contacts):
			Wi = np.vstack((np.eye(3),np.array(so3.matrix(so3.cross_product(p)))))
			W.append(Wi)
		return contacts,W,F

class GeneralizedStabilitySolver:
	"""A solver for a generalized stability problem with nonconvex friction cones.

	Attributes:
	- factory: a GeneralizedFrictionConeFactory giving the friction cone for each contact
	- numContactPatches: the number of contact patches (each patch may have more than one contact point)
	- contactPoints: the world positions of each contact point
	- wrenchMatrices: the wrench matrices for each contact point.
	- frictionVolumes: the friction cone Volume for each contact point.
	- normalDirections: the normal direction for each contact point.
	- pointIndices: the contact patch ID for each contact point.
	- wext: the external force.  If you plan to reuse this data structure for multiple solves, call
	  setExternalWrench or setGravityWrench rather than modifying this directly
	- minimize: if stability is impossible, the solver will find the forces that minimize the object's
	  residual wrench or some other custom objective function
	- P,q: the objective function is 1/2 f^T P f + q^T f where f is the optimization vector.
	  An easy way to set these is with setWrenchObjective().
	- c: temporary variable giving objective function direction
	- A_eq,b_eq: temporary variables including all equality constraints
	- A_eq_extra,b_eq_extra: additional equality constraints
	- forces: temporary variables giving forces at each contact point, in world coordinates
	- maximizeWrenchMagnitude: if true, tries to optimize the maximum external wrench in proportion to wext
	- maximizeWrenchDot: if true, tries to optimize the external wrench to maximize the dot product with wext
	"""
	def __init__(self,factory):
		self.factory = factory
		self.numContactPatches = 0
		self.contactPoints = []
		self.wrenchMatrices = []
		self.frictionVolumes = []
		self.normalDirections = []
		self.pointIndices = []
		self.wext = None
		self.minimize = False
		self.maximizeWrenchMagnitude = False
		self.maximizeWrenchDot = False
		self.P,self.q = None,None
		self.c = None
		self.c_cvxopt = None
		self.A_eq_extra,self.b_eq_extra = [],[]
		self.A_ineq_extra,self.b_ineq_extra = [],[]
		self.forces = None
		self.numConvexSolves = 0
	def setGravityWrench(self,mass,cm=(0,0,0),gravity=(0,0,-9.8)):
		"""	- mass: the object's mass
			- cm: the object's center of mass (default (0,0,0)).  
			- gravity: the gravity vector (default (0,0,-9.8))
		"""
		assert mass > 0
		self.wext = mass*np.array(gravity+vectorops.cross(cm,gravity))
		self.c = None
		self.c_cvxopt = None
	def setExternalWrench(self,w):
		"""Sets an external wrench (6D force and torque).  Can be a list or np.ndarray"""
		self.wext = np.array(w)
		assert self.wext.shape == (6,)
		self.c = None
		self.c_cvxopt = None
	def addContactPatch(self,Tpatch,patchSize=None,type=None):
		"""Adds a new contact patch with transform Tpatch = (Rpatch,t) with Rpatch = [x,y,z].
		The patch center is center t, outward facing normal z, and two orthogonal directions x and y
		describe the orientation of the contact interface.

		If given, patchSize is the size of the contact patch.
		"""
		id = self.numContactPatches
		if type is None:
			c,W,Fc = self.factory.make(Tpatch,patchSize)
		else:
			c,W,Fc = self.factory[type].make(Tpatch,patchSize)
		self.contactPoints += c
		self.wrenchMatrices += W
		self.frictionVolumes += [Fc]*len(W)
		z = so3.apply(Tpatch[0],[0,0,1])
		self.normalDirections += [z]*len(W)
		self.pointIndices += [id]*len(W)
		self.numContactPatches+= 1
		return id
	def addLinearConstraint(self,A,b,bhigh=None):
		"""Can either add an equality A^T f = b
		Or an inequality b <= A^T f <= bhigh.

		f here is the stacked vector of forces applied to the object.

		For one-sided inequalities, set b to -inf or bhigh to inf.
		"""
		assert self.c == None,"Can only call addLinearConstraint once"
		assert len(A) == len(self.contactPoints)*3,"The A matrix is not the right size"
		if bhigh == None or b == bhigh:
			self.A_eq_extra.append(A.reshape((1,len(A))))
			self.b_eq_extra.append(b)
		else:
			assert b < bhigh,"Range must be ordered"
			if b > -float('inf'):
				self.A_ineq_extra.append(-A.reshape((1,len(A))))
				self.b_ineq_extra.append(-b)
			if bhigh < float('inf'):
				self.A_ineq_extra.append(A.reshape((1,len(A))))
				self.b_ineq_extra.append(bhigh)
	def setWrenchObjective(self,force_term,moment_term):
		"""Sets the solver to minimize resultant wrench instead of returning failure, and 
		the given terms (either np 3x3 matrices or scalars) are used to define the quadratic
		cost terms P and q.  This must be called after the masses and contact patches are set."""
		#min ||W f - v||_C^2 = (Wf-v)^T C (Wf-v) = f^T W^T C W f - 2 f^T W^T C v + v^T C v
		assert self.c is not None,"Need to set the external wrench first"
		self.minimize = True
		W = np.hstack(self.wrenchMatrices)
		assert self.wext != None
		v = -self.wext
		C = np.zeros((6,6))
		if hasattr(force_term,'__iter__'):
			C[:3,:3] = force_term
		else:
			C[0,0] = C[1,1] = C[2,2] = force_term
		if hasattr(moment_term,'__iter__'):
			C[3:,3:] = moment_term
		else:
			C[3,3] = C[4,4] = C[5,5] = moment_term
		self.P = np.dot(np.dot(W.T,C),W)
		self.q = -np.dot(W.T,np.dot(C,v))
		self.r = np.dot(v.T,np.dot(C,v))*0.5
	def solveHulls(self,convex_hulls,objective_bound=None):
		"""Given force convex hulls, returns (q,x), where q is the quality of the solution and x is the
		list of forces (or None if the problem is infeasible).

		If self.minimize = True, q is the resulting kinetic energy of the object.
		Otherwise, q = True indicates success and f = False indicates infeasibility

		If self.maximizeWrenchMagnitude = True, tries to maximize the magnitude of the wrench proportional to Wext for feasibility to hold
		max w s.t. W f + w wext = 0, fi in FCi

		if self.maximizeWrenchDot = True, tries to maximize forces in the direction Wext for feasibility to hold
		max w^T wext s.t. W f + w = 0, fi in FCi

		Cannot have more than one of minimize, maximizeWrenchMagnitude, or maximizeWrenchDot true at the same time.
		"""
		objective_bound = None
		self.numConvexSolves += 1
		t0 = time.time()
		do_regularize = True
		norm = float('inf')
		nf = len(self.contactPoints)*3
		numnew = 0
		if self.maximizeWrenchMagnitude or self.maximizeWrenchDot or self.minimize:
			do_regularize = False
		if self.maximizeWrenchMagnitude:
			numnew = 1
		if do_regularize:
			if norm == 1:
				#objective is sum{k,i} |fk[i] - ck[i]|
				#introduce auxiliary variables e{k,i}
				#minimize sum of e{k,i} subject to
				#fk[i] - ck[i] <= e{k,i} 
				#-fk[i] + ck[i] <= e{k,i} 
				raise NotImplementedError("L-1 regularization")
			else:
				#objective is max_i,k |fk[i] - ck[i]|
				#add a new variable d for each contact point
				#minimize d subject to 
				#|fk[i] - ck[i]| < dk for all i
				numnew = 1
		w = self.wext

		if self.c is None:
			self.c = np.array(sum(self.normalDirections,()))
			if do_regularize:
				self.c = np.array([0.0]*nf + [1.0]*numnew)
			self.A_eq = np.hstack(self.wrenchMatrices)
			if self.maximizeWrenchMagnitude or self.maximizeWrenchDot:
				self.b_eq = np.zeros((6,))
			else:
				self.b_eq = -w
			if len(self.A_eq_extra) > 0:
				self.A_eq = np.vstack([self.A_eq]+self.A_eq_extra)
				self.b_eq = np.hstack([self.b_eq,np.array(self.b_eq_extra)])
			if self.maximizeWrenchMagnitude:
				self.c = np.array([0.0]*nf + [1.0]*numnew)
				#add a column with wext
				self.A_eq = np.hstack((self.A_eq,-w.reshape((self.A_eq.shape[0],numnew))))
			else:
				#add a column of 0's
				self.A_eq = np.hstack((self.A_eq,np.zeros((self.A_eq.shape[0],numnew))))
		assert self.A_eq is not None
		assert self.A_eq.shape[0] <= self.c.shape[0]
		if self.c_cvxopt == None:
			self.c_cvxopt = cvxopt.matrix(self.c)				
			self.A_eq_cvxopt = cvxopt.matrix(self.A_eq)
			self.b_eq_cvxopt = cvxopt.matrix(self.b_eq)
		planes = [halfplanes(V) for V in convex_hulls]
		A_ineq = scipy.linalg.block_diag(*[A for (A,b) in planes])
		#A_ineq_blocks = []
		#for i,(A,b) in enumerate(planes):
		#	row = [None]*len(planes)
		#	row[i] = A
		#	A_ineq_blocks.append(row)
		b_ineq = np.hstack([b for (A,b) in planes])
		#add user-defined constraints
		if len(self.A_ineq_extra) > 0:
			A_ineq = np.vstack([A_ineq]+self.A_ineq_extra)
			#A_ineq_planes = np.vstack([A_ineq]+self.A_ineq_extra)
			b_ineq = np.hstack([b_ineq,np.array(self.b_ineq_extra)])
			#for (A,b) in zip(self.A_ineq_extra,self.b_ineq_extra):
			#	print A,"* f <=",b
			#raw_input()
		if numnew > 0:
			#add required columns of 0's
			A_ineq = np.hstack((A_ineq,np.zeros((A_ineq.shape[0],numnew))))
			if objective_bound is not None and objective_bound != float('inf'):
				assert numnew == 1
				row = [0.0]*(nf + numnew)
				row[-1] = 1.0
				A_ineq = np.vstack((A_ineq,[row]))
				b_ineq = np.hstack((b_ineq,[objective_bound]))
		if do_regularize:
			#add rows constraining |f - c| < d
			newrows = []
			newrhs = []
			centers = [sum([V.points[v] for v in V.vertices],np.zeros((3,)))/len(V.vertices) for V in convex_hulls]
			for i,c in enumerate(centers):
				for k in range(3):
					#f - c < d
					row = [0.0]*(nf + numnew)
					row[i*3+k] = 1
					row[-1] = -1
					#row[nf+i] = -1
					rhs = c[k]
					newrows.append(row)
					newrhs.append(rhs)
					#-f + c < d
					row = [0.0]*(nf + numnew)
					row[i*3+k] = -1
					row[-1] = -1
					#row[nf+i] = -1
					rhs = -c[k]
					newrows.append(row)
					newrhs.append(rhs)			
			A_ineq = np.vstack((A_ineq,np.array(newrows)))
			b_ineq = np.hstack((b_ineq,np.array(newrhs)))
		assert A_ineq.shape[0] == len(b_ineq)
		assert self.c.shape[0] == A_ineq.shape[1]
		assert A_ineq.shape[1] == self.A_eq.shape[1]
		#scipy is found to be quite a bit slower than CVXOPT on average and also comes up with some poor solutions
		#res = scipy.optimize.linprog(self.c,A_eq=self.A_eq,b_eq=self.b_eq,A_ub=A_ineq,b_ub=b_ineq,bounds=[(None,None)]*len(self.c))
		cvxopt.solvers.options['show_progress'] = False
		if self.minimize:
			#ignore equilibrium constraint, add unilateral constraints
			W = self.A_eq[:6]
			g = -self.b_eq[:6]
			#NW^T M^-1(W*f+g) >= 0
			#need Af <= b so -NW^TM^-1W f <= NW^TM^-1 g
			#TODO: inertia matrix in M
			N = np.dot(scipy.linalg.block_diag(*self.normalDirections),W.T)
			NW = np.dot(N,W)
			Ng = np.dot(N,g)
			G = cvxopt.matrix(np.vstack((A_ineq,-NW)))
			h = cvxopt.matrix(np.hstack((b_ineq,Ng)))
			t1 = time.time()
			if do_regularize:
				raise NotImplementedError("Can't do regularization with minimization")
			res = cvxopt.solvers.qp(cvxopt.matrix(self.P),cvxopt.matrix(self.q),G,h,cvxopt.matrix(self.A_eq[6:,:]),cvxopt.matrix(self.b_eq[6:]))
			print "QP SOLVE TIME",time.time()-t1,"SETUP TIME",t1-t0,"PROBLEM SIZE",A_ineq.shape
			assert res['x']!=None
			f = [v for v in res['x']]
			return (res['primal objective']+self.r,np.split(np.array(f),len(self.contactPoints)))
		else:
			#this takes about 3 ms? could shave off a few % of running time by optimizing sparse matrix construction
			G = cvxopt.sparse(cvxopt.matrix(A_ineq))
			h = cvxopt.matrix(b_ineq)
			t1 = time.time()
			res = cvxopt.solvers.lp(self.c_cvxopt,G,h,self.A_eq_cvxopt,self.b_eq_cvxopt)
			self.last_res = res
			#print "Primal objective",res['primal objective']
			#print "LP SOLVE TIME",time.time()-t1,"SETUP TIME",t1-t0,"PROBLEM SIZE",A_ineq.shape

			#print res
			#print "Status:",res.status,"message:",res.message
			#raw_input()
			#if res.success:
			#	return (True,np.split(res.x,len(self.wrenchMatrices)))
			if res['x']!=None:
				f = [v for v in res['x']]
				if nf < len(f):
					f = f[:nf]
				if self.maximizeWrenchMagnitude or self.maximizeWrenchDot:
					if objective_bound != None:
						if res['primal objective'] > objective_bound:
							print res
							assert res['status'] == 'dual infeasible' or res['status'] == 'primal infeasible' or res['primal infeasibility'] > 1e-5 or res['dual infeasibility'] > 1e-5
							return (float('inf'),None)
					return (res['primal objective'],np.split(np.array(f),len(self.contactPoints)))
				return (True,np.split(np.array(f),len(self.contactPoints)))
			else:
				if self.maximizeWrenchMagnitude or self.maximizeWrenchDot:
					return (float('inf'),None)
				return (False,None)
	def solveComponents(self,components,objective_bound=None):
		"""Given integer component indices, returns a solution to all forces or None"""
		hulls = [(V.convex_hull if c<0 else V.convex_decomposition[c]) for V,c in zip(self.frictionVolumes,components)]
		return self.solveHulls(hulls,objective_bound)

	def solveScip(self):
		"""Uses pyscipopt"""
		from pyscipopt import Model,quicksum

		assert not self.minimize, "Can't solve a QP with Scip yet"
		do_regularize = True
		norm = float('inf')
		nf = len(self.contactPoints)*3
		numnew = 0
		if self.maximizeWrenchMagnitude or self.minimize:
			do_regularize = False
		if self.maximizeWrenchMagnitude:
			numnew = 1
		if do_regularize:
			if norm == 1:
				#objective is sum{k,i} |fk[i] - ck[i]|
				#introduce auxiliary variables e{k,i}
				#minimize sum of e{k,i} subject to
				#fk[i] - ck[i] <= e{k,i} 
				#-fk[i] + ck[i] <= e{k,i} 
				raise NotImplementedError("L-1 regularization")
			else:
				#objective is max_i,k |fk[i] - ck[i]|
				#add a new variable d for each contact point
				#minimize d subject to 
				#|fk[i] - ck[i]| < dk for all i
				numnew = 1
		w = self.wext

		M = 1000  #some large number, larger than possible maximum force
		model = Model("non-convex friction")
		zs = []
		fs = []
		ds = []
		for i,V in enumerate(self.frictionVolumes):
			zi = []
			for j in range(len(V.convex_decomposition)):
				zij = model.addVar(vtype="B", name="z"+str(i)+","+str(j))
				zi.append(zij)
			fix = model.addVar(vtype="C", name="f"+str(i)+"x", lb=-M, ub=None)
			fiy = model.addVar(vtype="C", name="f"+str(i)+"y", lb=-M, ub=None)
			fiz = model.addVar(vtype="C", name="f"+str(i)+"z", lb=-M, ub=None)
			fs += [fix,fiy,fiz]
			zs.append(zi)
		if numnew:
			for i in range(numnew):
				di = model.addVar(vtype="C", name="d"+str(i), lb=-M, ub=None)
				ds.append(di)

		self.c = np.array(sum(self.normalDirections,()))
		if do_regularize:
			self.c = np.array([0.0]*nf + [1.0]*numnew)
		self.A_eq = np.hstack(self.wrenchMatrices)
		if self.maximizeWrenchMagnitude:
			self.b_eq = np.zeros((6,))
		else:
			self.b_eq = -w
		if len(self.A_eq_extra) > 0:
			self.A_eq = np.vstack([self.A_eq]+self.A_eq_extra)
			self.b_eq = np.hstack([self.b_eq,np.array(self.b_eq_extra)])
		if self.maximizeWrenchMagnitude:
			self.c = np.array([0.0]*nf + [1.0]*numnew)
			#add a column with wext
			self.A_eq = np.hstack((self.A_eq,-w.reshape((self.A_eq.shape[0],numnew))))
		else:
			#add a column of 0's
			self.A_eq = np.hstack((self.A_eq,np.zeros((self.A_eq.shape[0],numnew))))

		# Set up objective
		model.setObjective(quicksum(ci*fi for (ci,fi) in zip(self.c,fs+ds)), 'minimize')
		# Set up equality constraints
		for i in xrange(self.A_eq.shape[0]):
			model.addCons(quicksum(ci*fi for (ci,fi) in zip(self.A_eq[i,:],fs+ds)) == self.b_eq[i],name="eq"+str(i))
		# Set up friction constraints
		for i,V in enumerate(self.frictionVolumes):
			for j,Vj in enumerate(V.convex_decomposition):
				A,b = halfplanes(Vj)
				for k in xrange(A.shape[0]):
					model.addCons(A[k,0]*fs[i*3] + A[k,1]*fs[i*3+1] + A[k,2]*fs[i*3+2] <= b[k] + (1-zs[i][j])*M,name="F%d,%d,%d"%(i,j,k))
			model.addCons(quicksum(zs[i])==1,name="z_equality"+str(i))
		#add user-defined constraints
		for i,(A,b) in enumerate(zip(self.A_ineq_extra,self.b_ineq_extra)):
			model.addCons(quicksum(ci*fi for (ci,fi) in zip(A[0],fs)) <= b,name="user_ineq"+str(i))
		#add regularization constraints
		if do_regularize:
			#add rows constraining |f - c| < d
			centers = [sum([V.convex_hull.points[v] for v in V.convex_hull.vertices],np.zeros((3,)))/len(V.convex_hull.vertices) for V in self.frictionVolumes]
			for i,c in enumerate(centers):
				for k in range(3):
					#f - c < d
					model.addCons(fs[i*3+k] - ds[0] <= c[k],name="regularize"+str(i*3+k))
					#-f + c < d
					model.addCons(-fs[i*3+k] - ds[0] <= -c[k],name="regularize2"+str(i*3+k))
		model.data = fs,zs,ds
		t0 = time.time()
		print "Beginning SCIP optimization..."
		model.optimize()
		t1 = time.time()
		print "SOLVE TIME",t1-t0
		print "STATUS:",model.getStatus()
		if model.getStatus() != 'optimal':
			return None
		print "Optimal value :",model.getObjVal()
		flist = [model.getVal(f) for f in fs]
		self.forces = np.split(np.array(flist),len(self.contactPoints))
		wrenches = [np.zeros(6) for i in xrange(max(self.pointIndices)+1)]
		for p,W,f in zip(self.pointIndices,self.wrenchMatrices,self.forces):
			wrenches[p] += np.dot(W,f)
		return wrenches
	
	def solveGurobi(self):
		"""Uses Gurobi"""
		from gurobipy import Model

		assert not self.minimize, "Can't solve a QP with Gurobi yet"
		do_regularize = True
		norm = float('inf')
		nf = len(self.contactPoints)*3
		numnew = 0
		if self.maximizeWrenchMagnitude or self.minimize:
			do_regularize = False
		if self.maximizeWrenchMagnitude:
			numnew = 1
		if do_regularize:
			if norm == 1:
				#objective is sum{k,i} |fk[i] - ck[i]|
				#introduce auxiliary variables e{k,i}
				#minimize sum of e{k,i} subject to
				#fk[i] - ck[i] <= e{k,i} 
				#-fk[i] + ck[i] <= e{k,i} 
				raise NotImplementedError("L-1 regularization")
			else:
				#objective is max_i,k |fk[i] - ck[i]|
				#add a new variable d for each contact point
				#minimize d subject to 
				#|fk[i] - ck[i]| < dk for all i
				numnew = 1
		w = self.wext

		M = 1000  #some large number, larger than possible maximum force
		model = Model("non-convex friction")
		zs = []
		fs = []
		ds = []
		for i,V in enumerate(self.frictionVolumes):
			zi = []
			for j in range(len(V.convex_decomposition)):
				zij = model.addVar(vtype=GRB.BINARY, name="z"+str(i)+","+str(j))
				zi.append(zij)
			fix = model.addVar( name="f"+str(i)+"x", lb=-M, ub=None)
			fiy = model.addVar( name="f"+str(i)+"y", lb=-M, ub=None)
			fiz = model.addVar( name="f"+str(i)+"z", lb=-M, ub=None)
			fs += [fix,fiy,fiz]
			zs.append(zi)
		if numnew:
			for i in range(numnew):
				di = model.addVar( name="d"+str(i), lb=-M, ub=None)
				ds.append(di)

		self.c = np.array(sum(self.normalDirections,()))
		if do_regularize:
			self.c = np.array([0.0]*nf + [1.0]*numnew)
		self.A_eq = np.hstack(self.wrenchMatrices)
		if self.maximizeWrenchMagnitude:
			self.b_eq = np.zeros((6,))
		else:
			self.b_eq = -w
		if len(self.A_eq_extra) > 0:
			self.A_eq = np.vstack([self.A_eq]+self.A_eq_extra)
			self.b_eq = np.hstack([self.b_eq,np.array(self.b_eq_extra)])
		if self.maximizeWrenchMagnitude:
			self.c = np.array([0.0]*nf + [1.0]*numnew)
			#add a column with wext
			self.A_eq = np.hstack((self.A_eq,-w.reshape((self.A_eq.shape[0],numnew))))
		else:
			#add a column of 0's
			self.A_eq = np.hstack((self.A_eq,np.zeros((self.A_eq.shape[0],numnew))))

		# Set up objective
		model.setObjective(sum(ci*fi for (ci,fi) in zip(self.c,fs+ds)), 'minimize')
		# Set up equality constraints
		for i in xrange(self.A_eq.shape[0]):
			model.addCons(sum(ci*fi for (ci,fi) in zip(self.A_eq[i,:],fs+ds)) == self.b_eq[i],name="eq"+str(i))
		# Set up friction constraints
		for i,V in enumerate(self.frictionVolumes):
			for j,Vj in enumerate(V.convex_decomposition):
				A,b = halfplanes(Vj)
				for k in xrange(A.shape[0]):
					model.addCons(A[k,0]*fs[i*3] + A[k,1]*fs[i*3+1] + A[k,2]*fs[i*3+2] <= b[k] + (1-zs[i][j])*M,name="F%d,%d,%d"%(i,j,k))
			model.addCons(sum(zs[i])==1,name="z_equality"+str(i))
		#add user-defined constraints
		for i,(A,b) in enumerate(zip(self.A_ineq_extra,self.b_ineq_extra)):
			model.addCons(sum(ci*fi for (ci,fi) in zip(A[0],fs)) <= b,name="user_ineq"+str(i))
		#add regularization constraints
		if do_regularize:
			#add rows constraining |f - c| < d
			centers = [sum([V.convex_hull.points[v] for v in V.convex_hull.vertices],np.zeros((3,)))/len(V.convex_hull.vertices) for V in self.frictionVolumes]
			for i,c in enumerate(centers):
				for k in range(3):
					#f - c < d
					model.addCons(fs[i*3+k] - ds[0] <= c[k],name="regularize"+str(i*3+k))
					#-f + c < d
					model.addCons(-fs[i*3+k] - ds[0] <= -c[k],name="regularize2"+str(i*3+k))
		model.data = fs,zs,ds
		t0 = time.time()
		print "Beginning Gurobi optimization..."
		model.optimize()
		t1 = time.time()
		print "SOLVE TIME",t1-t0
		print "STATUS:",model.getStatus()
		if model.getStatus() != 'optimal':
			return None
		print "Optimal value :",model.getObjVal()
		flist = [model.getVal(f) for f in fs]
		self.forces = np.split(np.array(flist),len(self.contactPoints))
		wrenches = [np.zeros(6) for i in xrange(max(self.pointIndices)+1)]
		for p,W,f in zip(self.pointIndices,self.wrenchMatrices,self.forces):
			wrenches[p] += np.dot(W,f)
		return wrenches

	def solve(self):
		"""Returns a list of all contact wrenches on the contact patches, or None if this could not be found"""
		#TEMP: test Gurobi
		#return self.solveGurobi()
		#TEMP: test SCIP
		#return self.solveScip()

		self.numConvexSolves = 0
		if self.minimize or self.maximizeWrenchMagnitude:
			res,self.forces = self.minimize_recurse([-1]*len(self.wrenchMatrices))
			self.objectiveValue = res
		else:
			res,self.forces = self.solve_recurse([-1]*len(self.wrenchMatrices))
			#Try heuristic solve first
			#res,self.forces = self.solve_recurse_heuristic([-1]*len(self.wrenchMatrices))
			#if res == 'unknown':
			#	res,self.forces = self.solve_recurse([-1]*len(self.wrenchMatrices))
		self.c = None
		self.c_cvxopt = None
		if self.forces == None:
			return None
		wrenches = [np.zeros(6) for i in xrange(max(self.pointIndices)+1)]
		for p,W,f in zip(self.pointIndices,self.wrenchMatrices,self.forces):
			wrenches[p] += np.dot(W,f)
		return wrenches

	def solve_recurse(self,components):
		outerhulls = [0 if c >= 0 else 1 for c in components]
		res,soln = self.solveComponents(components)
		if not res:
			#this branch failed
			print "Solve",components,"infeasible"
			return False,None
		t0 = time.time()
		split_order = []
		child = components[:]
		for i,outer in enumerate(outerhulls):
			if outer:
				#in the outer convex hull... check for inclusion in one of the other volumes
				mind = float('inf')
				for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
					d = hull_distance(soln[i],h)
					mind = min(d,mind)
					if d <= 0:
						child[i] = j
						break
				if mind <= 0:
					#inside volume
					continue
				#worst violation first
				split_order.append((-mind,i))
		if len(split_order) == 0:
			#solved!
			print "Found a solution inside all convex components",
			if sum(outerhulls) == len(components):
				print
			else:
				print components
			return res,soln

		#Try recursion with everything inside assigned
		for (d,i) in split_order:
			child[i] = random.randint(0,len(self.frictionVolumes[i].convex_decomposition)-1)
		print "Testing random assignment",child,"..."
		res,csoln = self.solveComponents(child)
		if res:
			print "Solved a random solution"
			return True,csoln
		print "Failed solving a random solution"
		
		split_order = sorted(split_order)
		print "Overhead time",time.time()-t0,"max distance",-split_order[0][0]
		#loop over some other decomposition
		for (d,i) in split_order:
			#determine a recursion order
			child = components[:]
			order = []
			for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
				order.append((hull_distance(soln[i],h),j))
			order = sorted(order)
			print "Splitting on child",i,"force",soln[i],"distances to volumes",[v[0] for v in order]
			#t1 = time.time()
			#print "Overhead time",t1-t0
			#recurse
			for j in xrange(len(self.frictionVolumes[i].convex_decomposition)):
				child[i] = order[j][1]
				print child
				cres,csoln = self.solve_recurse(child)
				if cres:
					print "  Successful result on child",i,"branch",order[j][1],"force",csoln[i]
					return cres,csoln
			#since the recursive call above checks all other components, can safely stop here
			break
		print "Branch",components,"failed"
		return False,None
	def solve_recurse_heuristic(self,components):
		"""Can return True, False (no solution) or 'unknown'."""
		outerhulls = [0 if c >= 0 else 1 for c in components]
		res,soln = self.solveComponents(components)
		if not res:
			#this branch failed
			print "Solve",components,"infeasible"
			return False,None
		t0 = time.time()
		split_order = []
		child = components[:]
		nassigned = 0
		for i,outer in enumerate(outerhulls):
			if outer:
				#in the outer convex hull... check for inclusion in one of the other volumes
				mind = float('inf')
				for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
					d = hull_distance(soln[i],h)
					mind = min(d,mind)
					if d <= 0:
						child[i] = j
						nassigned += 1
						break
				if mind <= 0:
					#inside volume
					continue
				#worst violation first
				split_order.append((-mind,i))
		if len(split_order) == 0:
			#solved!
			print "Found a solution inside all convex components",
			if sum(outerhulls) == len(components):
				print
			else:
				print components
			return res,soln
		
		split_order = sorted(split_order)
		#print "Overhead time",time.time()-t0,"distances",[(-v[0],v[1]) for v in split_order]
		#loop over some other decomposition
		for (d,i) in split_order:
			#determine a recursion order
			order = []
			for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
				order.append((hull_distance(soln[i],h),j))
			order = sorted(order)
			#print "Splitting on child",i,"force",soln[i],"distances to volumes",[v[0] for v in order]
			#t1 = time.time()
			#print "Overhead time",t1-t0
			#recurse
			for j in xrange(len(self.frictionVolumes[i].convex_decomposition)):
				child[i] = order[j][1]
				print child
				cres,csoln = self.solve_recurse_heuristic(child)
				if cres == True:
					print "  Successful result on child",i,"branch",order[j][1],"force",csoln[i]
					return cres,csoln
			#since the recursive call above checks all other components, can safely stop here
			break
		print "Branch",components,"failed"
		if nassigned > 0:
			return 'unknown',None
		else:
			return False,None

	def minimize_recurse(self,components,bound=float('inf')):
		epsilon = 1e-3
		queue = []
		res,soln = self.solveComponents(components,bound)
		if res+(1+abs(res))*epsilon > bound:
			print "Solve outer bound",components,"exceeded bound",res,">",bound
			return (bound,None)
		best = soln
		heapq.heappush(queue,(res,components,soln))
		validset = [{} for i in range(len(components))]
		for i in range(len(components)):
			for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
				validset[i][j]=h
		visited = set()
		visited.add(tuple(components))
		numVisited = 0
		lastUpdated = 0
		while len(queue) > 0:
			numVisited += 1
			if numVisited >= lastUpdated + 1000:
				print "**** WENT FOR 1000 ITERATIONS WITHOUT IMPROVEMENT ****"
				raw_input()
				return (bound,best)
			if numVisited == 10000:
				print "**** EXCEEDED 10k ITERATION LIMIT ****"
				print "**** Last time the best was updated was iteration",lastUpdated,"*****"
				raw_input()
				return (bound,best)
			res,components,soln = heapq.heappop(queue)
			if res > bound:
				print "Solve outer bound",components,"exceeded bound",res,">",bound
				return bound,best
			if numVisited == len(components)*len(self.frictionVolumes[0].convex_decomposition):
				#try doing some exclusions
				print "**** TRYING EXCLUSIONS ****"
				for i in range(len(components)):
					validi = {}
					test = [-1]*len(components)
					for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
						test[i] = j
						cres,csoln = self.solveComponents(test,bound)
						if cres+(1+abs(cres))*epsilon > bound:
							print cres,">",bound
							print "***** ELIMINATING",i,j,"*****"
						else:
							validi[j] = h
					if len(validi) == 0:
						print "***** ALL PRUNED *****"
						#raw_input("Press enter to continue")
						return (bound,best)
					print "****** VALID FOR",i,validi.keys(),"******"
					validset[i] = validi
				#raw_input("Press enter to continue")

			for i in range(len(components)):
				if components[i] >= 0 and components[i] not in validset[i]:
					continue
			print components,"lower bound",res,"upper bound",bound
			outerhulls = [0 if c >= 0 else 1 for c in components]
			split_order = []
			inside = []
			for i,outer in enumerate(outerhulls):
				if outer:
					#in the outer convex hull... check for inclusion in one of the other volumes
					mind = float('inf')
					for j,h in enumerate(self.frictionVolumes[i].convex_decomposition):
						d = hull_distance(soln[i],h)
						mind = min(d,mind)
						if d <= 0:
							inside.append((i,j))
							break
					if mind <= 0:
						#inside volume
						continue
					#maximum violation first
					split_order.append((-mind,i))
			#TEMP: just use convex hulls
			#split_order = []
			if len(split_order) == 0:
				#solved!
				print "Found a solution inside all convex components",components,"quality",res
				#bound is lowered 
				if res < bound:
					bound,best = res,soln
					lastUpdated = numVisited
				continue
			if res+(1+abs(res))*epsilon > bound:
				print "Solve outer bound",components,"exceeded relaxed bound",res,"+epsilon >",bound
				return bound,best
			
			#evaluate some random point in the domain to obtain an upper bound
			child = components[:]
			for d,i in split_order:
				#DETERMINISTIC
				mind = float('inf')
				for j,h in validset[i].iteritems():
					d = hull_distance(soln[i],h)
					if d < mind:
						child[i] = j
						mind = d
				#RANDOM
				#child[i] = random.choice(validset[i].keys())
			for i,j in inside:
				child[i] = j
			cres,csoln = self.solveComponents(child,bound)
			if cres < bound:
				print "Point solution improved bound to",cres,"child",child
				bound = cres
				best = csoln
			else:
				print "Point solution",child,"got cost",cres
			visited.add(tuple(child))
			#within epsilon of optimum, no need to split
			if cres - (1+abs(cres))*epsilon < res:
				print "Close to best in node",cres,res,"avoiding split"
				continue

			#loop over some other decomposition
			split_order = sorted(split_order)
			for (d,i) in split_order:
				print "Splitting on child",i,"force",soln[i],"lower bound",res,"upper bound",bound
				#subdivide and add to queue
				for j in validset[i].iterkeys():
					child = components[:]
					child[i] = j
					if tuple(child) in visited:
						continue
					visited.add(tuple(child))
					res,soln = self.solveComponents(child)
					if res > bound:
						print "Solve child",child,"exceeded bound",res,">",bound
					else:
						heapq.heappush(queue,(res,child,soln))
				break
		return bound,best

def show_friction_cone(folder="FrictionCones/StanfordMicroSpineUnit"):
	V = Volume()
	V.load(folder)

	for C in V.convex_decomposition:
		print "Convex decomposition"
		print np.amin(C.points,axis=0)
		print np.amax(C.points,axis=0)
	Chull = np.vstack([V.convex_hull.points[u] for u in V.convex_hull.vertices])
	#print Chull
	Chull *= 0.05
	for i,C in enumerate(V.convex_decomposition):
		if len(V.convex_decomposition) == 1:
			g = 0
		else:
			g = float(i)/(len(V.convex_decomposition)-1)
		#for j,v in enumerate(C.vertices):
		#	vis.add("pt%d,%d"%(i,j),vectorops.mul(C.points[v,:].tolist(),0.05))
		#	vis.setColor("pt%d,%d"%(i,j),1,g,0)
		#	vis.hideLabel("pt%d,%d"%(i,j))
		for j,pt in enumerate(C.points):
			v = j
			vis.add("pt%d,%d"%(i,j),vectorops.mul(C.points[v,:].tolist(),0.05))
			vis.setColor("pt%d,%d"%(i,j),1,g,0)
			vis.hideLabel("pt%d,%d"%(i,j))
	for i in xrange(Chull.shape[0]):
		vis.add("CH%d"%(i,),Chull[i,:].tolist())
		vis.hideLabel("CH%d"%(i,))
	vis.add("origin",(0,0,0))
	vis.setColor("origin",0,1,0)
	vis.dialog()

class GLStabilityPlugin(GLWidgetPlugin):
	def __init__(self,folder="FrictionCones/StanfordMicroSpineUnit"):
		GLWidgetPlugin.__init__(self)
		self.factory = GeneralizedFrictionConeFactory()
		self.factory.load(folder)
		self.contacts = []
		self.widgets = self.klamptwidgetmaster
		self.cmWidget = PointPoser()
		self.cmWidget.set((0,0,0.5))
		self.contactWidgets = [TransformPoser()]
		self.widgets.add(self.cmWidget)
		self.widgets.add(self.contactWidgets[0])
		self.wrenches = None
		self.contactSize = 0.02
		self.feasible = None

	def display(self):
		GLWidgetPlugin.display(self)
		glEnable(GL_LIGHTING)
		if self.feasible == True:
			gldraw.setcolor(0,1,0,lighting=True)
		else:
			gldraw.setcolor(1,0,0,lighting=True)
		glPushMatrix()
		glTranslatef(*self.cmWidget.get())
		radius = 0.05
		gldraw.box([-radius]*3,[radius]*3)
		glPopMatrix()
		for w in self.contactWidgets:
			T = w.get()
			glPushMatrix()
			try:
				mat = zip(*se3.homogeneous(T))
				mat = sum([list(coli) for coli in mat],[])
				glMultMatrixf(mat)
				self.factory.forceTemplate.drawGL(self.contactSize)
			except:
				import traceback
				traceback.print_exc()
				exit(-1)
			glPopMatrix()
		if self.wrenches != None and len(self.wrenches) == len(self.contactWidgets):
			glDisable(GL_LIGHTING)
			glDisable(GL_DEPTH_TEST)
			glColor3f(1,0.5,0)
			glBegin(GL_LINES)
			for w,widget in zip(self.wrenches,self.contactWidgets):
				p = widget.get()[1]
				glVertex3fv(p)
				glVertex3fv(vectorops.madd(p,(w[0],w[1],w[2]),1.0/9.8))
			glEnd()
			glColor3f(0,0.5,1)
			glBegin(GL_LINES)
			for w,widget in zip(self.wrenches,self.contactWidgets):
				p = widget.get()[1]
				glVertex3fv(p)
				glVertex3fv(vectorops.madd(p,(w[3],w[4],w[5]),1.0/9.8))
			glEnd()
			glEnable(GL_DEPTH_TEST)

	def solve(self):
		solver = GeneralizedStabilitySolver(self.factory)
		for w in self.contactWidgets:
			print "Transform",w.get()
			solver.addContactPatch(w.get(),self.contactSize)
		t0 = time.time()
		print "Mass 4, CM",self.cmWidget.get()
		solver.setGravityWrench(4,self.cmWidget.get())
		self.wrenches = solver.solve()
		self.feasible = (self.wrenches != None)
		print "Feasible:",self.feasible
		if self.feasible:
			print self.wrenches
			assert len(self.wrenches) == len(self.contactWidgets)

	def transform(self,T):
		self.cmWidget.set(se3.apply(T,self.cmWidget.get()))
		for w in self.contactWidgets:
			w.set(*se3.mul(T,w.get()))

	def keyboardfunc(self,c,x,y):
		if c == '?':
			print "Help:"
			print "-?: display this message"
			print "-s: solve for stability"
			print "-a: add a new contact"
			print "-r: remove the currrently hovered contact"
			print "-<: rotate the entire setup ccw about the y axis"
			print "->: rotate the entire setup cw about the y axis"
		elif c == 's':
			self.solve()
		elif c == 'a':
			self.contactWidgets.append(TransformPoser())
			self.widgets.add(self.contactWidgets[-1])
			self.solve()
		elif c == 'r':
			removed = False
			for i,w in enumerate(self.contactWidgets):
				if w.hasHighlight():
					self.widgets.remove(w)
					self.contactWidgets.pop(i)
					removed = True
					break
			if not removed:
				print "Warning, no currently hovered widget"
			else:
				self.solve()
		elif c == '<':
			T = (so3.rotation([0,1,0],math.radians(-5)),[0]*3)
			self.transform(T)
			self.solve()
		elif c == '>':
			T = (so3.rotation([0,1,0],math.radians(5)),[0]*3)
			self.transform(T)
			self.solve()
		self.refresh()


def stability_vis():
	program = GLStabilityPlugin()
	vis.run(program)

if __name__=="__main__":
	#show_friction_cone()
	stability_vis()
	vis.kill()
