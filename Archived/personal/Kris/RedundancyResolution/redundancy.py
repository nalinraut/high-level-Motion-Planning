import pkg_resources
pkg_resources.require("klampt>=0.6.2")
if pkg_resources.get_distribution("klampt").version >= '0.7':
	#Klampt v0.7.x
	from klampt import *
	from klampt.model import ik
	from klampt.math import vectorops,so3,se3
	from klampt import vis
	from klampt.vis.glcommon import GLWidgetPlugin
	from klampt import PointPoser,TransformPoser
	from klampt.vis import gldraw
	from klampt.vis.glcommon import CachedGLObject
	GLBaseClass = GLWidgetPlugin
	KLAMPT_VERSION = 0.7
else:
	#Klampt v0.6.x
	from klampt import *
	from klampt.glrobotprogram import GLWidgetProgram
	from klampt import PointPoser,TransformPoser
	from klampt import gldraw
	from klampt.visualization import CachedGLObject
	GLBaseClass = GLWidgetProgram
	KLAMPT_VERSION = 0.6
from cgraph import *
from nearestneighbors import *
from metric import *
from disjointset import *
from collections import defaultdict
from ikdb.ikproblem import IKProblem,IKSolverParams
from ikdb.utils import mkdir_p
from csp import *
import scipy.spatial
import itertools
import numpy as np
import networkx as nx
from OpenGL.GL import *
import random
import math
import os
import time
import json
import sys

def linf_distance(a,b):
	d = 0
	for (v,w) in zip(a,b):
		d = max(d,abs(v-w)%math.pi)
	return d

def workspace_distance(a,b):
	if len(a) <= 3:
		return vectorops.distance(a,b)
	dt = vectorops.distance(a[:3],b[:3])
	dR = so3.distance(so3.from_moment(a[3:]),so3.from_moment(b[3:]))
	return dt + dR/math.pi

def workspace_interpolate(a,b,u):
	if len(a) <= 3:
		return vectorops.interpolate(a,b,u)
	Ra,Rb = so3.from_moment(a[3:]),so3.from_moment(b[3:])
	R = so3.interpolate(Ra,Rb,u)
	return vectorops.interpolate(a[:3],b[:3],u) + so3.moment(R)

def mmult(A,b):
	v = []
	for ai in A:
		assert (len(ai) == len(b))
		v.append(vectorops.dot(ai,b))
	return v

def segment_point_distance(a,b,x):
	"""Returns (distance, parameter) pair between segment (a,b) and point x"""
	d = vectorops.sub(b,a)
	u = vectorops.dot(vectorops.sub(x,a),d)/vectorops.dot(d,d)
	u = min(1.0,max(u,0.0))
	d = vectorops.distance(x,vectorops.interpolate(a,b,u))
	return (d,u)

def segment_segment_intersection(seg1,seg2,umin=0,umax=1,vmin=0,vmax=1):
	"""Given 2 2D segments (a1,b1) and (a2,b2) returns whether the portion of
	seg1 in the range [umin,umax] overlaps seg2 in the range [vmin,vmax].

	Returns the point of intersection or None if no intersection exists
	"""
	a1,b1 = seg1
	a2,b2 = seg2
	assert len(a1)==2
	assert len(b1)==2
	assert len(a2)==2
	assert len(b2)==2
	d = vectorops.sub(b1,a1)
	a,b = vectorops.sub(a2,a1),vectorops.sub(b2,a1)
	#now it's centered at a1
	#u*d = a + v*(b-a)
	e = vectorops.sub(b2,a2)
	A = np.array([d,vectorops.mul(e,-1)]).T
	try:
		uv = np.dot(np.linalg.inv(A),a)
	except np.linalg.linalg.LinAlgError:
		return None;
	u,v = uv
	if u < umin or u > umax:
		return None
	if v < umin or v > umax:
		return None
	return vectorops.interpolate(a1,b1,u)

def segment_triangle_intersection(seg,tri,umin=0,umax=1):
	"""Given a 3D segment (p,q) and a triangle (a,b,c), returns the point of
	intersection if the region of the segment in interval [umin,umax] intersects the
	triangle. Otherwise returns None"""
	p,q = seg
	a,b,c = tri
	d = vectorops.sub(b,a)
	e = vectorops.sub(c,a)
	n = vectorops.cross(d,e)
	if vectorops.norm(n) < 1e-7: #degenerate
		return None
	n = vectorops.unit(n)
	ofs = vectorops.dot(n,a)
	#Solve for n^T(p + u(q-p)) = ofs
	denom = vectorops.dot(n,vectorops.sub(q,p))
	numer = ofs - vectorops.dot(n,p)
	if abs(denom) < 1e-7: #near parallel
		return None
	u = numer / denom
	if u < umin or u > umax:
		return None
	#now check whether point of intersection lies in triangle
	x = vectorops.madd(p,vectorops.sub(q,p),u)
	xloc = vectorops.sub(x,a)

	#solve for coordinates [r,s,t] such that xloc = r*n + s*d + t*e
	try:
		rst = np.dot(np.linalg.inv(np.array([n,d,e]).T),xloc)
	except np.linalg.linalg.LinAlgError:
		return None;
	r,s,t = rst
	assert abs(r) < 1e-4
	if s >= 0 and t >= 0 and s + t <= 1:
		return x
	return None

def robot_average(robot,configs,weights=None):
	assert len(configs) > 0
	qres = configs[0]
	if weights == None:
		for (i,q) in enumerate(configs):
			if i == 0: continue
			qres = robot.interpolate(qres,q,1.0/(i+1))
	else:
		assert len(weights) == len(configs)
		sumw = weights[0]
		for (i,(q,w)) in enumerate(zip(configs,weights)):
			if i == 0: continue
			if w == 0: continue
			qres = robot.interpolate(qres,q,w/(sumw+w))
			sumw += w
	return qres

def set_cover(sets,universe=None):
	"""
	Performs greedy set-cover.

	In: 
	- sets: a dictionary mapping set names to lists of items in the universe
	- universe: a list or set of items, or None if this is to be determined automatically.

	Return value: a dictionary mapping set names to the items that are *first* covered
	using the greedy algorithm.
	If a set is not included in the set cover, then it is not included in the result.
	"""
	newsets = dict()
	for k,v in sets.iteritems():
		if isinstance(v,(list,tuple)):
			newsets[k] = set(v)
		else:
			newsets[k] = v
	sets = newsets
	if universe == None:
		#auto-determine universe
		universe = set()
		for k,v in sets.iteritems():
			universe |= v
	else:
		universe = set(universe)
	#precompute element coverse
	covers = {}
	for e in universe:
		covers[e] = []
	for k,v in sets.iteritems():
		for e in v:
			assert e in covers,"Set "+str(k)+" contains element not in universe"
			covers[e].append(k)
	#compute sizes
	sizes = {}
	for k,v in sets.iteritems():
		sizes[k] = len(v)
	result = {}
	while len(universe) > 0:
		best = max((v,k) for (k,v) in sizes.iteritems())[1]
		covered = universe & sets[best]
		result[best] = covered
		del sizes[best]
		for e in covered:
			for s in covers[e]:
				if s in sizes:
					sizes[s] -= 1
		universe = universe - sets[best]
	return result

def nearest_point(G,nodes,weight=None,p=1):
	"""Finds the node closest to the given nodes on the graph
	according to the L_p metric.  If no node is reachable by all the
	nodes, this returns the one that is nearest to the maximum number
	of nodes.
	"""
	acc_lengths = defaultdict(float)
	acc_reachable = defaultdict(int)
	for n in nodes:
		L = nx.shortest_path_length(G,source=n,weight=weight)
		for m,v in L.iteritems():
			acc_reachable[m] += 1
			if p == 1:
				acc_lengths[m] += v
			elif p == float('inf'):
				acc_lengths[m] = max(acc_lengths[m],v)
			else:
				acc_lengths[m] += pow(v,p)
	allreachable = False
	numreachable = 0
	best = None
	for (n,r) in acc_reachable.iteritems():
		if r == len(nodes):
			numreachable = r
			allreachable = True
			break
		if r > numreachable:
			numreachable = r
			best = n
	if allreachable:
		#find the optimal node that is reachable by all specified nodes
		minlen = float('inf')
		best = None
		for (n,r) in acc_reachable.iteritems():
			if r != numreachable: 
				continue
			if acc_lengths[n] < minlen:
				minlen = acc_lengths[n]
				best = n
		assert best is not None
		return best
	else:
		raise NotImplementedError("TODO: nearest_point when graph is not connected")

def ring(G,S,k=1):
	"""Computes the k-ring of the set of node S in the graph G.  Given an undirected graph
	and set of nodes S, the 1-ring of S in G is the set of nodes neighboring one node in S,
	not including S.  The k-ring is defined recursively as the 1-ring of the (k-1)-ring.

	The return value is a set. It also runs fastest when S is a set.
	"""
	if k>1:
		return ring(G,ring(G,S,k-1))
	res = set()
	for v in S:
		res |= set(G.neighbors(v))
	return res - set(S)

class ProgressUpdater:
	def __init__(self,n=None,frequency=1):
		self.cnt = 0
		self.n = n
		self.frequency = frequency
	def update(self):
		self.cnt += 1
		if self.n == None:
			if self.cnt % self.frequency == 0:
				print '  ...'
		else:
			if (100*self.cnt) / (self.n*self.frequency) != (100*(self.cnt-1)) / (self.n*self.frequency):
				print "  %d%%.."%((100*self.cnt) / self.n,),
				if self.cnt == self.n: print
				else: sys.stdout.flush()
	def done(self):
		pass

class RedunancyResolver:
	def __init__(self,robot):
		self.robot = robot
		self.domain = None
		self.dims = None
#		self.Gw = nx.Graph(qnodes=0)
		self.Gw = CGraph(cachesize=20)
		#self.Gq = nx.Graph()
		self.nnw = NearestNeighbors(L2Metric,'kdtree')
		#self.nnq = NearestNeighbors(L2Metric,'kdtree')
		#self.infeasibleEdges = set()
		self.ikTemplate = IKProblem()
		self.ikTemplate.setJointLimits()
		self.ikSolverParams = IKSolverParams()
		self.ikSolverParams.startRandom = True
		self.wRadius = None
		self.Qsubgraph = set()

	def save(self,folder):
		mkdir_p(folder)
		nx.write_gpickle(self.Gw,os.path.join(folder,"Gw.pickle"))
		nx.write_gpickle(self.Gq,os.path.join(folder,"Gq.pickle"))
		if len(self.Qsubgraph) > 0:
			f = open(os.path.join(folder,"Qsubgraph.txt"),"w")
			for i in sorted([v for v in self.Qsubgraph]):
				f.write(str(i)+" ")
			f.write("\n")
			f.close()
		return

	def load(self,folder):
		self.Gw = nx.read_gpickle(os.path.join(folder,"Gw.pickle"))
		self.Gq = nx.read_gpickle(os.path.join(folder,"Gq.pickle"))
		try:
			f = open(os.path.join(folder,"Qsubgraph.txt"),"r")
			items = f.readline().split()
			self.Qsubgraph = set(int(v) for v in items)
			f.close()
		except Exception as e:
			print e
			print "Warning, Qsubgraph could not be loaded"
			#print "Making assignment..."
			#self.uniqueAssignment()
			pass
		#sanity check
		for i,d in self.Gw.nodes(data=True):
			for iq in d['qlist']:
				assert iq >= 0 and iq < len(self.Gq.node)
				assert self.Gq.node[iq]['windex'] == i
		for iq,d in self.Gq.nodes(data=True):
			iw = self.Gq.node[iq]['windex']
			assert iw >= 0 and iw < len(self.Gw.node)
			if iq not in self.Gw.node[iw]['qlist']:
				print "C-space node %d is mapped incorrectly to workspace node %d, workspace node contains %s"%(iq,iw,' '.join([str(v) for v in self.Gw.node[iw]['qlist']]))
				self.Gw.node[iw]['qlist'].append(iq)
				#raw_input()
			assert iq in self.Gw.node[iw]['qlist'],"C-space node %d is mapped incorrectly to workspace node %d, workspace node contains %s"%(iq,iw,' '.join([str(v) for v in self.Gw.node[iw]['qlist']]))
		for i,j,d in self.Gw.edges_iter(data=True):
			for iq,jq in d['qlist']:
				if not ((iq in self.Gw.node[i]['qlist'] and jq in self.Gw.node[j]['qlist']) or (iq in self.Gw.node[j]['qlist'] and jq in self.Gw.node[i]['qlist'])):
					print "C-space ndge",iq,jq,"not in workspace node configuration lists",i,j
					print self.Gw.node[i]['qlist']
					print self.Gw.node[j]['qlist']
				assert (iq in self.Gw.node[i]['qlist'] and jq in self.Gw.node[j]['qlist']) or (iq in self.Gw.node[j]['qlist'] and jq in self.Gw.node[i]['qlist'])
		#setup domain
		bmin = self.Gw.node[0]['params'][:]
		bmax = self.Gw.node[0]['params'][:]
		for i,d in self.Gw.nodes(data=True):
			x = d['params']
			for j,v in enumerate(x):
				bmin[j] = min(bmin[j],v)
				bmax[j] = max(bmax[j],v)
		self.domain = (bmin,bmax)
		#setup nearest neighbors
		self.nnw = NearestNeighbors(L2Metric,'kdtree')
		for n,d in self.Gw.nodes(data=True):
			self.nnw.add(d['params'],n)

	def printStats(self):
		print "Roadmap has",self.Gw.number_of_nodes(),"workspace nodes and",self.Gw.number_of_edges(),"edges"
		print "  and",self.Gw.graph['qnodes'],"configuration space nodes and",self.Gq.number_of_edges(),"edges"
		if len(self.Qsubgraph) == 0:
			print "  Resolution is empty."
			return
		numNodes = 0
		for i,d in self.Gw.nodes(data=True):
			if len(d['qlist']) > 0:
				numNodes += 1
		print "  of %d nodes, %d have no configuration, and %d / %d are not in redundancy resolution"%(self.Gw.number_of_nodes(),self.Gw.number_of_nodes()-numNodes,numNodes-len(self.Qsubgraph),self.Gw.number_of_nodes()-numNodes)
		numPotentialEdges = 0
		numEdgesWithConfigurations = 0
		numEdgesInResolution = 0
		sumdistances = 0
		sumwsdistances = 0
		for i,j,d in self.Gw.edges_iter(data=True):
			if len(self.Gw.node[i]['qlist']) > 0 and len(self.Gw.node[j]['qlist'])>0:
				numPotentialEdges += 1
			if len(d['qlist'])>0:
				numEdgesWithConfigurations += 1
				qedges = [(iq,jq)  for (iq,jq) in d['qlist'] if iq in self.Qsubgraph and jq in self.Qsubgraph]
				if len(qedges) > 0:
					numEdgesInResolution += 1
					iq,jq = qedges[0]
					sumdistances += self.robot.distance(self.Gq.node[iq]['config'],self.Gq.node[jq]['config'])
					sumwsdistances += workspace_distance(self.Gw.node[i]['params'],self.Gw.node[j]['params'])
		print "  of %d edges, %d  are missing a configuration space edge, and %d / %d are not in redundancy resolution"%(numPotentialEdges,numPotentialEdges-numEdgesWithConfigurations,numPotentialEdges-numEdgesInResolution,numPotentialEdges)
		if numEdgesInResolution != 0:
			print "  Average configuration space distance:",sumdistances / numEdgesInResolution
			print "  Average configuration space distance / workspace distance:",sumdistances / sumwsdistances

	def clearSamples(self):
		"""Reinitializes workspace and configuration space graphs."""
		self.Gw = nx.Graph()
		self.Gq = nx.Graph()
		self.nnw = NearestNeighbors(L2Metric,'kdtree')
		#self.nnq = NearestNeighbors(L2Metric,'kdtree')
		self.Qsubgraph = set()

	def addIKConstraint(self,link,eelocal,orientation='free'):
		"""Orientation can be free, variable, or a so3 element"""
		if orientation == 'free':
			self.ikTemplate.addObjective(ik.objective(link,local=eelocal,world=[0,0,0]))
		elif orientation == 'variable':
			T = se3.identity()
			obj = ik.objective(link,R=T[0],t=T[1])
			obj.setFixedPosConstraint(eelocal,[0,0,0])
			self.ikTemplate.addObjective(obj)
		else:
			assert isinstance(orientation,(list,tuple))
			#assume it's a fixed orientation matrix
			obj = ik.objective(link,R=orientation,t=[0,0,0])
			obj.setFixedPosConstraint(eelocal,[0,0,0])
			self.ikTemplate.addObjective(obj)

	def setIKProblem(self,x):
		"""Given a workspace setting x, sets the self.ikTemplate problem so that
		it corresponds with the workspace parameters x"""
		n = 0
		for obj in self.ikTemplate.objectives:
			np = obj.numPosDims()
			nr = obj.numRotDims()
			assert n + np <= len(x)
			assert np == 3 or np == 0,"Can only handle fixed or free position constraints"
			assert nr == 3 or nr == 0,"Can only handle fixed or free rotation constraints"
			if np == 3:
				local,world = obj.getPosition()
				obj.setFixedPosConstraint(local,x[n:n+3])
			n += np
			if nr == 3 and len(x) >= 3 + n:
				assert n + nr <= len(x)
				R = obj.getRotation()
				R = so3.from_moment(x[n:n+3])
				obj.setFixedRotConstraint(R)
				n += nr
		assert n == len(x),"Workspace parameters must be exactly the right number of variables in template IK constraint"

	def getIKParameters(self,q):
		"""Given a configuration q, gets the workspace parameters that matches q exactly."""
		x = []
		qOrig = self.robot.getConfig()
		self.robot.setConfig(q)
		for obj in self.ikTemplate.objectives:
			link = self.robot.link(obj.link())
			np = obj.numPosDims()
			nr = obj.numRotDims()
			assert np == 3 or np == 0,"Can only handle fixed or free position constraints"
			assert nr == 3 or nr == 0,"Can only handle fixed or free rotation constraints"
			if np == 3:
				local,world = obj.getPosition()
				x += link.getWorldPosition(local)
			#TODO: handle rotations
		self.robot.setConfig(qOrig)
		return x

	def solve(self,qinit,x):
		"""Solves the IK problem at x with the starting config qinit.  Returns None if
		failed, or the solution if successful."""
		self.setIKProblem(x)
		self.robot.setConfig(qinit)
		self.ikSolverParams.startRandom = False
		return self.ikTemplate.solve(self.robot,self.ikSolverParams)

	def addWorkspaceNode(self,x):
		"""Adds a node to the workspace graph with params x.  Returns the index"""
		iw = self.Gw.number_of_nodes()
		self.Gw.add_node(iw,params=x,configs=[],qccs=DisjointSet())
		self.nnw.add(x,iw)
		return iw

	def addWorkspaceEdge(self,i,j):
		"""Adds an edge to the workspace graph"""
		if i > j:
			self.addWorkspaceEdge(j,i)
			return
		assert i < self.Gw.number_of_nodes()
		assert j < self.Gw.number_of_nodes()
		self.Gw.add_edge(i,j,qlist=[])

	def addNode(self,iw,q):
		"""Adds a configuration node q corresponding to workspace node index iw.  Returns the index of the
		configuration space node"""
		assert iw < self.Gw.number_of_nodes()
		iq = len(self.Gw.node[iw]['configs'])
		self.Gw.node[iw]['configs'].append(q)
		self.Gw.graph['qnodes'] = self.Gw.graph['qnodes'] + 1
		if 'qccs' in self.Gw.node[iw]:
			self.Gw.node[iw]['qccs'].add(iq)
		return iq

	def addEdge(self,iw,iq,jw,jq):
		"""Adds a configuration space edge corresponding to workspace/configuration space node index (iw,iq) to
		index (jw,jq).
		"""
		if iw > jw:
			self.addEdge(jw,jq,iw,iq)
			return
		assert iw < self.Gw.number_of_nodes()
		assert jw < self.Gw.number_of_nodes()
		assert iq < len(self.Gw.node[iw]['configs'])
		assert jq < len(self.Gw.node[jw]['configs'])
		#TODO store list of self-motion edges?
		if iw == jw:
			self.Gw.node[iw]['qccs'].merge(iq,jq)
		else:
			if (iq,jq) in self.Gw.edge[iw][jw]['qlist']:
				assert False, "edge already exists"
			self.Gw.edge[iw][jw]['qlist'].append((iq,jq))

	def removeWorkspaceEdge(self,iw,jw):
		assert iw != jw
		if iw > jw:
			self.removeWorkspaceEdge(jw,iw)
			return
		assert iw < self.Gw.number_of_nodes()
		assert jw < self.Gw.number_of_nodes()
		self.Gw.remove_edge(iw,jw)
			
	def removeInterWnodeEdge(self,iw,iq,jw,jq):
		assert iw != jw
		if iw > jw:
			self.removeInterWnodeEdge(jw,jq,iw,iq)
			return
		assert iw < self.Gw.number_of_nodes()
		assert jw < self.Gw.number_of_nodes()
		assert iq < len(self.Gw.node[iw]['configs'])
		assert jq < len(self.Gw.node[jw]['configs'])
		assert (iq,jq) in self.Gw.edge[iw][jw]['qlist']
		self.Gw.edge[iw][jw]['qlist'].remove((iq,jq))

	def testAndConnectAll(self,iw,jw):
		"""Tries to connect two workspace nodes in configuration space by testing all pairs of configurations."""
		res = False
		for na in range(len(self.Gw.node[iw]['configs'])):
			for nb in range(len(self.Gw.node[jw]['configs'])):
				if self.testAndConnect(iw,na,jw,nb):
					res = True
		return res

	def testAndConnect(self,iw,iq,jw,jq):
		"""Tries to connect two configurations in the configuration map"""
		if iw > jw:
			return self.testAndConnect(jw,jq,iw,iq)
		assert iq < len(self.Gw.node[iw]['configs'])
		assert jq < len(self.Gw.node[jw]['configs'])
		if iw != jw and (iq,jq) in self.Gw.edge[iw][jw]['qlist']:#or (iq,jq) in self.infeasibleEdges:
			#already tested
			return True 
		a = self.Gw.node[iw]['configs'][iq]
		b = self.Gw.node[jw]['configs'][jq]
		if self.validEdge(a,b,self.Gw.node[iw]['params'],self.Gw.node[jw]['params']):
			#add it
			self.addEdge(iw,iq,jw,jq)
			return True
		#else:
			#self.infeasibleEdges.add((iq,jq))
			#self.infeasibleEdges.add((jq,iq))
		return False

	def testAndAddEdge(self,iw,iq,jw,q):
		"""Tries to connect an existing configuration to a new one.  If the connection
		is feasible, then the node is added and connected to iw,iq.  The index of
		the new node is returned.  If it's infeasible, None is returned."""
		assert iq < len(self.Gw.node[iw]['configs'])
		assert jw < self.Gw.number_of_nodes()
		a = self.Gw.node[iw]['configs'][iq]
		if self.validEdge(a,q,self.Gw.node[iw]['params'],self.Gw.node[jw]['params']):
			#add it
			jq = self.addNode(jw,q)
			self.addEdge(iw,iq,jw,jq)
			return jq
		return None

	def testAndConnectQccs(self,iw,iq,jw,jq):
		"""Tries to link two configuration-space connected components of different workspace points"""
		assert iw != jw
		assert iq < len(self.Gw.node[iw]['configs'])
		assert jq < len(self.Gw.node[jw]['configs'])
		iqccs = self.Gw.node[iw]['qccs']
		jqccs = self.Gw.node[jw]['qccs']
		distances = [(self.robot.distance(self.Gw.node[iw]['configs'][i],self.Gw.node[jw]['configs'][j]),i,j) for i in iqccs.iterate(iq) for j in jqccs.iterate(jq)]
		distances = sorted(distances)
		cnt = 0
		for dist,i,j in distances:
			cnt+=1
			if self.testAndConnect(iw,i,jw,j):
				return True
			if cnt > 10:
				break
		if len(distances) > 10:
			#random selection
			for (dist,i,j) in random.sample(distances[10:],min(10,len(distances)-10)):
				cnt+=1
				if self.testAndConnect(iw,i,jw,j):
					print "CC connection success with random connection"
					return True
		#print "CC connection failed after",cnt,"tests"
		return False

	def connectWorkspaceNeighbors(self,iw,radius=None,k=None):
		x = self.Gw.node[iw]['params']
		if radius != None:
			knn = self.nnw.neighbors(x,radius)
			if len(x) > 3:
				#rotational, check connections on opposite side of unit-sphere
				rx = x[3:]
				assert len(rx) <= 3
				if vectorops.norm(rx) > 0.01:
					rxnew = vectorops.madd(rx,vectorops.unit(rx),-2*math.pi)
					xnew = x[:3]+rxnew
					knn += self.nnw.neighbors(xnew,radius)
		else:
			assert k != None,"Need either k or radius to be specified"
			knn = self.nnw.knearest(x,k)
			if len(x) > 3:
				#rotational, check connections on opposite side of unit-sphere
				rx = x[3:]
				assert len(rx) <= 3
				if vectorops.norm(rx) > 0.01:
					rxnew = vectorops.madd(rx,vectorops.unit(rx),-2*math.pi)
					xnew = x[:3]+rxnew
					knn += self.nnw.knearest(xnew,k)
				#now subselect only the k nearest in terms of workspace distance
				sortlist = [(workspace_distance(pt,x),pt,n) for pt,n in knn]
				knn = [(pt,n) for (d,pt,n) in sorted(sortlist)]
				knn = knn[:k]
		for pt,n in knn:
			if n == iw:
				continue
			self.addWorkspaceEdge(iw,n)


	def sampleWorkspace(self,bmin,bmax,N,k='auto',method='random'):
		print "Sampling",N,"workspace points with method",method
		self.domain = (bmin,bmax)
		dims = len([1 for (a,b) in zip(bmin,bmax) if a!=b])
		self.dims = dims
		if method == 'random':
			for i in xrange(N):
				x = [random.uniform(a,b) for (a,b) in zip(bmin,bmax)]
				if len(bmin) == 6:
					#TODO: sample rotation uniformly?
					pass
				self.addWorkspaceNode(x)
			if k == 'auto':
				k = int((1+1.0/dims)*math.e*math.log(N))
				#k = 4*dims
		elif method == "grid":
			vol = 1
			for (a,b) in zip(bmin,bmax):
				if b > a:
					vol *= (b-a)
			cellvol = float(vol)/N
			celldim = math.pow(cellvol,1.0/dims)
			assert celldim > 0
			divs = [max(int(math.ceil((b-a)/celldim)),1) for (a,b) in zip(bmin,bmax)]
			import itertools
			for cell in itertools.product(*[range(n) for n in divs]):
				params = [float(i)/float(div) for i,div in zip(cell,divs)]
				x = [a+u*(b-a) for (a,b,u) in zip(bmin,bmax,params)]
				self.addWorkspaceNode(x)
			print "Grid discretization added",self.Gw.number_of_nodes(),"workspace nodes"
			if k == 'auto':
				celldim = max((b-a)/div for (a,b,div) in zip(bmin,bmax,divs))
				for i in self.Gw.nodes():
					self.connectWorkspaceNeighbors(i,radius=celldim*1.05)
				return
		elif method == "staggered_grid":
			vol = 1
			for (a,b) in zip(bmin,bmax):
				if b > a:
					vol *= (b-a)
			cellvol = 2*float(vol)/N
			celldim = math.pow(cellvol,1.0/dims)
			assert celldim > 0
			divs = [max(int(math.ceil((b-a)/celldim)),1) for (a,b) in zip(bmin,bmax)]
			active = [(1 if b > a else 0) for (a,b) in zip(bmin,bmax)]
			centers = []
			import itertools
			for cell in itertools.product(*[range(n) for n in divs]):
				params = [float(i)/float(div) for i,div in zip(cell,divs)]
				x = [a+u*(b-a) for (a,b,u) in zip(bmin,bmax,params)]
				self.addWorkspaceNode(x)
				centers.append(False)
				if all(i+a < div for (i,div,a) in zip(cell,divs,active)):
					params = [(float(i)+0.5*a)/float(div) for i,div,a in zip(cell,divs,active)]
					x = [a+u*(b-a) for (a,b,u) in zip(bmin,bmax,params)]
					self.addWorkspaceNode(x)
					centers.append(True)
			print "Grid discretization added",self.Gw.number_of_nodes(),"workspace nodes"
			if k == 'auto':
				print [(b-a)/div for (a,b,div) in zip(bmin,bmax,divs)]
				celldim = max((b-a)/div for (a,b,div) in zip(bmin,bmax,divs))
				print "Connecting each workspace grid node within radius",celldim*math.sqrt(dims)*1.01
				for i in self.Gw.nodes():
					if centers[i]:
						self.connectWorkspaceNeighbors(i,radius=celldim*math.sqrt(dims)*0.5*1.01)
					else:
						self.connectWorkspaceNeighbors(i,radius=celldim*1.01)
				return
		else:
			raise ValueError("Unknown method "+method)
		c = ProgressUpdater(self.Gw.number_of_nodes(),(1 if len(bmin) > 3 else 10))
		for i,d in self.Gw.nodes(data=True):
			c.update()
			self.connectWorkspaceNeighbors(i,k=k+1)
		c.done()
		return

	def getSelfMotionCcsStats(self):
		"""Returns statistics describing the connected components in the self-motion graph
		of each workspace node."""
		sumConfigs = 0        
		sumNumQccs = 0
		sumMaxSize = 0
		sumAvgSize = 0.0
		sumMinSize = 0
		configuredW = 0
		for w,d in self.Gw.nodes(data=True):
			if len(d['qccs']) > 0:
				configuredW += 1
				sumNumQccs += len(d['qccs'])
				maxSize = 0
				sumSizes = 0
				minSize = float('inf')
				for rep in d['qccs']:
					size = d['qccs'].length(rep)
					if size > maxSize:
						maxSize = size
					sumSizes += size
					if size < minSize:
						minSize = size
				sumConfigs += sumSizes
				sumMaxSize += maxSize
				sumAvgSize += float(sumSizes)/float(len(d['qccs']))
				sumMinSize += minSize
		if configuredW == 0:
			return float('inf'),float('inf'),float('inf'),float('inf'),float('inf')
		return float(sumConfigs)/float(configuredW),float(sumNumQccs)/float(configuredW),float(sumMaxSize)/float(configuredW),float(sumAvgSize)/float(configuredW),float(sumMinSize)/float(configuredW)
		
	def constructSelfMotionManifolds(self,k=None):
		"""Create self-motion C-space edges to minimize number of C-space connected
		components for all workspace points.

		If k is provided, does k-nearest neighbor connections.  Otherwise, tries all connections
		"""
		print "Constructing self-motion connected components"
		c = ProgressUpdater(self.Gw.number_of_nodes(),5)
		start = time.clock()
		for w,d in self.Gw.nodes(data=True):
			c.update()
			#construct visibility prm
			wqnodes = len(d['configs'])
			wqccs = d['qccs']
			if k is not None:
				nnq = NearestNeighbors(L2Metric,'kdtree')
				for iq in range(wqnodes):
					nnq.add(d['configs'][iq],iq)
				for iq in range(wqnodes):
					knn = nnq.knearest(d['configs'][iq],k)
					for q,jq in knn:
						if not wqccs.same(iq,jq):
							self.testAndConnect(w,iq,w,jq)
			else:
				#visibility prm with all-pairs connections 
				for iq in range(wqnodes):
					for jq in xrange(iq):
						if not wqccs.same(iq,jq):
							self.testAndConnect(w,iq,w,jq)
		
		end = time.clock()
		avgNumConfigs, avgNumQccs, avgMaxSize, avgAvgSize, avgMinSize = self.getSelfMotionCcsStats()
		print "CC statistics: %f configs, %f components, size max %f avg %f min %f"%(avgNumConfigs, avgNumQccs, avgMaxSize, avgAvgSize, avgMinSize)
		return end - start
		
		
	def connectConfigurationSpace(self,visibility=False,k=None,start_node_count=0):
		self_motion_time = self.constructSelfMotionManifolds(k=k) if visibility else 0
				
		#create C-space edges for each workspace edge as long as the edge is feasible and
		#the movement monotonically decreases the distance
		print "Testing configuration space edges"
		numNondegenerateEdges = 0
		numPotentialEdges = 0
		numActualizedEdges = 0
		c = ProgressUpdater(self.Gw.number_of_edges(),5)
		start = time.clock()
		if visibility:
			for (iw,jw,d) in self.Gw.edges_iter(data=True):
				numPotentialEdges += len(self.Gw.node[jw]['qccs'])*len(self.Gw.node[iw]['qccs'])
			print "Potentially " + str(numPotentialEdges) + " CC edges"
			print "(Non-visibility graph method has " + u"\u2248" + str(int(self.Gw.number_of_edges()*((float(self.Gw.graph['qnodes'])/float(self.Gw.number_of_nodes()))**2))) + " edges)"
			for (iw,jw,d) in self.Gw.edges_iter(data=True):
				c.update()
				connected = False
				for iq in self.Gw.node[iw]['qccs']:
					for jq in self.Gw.node[jw]['qccs']:
						if self.testAndConnectQccs(iw,iq,jw,jq):
							connected = True
							numActualizedEdges += 1
				if connected:
					numNondegenerateEdges += 1
		else:
			print "Potentially " + u"\u2248" + str(int(self.Gw.number_of_edges()*((float(self.Gw.graph['qnodes'])/float(self.Gw.number_of_nodes()))**2))) + " edges"
			for (i,j,d) in self.Gw.edges_iter(data=True):
				c.update()
				for na in range(len(self.Gw.node[i]['configs'])):
					for nb in range(len(self.Gw.node[j]['configs'])):
						if na < start_node_count and nb < start_node_count:
							#existed previously, don't check it
							#print "Not checking",na,nb,"start",start_node_count
							continue
						self.testAndConnect(i,na,j,nb)
						numPotentialEdges += 1
				if len(d['qlist']) > 0:
					numNondegenerateEdges += 1
		end = time.clock()
		inter_wnode_time = end - start
		c.done()
		#check for isolated nodes
		print numNondegenerateEdges,"of",self.Gw.number_of_edges(),"workspace edges are nondegenerate"
		if visibility:
			print "The self-motion graph of each workspace node has an average number of",round(self.getSelfMotionCcsStats()[1],2),"connected components"
			print numActualizedEdges,"edges out of possible",numPotentialEdges
		else:
#			ccs = list(nx.connected_components(self.Gq))
#			print "Configuration space graph has",self.Gw.graph['qnodes'],"nodes and",len(ccs),"connected components"
#			print "CC sizes:"
#			for i in range(len(ccs)):
#				if len(ccs[i]) == 1:
#					print 
#					print len(ccs)-i,"isolated nodes"
#					break
#				print "  ",len(ccs[i]),
#			print
#			print self.Gq.number_of_edges(),"edges out of possible",numPotentialEdges
			pass
		return numNondegenerateEdges, self.Gw.number_of_edges(), self_motion_time, inter_wnode_time

	def sampleConfigurationSpace(self,NConfigsPerPoint=100,connect=True,biased=False):
		print "Sampling configurations for all workspace points"
		start_node_count = self.Gw.graph['qnodes']
		number_of_nodes_with_configurations = len([j for j,d in self.Gw.nodes(data=True) if len(d['configs']) > 0])
		c = ProgressUpdater(self.Gw.number_of_nodes(),5)
		start = time.clock()
		for i,d in self.Gw.nodes(data=True):
			c.update()
			adjacentWithConfigs = [j for j in self.Gw.neighbors(i) if j < i and len(self.Gw.node[j]['configs']) > 0]
			fracAdjacent = float(len(adjacentWithConfigs)) / float(len(self.Gw.neighbors(i)))
			if start_node_count != 0:
				fracAdjacent = 0
			numAdjacentSamples = int(fracAdjacent*NConfigsPerPoint)
			numRandomSamples = NConfigsPerPoint - numAdjacentSamples 
			if biased and start_node_count != 0:
				#numconfigs * average # of configs per workspace point / # of configs for this point
				if len(d['configs']) == 0:
					numRandomSamples = 0
				else:
					numRandomSamples = NConfigsPerPoint * self.Gw.graph['qnodes'] / (number_of_nodes_with_configurations * len(d['configs']))
			#print fracAdjacent,"adjacent"

			self.setIKProblem(d['params'])

			#try starting from existing neighboring configurations
			self.ikSolverParams.startRandom = False
			numFailed = 0
			if seedFromAdjacent:
				for it in xrange(numAdjacentSamples):
					j = random.choice(adjacentWithConfigs)
					qstart = random.choice(self.Gw.node[j]['configs'])
					self.robot.setConfig(qstart)
					q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
					if q is not None:
						#this checking is useful for reducing # of edge checks esp for nonredundant robots
						same = False
						for jq in d['configs']:
							if self.robot.distance(q,jq) < 1e-2:
								same = True
								break
						if not same:
							self.addNode(i,q)
						else:
							#print "Same config"
							pass
					else:
						numFailed += 1

			self.ikSolverParams.startRandom = True
			for it in xrange(numRandomSamples+numFailed):
				#solver will sample randomly
				q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
				if q is not None:
					#this checking is useful for reducing # of edge checks esp for nonredundant robots
					same = False
					for iq in d['qlist']:
						if self.robot.distance(q,self.Gq.node[iq]['config']) < 1e-2:
							same = True
							break
					if not same:
						self.addNode(i,q)
					else:
						#print "Same config"
						pass
		end = time.clock()
		sampling_time = end - start
		c.done()
		print "Sampled",self.Gw.graph['qnodes'],"of possible",self.Gw.number_of_nodes()*NConfigsPerPoint

		if connect:
			self.connectConfigurationSpace()
		
		return sampling_time

	def makeCSP(self,visibility=False):
		"""Creates a CSP whereby each workspace node is a variable and configs are values.
		Each neighboring node must satisfy the constraint that its workspace edge corresponds to
		a feasible configuration space edge.
		"""
		csp = CSP()
		for n,d in self.Gw.nodes(data=True):
			if len(d['configs']) > 0:
				if visibility:
					csp.addVariable(list(d['qccs'].getReps()),name="q_"+str(n))
				else:
					csp.addVariable([i for i in range(len(d['configs']))],name="q_"+str(n))
		for a,b,d in self.Gw.edges_iter(data=True):
			if (a < b and len(d['qlist']) > 0):
				validValues = set()
				for (iq,jq) in d['qlist']:
					if not iq < len(self.Gw.node[a]['configs']) or not jq < len(self.Gw.node[b]['configs']):
						assert False, "edge source and edge target are mismatched"
					if visibility:
						irep = self.Gw.node[a]['qccs'].getSetRep(iq)
						jrep = self.Gw.node[b]['qccs'].getSetRep(jq)
						assert irep < len(self.Gw.node[a]['configs'])
						assert jrep < len(self.Gw.node[b]['configs'])
						validValues.add((irep,jrep))
					else:
						try:
							assert iq < len(self.Gw.node[a]['configs'])
							assert jq < len(self.Gw.node[b]['configs'])
							validValues.add((iq,jq))
						except AssertionError:
							print "Warning, edge configuration value",iq,jq,"is not valid index in nodes"
							print "Start node config indices range to",len(self.Gw.node[a]['configs'])
							print "End node config indices range to",len(self.Gw.node[b]['configs'])
				csp.addConstraint(validValues,("q_"+str(a),"q_"+str(b)))
		return csp

	def collapseQccSubgraph(self):
		self.Qsubgraph = set()
		totalAnchors = 0
		totalNeighbors = 0
		totalCollapsed = 0
		triedToCollapse = 0
		wToQccRep = dict()
		for rep in self.QccSubgraph:
			iw = self.Gq.node[rep]['windex']
			wToQccRep[iw] = rep
		for n,d in self.Gq.nodes(data=True):
			assert d['windex'] in wToQccRep,"Workspace node %d not covered by QccSubgraph?"%(d['windex'],)

		assignedWEdges = set()
		resolvedAnchors = dict()
		resolutionFailures = set()
		order = sorted([self.Gq.node[rep]['windex'],rep] for rep in self.QccSubgraph)
		for iw,rep in order:
			#the set of configs in same self-motion manifold (SMM)
			manifold = self.Gw.node[iw]['qccs'].getSetRef(rep)
			#the dict mapping configs in SMM to lists of their connections at neighboring workspace points
			anchors = dict()
			#the set of all attached configs at neighboring workspace points
			neighbors = set()
			#the dictionary mapping workspace neighbors to the configuration space edge connecting them
			wneighbors = dict()
			for iq in manifold:
				touched = []
				#get all touched neighbors of this workspace node
				for jq in self.Gq.neighbors(iq):
					jw = self.Gq.node[jq]['windex']
					if iw == jw: continue
					assert self.Gw.has_edge(iw,jw)
					if self.Gw.node[jw]['qccs'].getSetRep(jq) == wToQccRep[jw]:
						if jw in wneighbors:
							print "Edge to workspace node %d duplicated, C-space edges (%d,%d) and (%d,%d) exist"%(jw,iq,jq,wneighbors[jw][0],wneighbors[jw][1])
							continue
						wneighbors[jw] = (iq,jq)
						assignedWEdges.add((iw,jw))
						assignedWEdges.add((jw,iw))
						touched.append(jq)
				if len(touched) > 0:
					assert iq not in anchors,"hm... duplicate node in Qcc?"
					anchors[iq] = touched
					neighbors.update(touched)
			
			totalAnchors += len(anchors)
			totalNeighbors += len(neighbors)
			
			#if we add new edges, we'll take them out before this is complete so that only edges between anchors are kept.
			newEdges = set()

			#try to find a hub connecting edges to all adjacent workspace nodes, if possible
			if len(anchors) > 1:
				#print "Anchors for",iw,":",anchors.keys()
				for iq in anchors:
					isHub = True
					for jq in neighbors:
						#assert self.Gq.has_edge(jq,iq) == self.Gq.has_edge(iq,jq)
						if not self.Gq.has_edge(iq,jq):
							assert self.Gw.has_edge(iw,self.Gq.node[jq]['windex'])
							if self.testAndConnect(iw, iq, self.Gq.node[jq]['windex'], jq):
								newEdges.add((iq,jq))
								anchors[iq].append(jq)
							else:
								isHub = False							
								break
					if isHub:
						#quick exit: there exists a hub, just use that
						toPrune = [iq2 for iq2 in anchors if iq != iq2]					
						for iq2 in toPrune:
							#remove jq's inter wnode edges
							cut = [kq for kq in self.Gq.neighbors(iq2) if self.Gq.node[kq]['windex'] != iw]
							for kq in cut:
								self.removeInterWnodeEdge(iw, iq2, self.Gq.node[kq]['windex'], kq)
						anchors = {iq:list(neighbors)}
						totalCollapsed += 1
						#print "  Anchor",iq,"is a hub to",list(neighbors),"on nodes",[self.Gq.node[jq]['windex'] for jq in neighbors]
						break
					else:
						#print "  Anchor",iq,"is not a hub"
						pass
			
			if len(anchors) == 0: #TODO should we ever reach this case?
				self.Qsubgraph.add(rep)
			elif len(anchors)  == 1:
				triedToCollapse += 1
			elif len(anchors) > 1:
				triedToCollapse += 1
				print "Failed to collapse self-motion manifold of node",self.Gq.node[rep]['windex']
				#collapse using a set-cover algorithm
				result = set_cover(anchors,neighbors)
				print "  Set cover reduced from",len(anchors),"anchors to",len(result)
				anchors = result

				#now try to shrink down the distance between anchors via a spanning tree algorithm
				Gcc = nx.subgraph(self.Gq,manifold)
				nroot = nearest_point(Gcc,anchors.keys())
				P = nx.predecessor(Gcc,nroot)
				#make the anchors graph
				Ganchors = nx.DiGraph()
				for n in anchors:
					Ganchors.add_node(n)
					p = P[n]
					while len(p) != 0 and p[0] not in anchors:
						if Ganchors.has_node(p[0]):
							break
						Ganchors.add_node(p[0])
						Ganchors.add_edge(p[0],n)
						p = P[p[0]]
				#collapse along edges in Ganchors
				for iq in reversed(nx.topological_sort(Ganchors)):
					#try attaching all anchors for iq to P[iq]
					if iq not in anchors:
						continue
					if len(P[iq]) == 0:
						#hit the root
						continue
					pq = P[iq][0]
					connected = True
					for jq in anchors[iq]:
						if not self.Gq.has_edge(pq,jq):
							if self.testAndConnect(self.Gq.node[pq]['windex'], pq, self.Gq.node[jq]['windex'], jq):
								newEdges.add((pq,jq))
							else:
								connected = False
					if connected:
						print "Moving anchors up from",iq,"to",pq
						if pq not in anchors:
							anchors[pq] = set()
						anchors[pq] |= anchors[iq]
						del anchors[iq] 
				print "Collapsed further to",len(anchors),"anchors using spanning tree"

				#do we keep around the edges?
				for iq,jq in newEdges:
					if iq not in result or jq not in result[iq]:
						self.removeInterWnodeEdge(self.Gq.node[iq]['windex'], iq, self.Gq.node[jq]['windex'], jq)

				if len(anchors) > 1:
					#can't resolve this CC, fix it using full resolution afterward
					resolutionFailures.add(rep)
					continue

				"""
				#finally, pick a root anchor and make new cobbled-together workspace edge
				nroot = max((len(v),k) for (k,v) in anchors.iteritems())[1]
				success = True
				newanchors = {nroot:None}
				for iq,v in anchors.iteritems():
					if iq == nroot: continue
					iqpath = nx.shortest_path(Gcc,nroot,iq)
					qpath = [self.Gq.node[i]['config'] for i in iqpath]
					wsrc = self.Gw.node[iw]['params']
					#first, try to shortcut the path on the self-motion manifold
					i=0
					while i+2 < len(qpath):
						imaxfeas = i
						for j in range(i+2,len(qpath)):
							if not self.validEdge(qpath[i],qpath[j],wsrc,wsrc):
								break
							print "Valid edge from",i,"to",j,"out of",len(qpath)
							imaxfeas = j
						#cut out all nodes from i + 1 to imaxfeas
						if i+1 <= imaxfeas:
							print "Deleting nodes on self-motion manifold path from index",i+1,"to",imaxfeas-1,"out of",len(qpath)
							qpath= qpath[:i+1]+qpath[imaxfeas:]
							iqpath= iqpath[:i+1]+iqpath[imaxfeas:]
						stopped = True

						i += 1
					#make a new workspace edge for each neighbor with an intermediate point determined via bisection 
					alladapted = True
					for jq in v:
						jw = self.Gq.node[jq]['windex']
						qtgt = self.Gq.node[jq]['config']
						wtgt = self.Gw.node[jw]['params']
						#test whether the qpath plus qtgt path is resolvable
						adapted = False
						amount = 0.5
						while amount > 1e-3:
							adaptPath = []
							adaptWpath = []
							for i,q in enumerate(qpath):
								if i==0:
									adaptWpath.append(wsrc)
									adaptPath.append(q)
								else:
									u = float(i)/float(len(qpath)-1)
									wnew = workspace_interpolate(wsrc,wtgt,amount*u)
									qnew = self.solve(q,wnew)
									adaptWpath.append(wnew)
									adaptPath.append(qnew)
									if qnew == None:
										print "Invalid config %d at backtracking amount %f"%(i,amount)
										break
									if not self.validEdge(adaptPath[-2],adaptPath[-1],adaptWpath[-2],adaptWpath[-1]):
										print "Invalid edge %d at backtracking amount %f"%(i,amount)
										print adaptWpath[-2],adaptWpath[-1]
										print "Shifted distances",self.robot.distance(adaptPath[-2],qpath[i-1]),self.robot.distance(adaptPath[-1],qpath[i])
										adaptPath[-1] = None
										for j in xrange(len(qpath)-1):
											if not self.validEdge(qpath[j],qpath[j+1],wsrc,wsrc):
												print "Uh... invalid path on self motion manifold?!?"
												raw_input()
										break
							if not any(q == None for q in adaptPath):
								if not self.validEdge(adaptPath[-1],qtgt,adaptWpath[-1],wtgt):
									print "Invalid final edge at backtracking amount %f"%(amount,)
									adaptPath[-1] = None
							if not any(q == None for q in adaptPath):
								print "Adapted edge with backtracking amount",amount
								#successful!
								adapted = True
								#now make workspace edges
								iwlast = iw
								iqlast = nroot
								for w,q in zip(adaptWpath,adaptPath)[1:]:
									iwnew = self.addWorkspaceNode(w)
									self.addWorkspaceEdge(iwlast,iwnew)
									iqnew = self.addNode(iwnew,q)
									self.Gw.node[iwnew]['qccs'] = DisjointSet()
									self.Gw.node[iwnew]['qccs'].add(iqnew)
									self.addEdge(iwlast,iqlast,iwnew,iqnew)
									#add iqnew to indexing structure
									wToQccRep[iwnew] = iqnew
									#add iqnew to Qsubgraph
									self.Qsubgraph.add(iqnew)
									iwlast,iqlast = iwnew,iqnew

								#connect to target node
								self.addWorkspaceEdge(iwlast,jw)
								self.addEdge(iwlast,iqlast,jw,jq)
								#need to remove (iq,jq) from edge list
								self.removeInterWnodeEdge(iw, iq, jw, jq)
								#also remove (iw,jw)?
								self.removeWorkspaceEdge(iw,jw)
								break
							amount *= 0.5
						if not adapted:
							alladapted = False
							print "Strange, could not make path to slightly leave self-motion manifold"
							raw_input()
					if not alladapted:
						print "Need to keep",iq,"as extra anchor"
						raw_input("Press enter to continue")
						success = False
						newanchors[iq] = None
						#need to keep as anchors
				anchors = newanchors
				"""
			assert len(anchors) <= 1
			for iq in anchors:
				resolvedAnchors[iw] = iq
				self.Qsubgraph.add(iq)

		#some resolutions failed -- try doing a CSP only on these and their neighbors
		if len(resolutionFailures) > 0:
			print "TRYING NEW RESOLUTION TECHNIQUE"
			failureWNodes = [self.Gq.node[rep]['windex'] for rep in resolutionFailures]
			failureManifolds = dict([(iw,self.Gw.node[iw]['qccs'].getSetRef(rep)) for (iw,rep) in zip(failureWNodes,resolutionFailures)])
			failureWNodes = set(failureWNodes)
			neighbors = ring(self.Gw,failureWNodes)
			neighborAnchors = ring(self.Gw,failureWNodes,2)
			neighborHubs = dict()
			print len(failureWNodes),"nodes could not be resolved"
			print "1-ring size",len(neighbors)
			print "2-ring size",len(neighborAnchors)
			print "Determining 1-ring hubs..."
			numManifoldConfigs = sum(len(v) for k,v in failureManifolds.iteritems())
			for jw in neighbors:
				if jw not in wToQccRep: continue
				#might be missing a C-space edge from iw to jw
				assert jw in resolvedAnchors
				jwRep = wToQccRep[jw]
				jmanifold = self.Gw.node[jw]['qccs'].getSetRef(jwRep)
				kwlist = [kw for kw in self.Gw.neighbors(jw) if kw in neighborAnchors and (jw,kw) in assignedWEdges]
				kwanchors = [resolvedAnchors.get(kw,None) for kw in kwlist]
				#determine potential hub configurations
				hubs = []
				for jq in jmanifold:
					valid = True
					for kw,anch in zip(kwlist,kwanchors):
						if anch != None:
							if not self.testAndConnect(jw,jq,kw,anch):
								valid = False
								break
					if valid:
						hubs.append(jq)
				if len(hubs) == 0:
					print "There appears to be a node in the 1 ring (%d) that has no hub vertex?"%(jw,)
					print "Assignment is",resolvedAnchors[jw]
					assert resolvedAnchors[jw] in jmanifold
					print "Neighboring wnodes",kwlist
					print "Neighboring anchors",kwanchors
					raise ValueError("Collapse function is mssed up")

				neighborHubs[jw] = hubs
				self.Qsubgraph.remove(resolvedAnchors[jw])
			numHubConfigs = sum(len(hub) for hub in neighborHubs.itervalues())
			print "# of manifold configs total",numManifoldConfigs,"# of hub configs total",numHubConfigs
			#now do exhaustive connections between manifold configs in failureWNodes and hub configs in neighbors
			#add these options to the CSP
			csp = CSP()
			for iw,imanifold in failureManifolds.iteritems():
				csp.addVariable(list(imanifold),'q_'+str(iw))
			for jw,jhub in neighborHubs.iteritems():
				csp.addVariable(jhub,'qhub_'+str(jw))
			print "Determining all-pair connections..."
			numValidEdges = 0
			numTestedEdges = 0
			for iw,imanifold in failureManifolds.iteritems():
				for jw in self.Gw.neighbors(iw):
					if jw in failureWNodes:
						validValues = []
						for iq in imanifold:
							for jq in failureManifolds[jw]:
								numTestedEdges += 1
								if self.testAndConnect(iw,iq,jw,jq):
									validValues.append((iq,jq))
									numValidEdges += 1
						csp.addConstraint(validValues,("q_"+str(iw),"q_"+str(jw)))
					elif jw in neighborHubs:
						validValues = []
						for iq in imanifold:
							for jq in neighborHubs[jw]:
								numTestedEdges += 1
								if self.testAndConnect(iw,iq,jw,jq):
									validValues.append((iq,jq))
									numValidEdges += 1
						csp.addConstraint(validValues,("q_"+str(iw),"qhub_"+str(jw)))
			print "# of valid edges",numValidEdges,"# total",numTestedEdges
			raw_input("Press enter to begin CSP assignment")
			assignment = csp.heuristicMaxAssignment()
			assignment = csp.randomDescentAssignment(assignment)
			print "# of conflicts",csp.numConflicts(assignment)
			varsinconflict = []
			failuresinconflict = []
			neighborsinconflict = []
			for c in xrange(len(csp.constraints)):
				if not csp.testConstraint(c,assignment):
					for v in csp.conDomains:
						vname = csp.variables[v]
						varsinconflict.append(vname)
						if vname.startswith('qhub_'):
							neighborsinconflict.append(int(vname[5:]))
						else:
							failuresinconflict.append(int(vname[2:]))
			print "# of centers in conflict",len(failuresinconflict)
			print "# of neighbors in conflict",len(neighborsinconflict)
			for v,vname in enumerate(csp.variables):
				self.Qsubgraph.add(assignment[v])
		else:
			print "NO NEED FOR FURTHER RESOLUTION"
				
		if len(self.QccSubgraph) > 0:
			return float(totalAnchors)/float(len(self.QccSubgraph)),float(totalNeighbors)/float(len(self.QccSubgraph)),totalCollapsed,triedToCollapse
		else:
			return -1,-1,-1,-1
			
	def decodeCSPAssignment(self,csp_assignment,visibility=False):
		"""For an assignment produced by makeCSP, calculates self.Qsubgraph."""
		if visibility:		
			self.QccSubgraph = set()
		else:
			self.Qsubgraph = set()
		vcount = 0
		for n,d in self.Gw.nodes(data=True):
			if len(d['qlist']) > 0:
				assert vcount < len(csp_assignment)
				if csp_assignment[vcount] != None:
					if visibility:
						self.QccSubgraph.add(csp_assignment[vcount])
					else:
						self.Qsubgraph.add(csp_assignment[vcount])
				vcount += 1
		if visibility:
			return self.collapseQccSubgraph()
		else:
			return None,None,None,None

	def encodeCSPAssignment(self):
		"""Given self.Qsubgraph, returns an assignment for the CSP produced by makeCSP."""
		if len(self.Qsubgraph) == 0:
			return None
		res = []
		for n,d in self.Gw.nodes(data=True):
			if len(d['qlist'])==0: continue
			res.append(None)
			for iq in d['qlist']:
				if iq in self.Qsubgraph:
					res[-1] = iq
					break
		return res
		
	def saveNoncollapsibleManifolds(self, path, create=True):
		"""Call this in experimenting.py with path = 'experiments'.
		The saved .dot files can be visualized with GraphViz, i.e. the command
		'dot -Tpng manifold186.dot -o manifold186.png'"""
		if create and not os.path.isdir(path):
			os.makedirs(path)
		#loop over and identify noncollapsible representatives
		noncollapsible = set()
		manifolds = set()
		for iq in self.Qsubgraph:
			iw = self.Gq.node[iq]['windex']
			rep = self.Gw.node[iw]['qccs'].getSetRep(iq)
			if rep in manifolds:
				noncollapsible.add(rep)
			else:
				manifolds.add(rep)
		#for each noncollapsible representative, save subgraph as .dot file
		for irep in noncollapsible:
			windices = set()
			iw = self.Gq.node[irep]['windex']
			windices.add(iw)
			manifold = self.Gw.node[iw]['qccs'].getSetRef(irep)
			adjacentManifolds = set()
			for iq in manifold:
				assert self.Gq.node[iq]['windex'] == iw
				touched = [jq for jq in self.Gq.neighbors(iq) if self.Gq.node[jq]['windex'] != iw]
				for jq in touched:
					jw = self.Gq.node[jq]['windex']
					adjacentManifolds.add(self.Gw.node[jw]['qccs'].getSetRep(jq))
			nbunch = set()
			nbunch.update(manifold)
			for jrep in adjacentManifolds:
				jw = self.Gq.node[jrep]['windex']
				windices.add(jw)
				nbunch.update(self.Gw.node[jw]['qccs'].getSetRef(jrep))
			subgraph = nx.Graph(self.Gq.subgraph(nbunch))
			windices = list(windices)
			for n,d in subgraph.nodes_iter(data=True):
				d['style'] = 'filled'
				d['fillcolor'] = '/greys7/' + str((windices.index(d['windex']) % 7) + 1)
				if d['windex'] == iw: d['shape'] = 'diamond'
			nx.drawing.nx_pydot.write_dot(subgraph,path+'/manifold'+str(irep)+'.dot')
			

	def uniqueAssignment(self):
		"""Tries to solve the optimization max{S} cover(S) such that each point in Gw is assigned to
		at most one element of Gq, and cover(S) measures the number of edges of Gw covered.

		Each edge in Gw is assigned to at most one edge of Gq.  This is a constraint satisfaction
		optimization problem, where each edge is a variable, and they must meet the
		constraint that an adjacent edge's endpoint must match the endpoint of the edge.

		Specifically, for edge (u,v), need to meet the constraint that for all t in Gw.neighbors(u),
			Val(t) in Gq.neighbors(Val(u))
		and for all W in Gw.neighbors(V),
			Val(w) in Gq.neighbors(Val(v))

		I've done some experimenting with heuristics.
		A typical CSP heuristic is to assign the edge with the fewest remaining values
		(most-constrained-value) and breaks ties with largest degree (most-constraining-variable). 
		The value to which it is assigned is the least-constraining-value heuristic. The only issue
		is that this tries to find a contradiction along a branch as quickly as possible.
		If we want to maximize the number of edges assigned, then perhaps a different ordering would be
		better.  It performs OK.

		Least-constraining variable might be a good heuristic but it's relatively expensive
		least-constraining-value works well for a value assignment.
		"""
		print "Trying CSP solver assignment..."
		t0 = time.time()
		csp = self.makeCSP()
		print "Time to make CSP:",time.time()-t0
		assignment = csp.heuristicMaxAssignment()
		if assignment != None:
			self.decodeCSPAssignment(assignment)
		else:
			print "CSP solver failed"
		return

		self.Qsubgraph = set()
		unassigned = set((a,b) for a,b,d in self.Gw.edges_iter(data=True) if (a < b and len(d['qlist']) > 0))
		vunassigned = set(n for n,d in self.Gw.nodes(data=True) if len(d['qlist']) > 0)
		numPotentialEdges = len(unassigned)
		numPotentialNodes = len(vunassigned)
		domains = dict([(n,set(self.Gw.node[n]['qlist'])) for n in vunassigned] + [(e,set(self.Gw.edge[e[0]][e[1]]['qlist'])) for e in unassigned])
		def ac3(e1,e2):
			#for two edges e1 and e2 (sorted) ensures that there are mutually compatible domains
			assert e1[0] in e2 or e1[1] in e2,"Edges must overlap on a vertex"
			d1 = [v for v in domains[e1]]
			d2 = [v for v in domains[e2]]
			numchanged = 0
			changed = True
			while changed:
				changed = False
				q1 = set([e[0] for e in d1] + [e[1] for e in d1])
				q2 = set([e[0] for e in d2] + [e[1] for e in d2])
				newd1 = []
				newd2 = []
				for qe1 in d1:
					if not (qe1[0] in q2 or qe1[1] in q2):
						#incompatible
						changed = True
						numchanged += 1
					else:
						newd1.append(qe1)
				for qe2 in d2:
					if not (qe2[0] in q1 or qe2[1] in q1):
						#compatible
						changed = True
						numchanged += 1
					else:
						newd2.append(qe2)
				if changed:
					d1 = newd1
					d2 = newd2
			if numchanged == 0:
				return domains[e1],domains[e2]
			#print "AC3 removed",numchanged,"elements"
			return set(d1),set(d2)
		def run_ac3(e1=None,e2=None):
			if e1 == None:
				for e1 in unassigned:
					run_ac3(e1)
			elif e2 == None:
				a,b = e1
				if a in vunassigned:
					for u in self.Gw.neighbors(a):
						e2 = (a,u) if a < u else u,a
						if e2 in unassigned:
							run_ac3(e1,e2)
				if b in vunassigned:
					for u in self.Gw.neighbors(b):
						e2 = (b,u) if a < u else u,b
						if e2 in unassigned:
							run_ac3(e1,e2)
			else:
				d1,d2 = ac3(e1,e2)
				if len(d1) == 0 or len(d2) == 0:
					#ack, we've eliminated everything
					return
				else:
					domains[e1] = d1
					domains[e2] = d2
		run_ac3()
		def getdegree(n):
			return len([m for m in self.Gw.neighbors(n) if m in vunassigned])
		degrees = dict((n,getdegree(n)) for n in vunassigned)
		def getscore(a,b):
			assert a < b
			#this is the number of neighbors of the edge's assigned endpoints, which
			#encourages coherence in the propagation.  Experiments suggest that it's not
			#a great heuristic
			assigneddegree = 0
			#this is the number of unassigned neighbors of the edge's endpoints
			degree = 0
			#minimum fraction of values remained by some assignment to a,b
			maxCompatible = 0
			if a in vunassigned:
				degree += degrees[a]
			else:
				for u in self.Gw.neighbors(a):
					e = (a,u) if a < u else (u,a)
					if e not in unassigned:
						assigneddegree += 1
			if b in vunassigned:
				degree += degrees[b]
			else:
				for u in self.Gw.neighbors(b):
					e = (b,u) if b < u else (u,b)
					if e not in unassigned:
						assigneddegree += 1
			#compute num introduced conflicts
			"""
			for (qa,qb) in domains[a,b]:
				minCompatible = 1
				if a in vunassigned:
					for u in self.Gw.neighbors(a):
						e = (a,u) if a < u else (u,a)
						if e not in unassigned: continue
						d = domains[e]
						if len(d) == 0: continue
						numCompatible = sum(1 for (qua,qub) in d if qa in (qua,qub))
						minCompatible = min(minCompatible,float(numCompatible)/len(d))
				if b in vunassigned:
					for u in self.Gw.neighbors(b):
						e = (b,u) if b < u else (u,b)
						if e not in unassigned: continue
						d = domains[e]
						if len(d) == 0: continue
						numCompatible = sum(1 for (qua,qub) in d if qb in (qua,qub))
						minCompatible = min(minCompatible,float(numCompatible)/len(d))
				maxCompatible = max(maxCompatible,minCompatible)
				if maxCompatible==1:
					break
			return (maxCompatible,degree)
			"""
			#most-constrained-value heuristic doesn't work well for minimizing number of conflicts
			#since we're not backtracking
			return (-len(domains[(a,b)]),degree)
			#return (assigneddegree,degree)
		scores = dict((e,getscore(*e)) for e in unassigned)
		failures = []
		while len(unassigned) > 0:
			bestlist = []
			bestScore = (-float('inf'),0)
			#find the max scoring node in scores
			for (a,b),score in scores.iteritems():
				#score2 = getscore(a,b)
				#if score2 != score:
				#    print "Warning, score for edge",a,b,"is different than cached value",score2,score
				#    print "Neighbors:",a,self.Gw.neighbors(a)
				#    print "  unassigned:",[n in vunassigned for v in self.Gw.neighbors(a)]
				#    print "Neighbors:",b,self.Gw.neighbors(b)
				#    print "  unassigned:",[n in vunassigned for v in self.Gw.neighbors(b)]
				#    raw_input()
				if score > bestScore:
					bestScore = score
					bestlist = [(a,b)]
				elif score == bestScore:
					bestlist.append((a,b))
			#Deterministic choice
			#best = min(bestlist)
			#Random choice among best
			best = random.choice(bestlist)
			domain = domains[best]
			if len(domain) == 0:
				print "Empty domain",best,"score",bestScore
				#out of values to assign
				failures.append(best)
				unassigned.remove(best)
				del scores[best]
				empty = []
				for (a,b) in unassigned:
					if len(domains[a,b])==0:
						empty.append((a,b))
				for (a,b) in empty:
					unassigned.remove((a,b))
					del scores[a,b]
					failures.append((a,b))
				continue
			print "Assigning edge",best,":",bestScore
			#look at all possible edge assignments and how many neighboring values
			#will remain after assignment
			bestValue = None
			bestScore = (-1,0,0)
			(a,b) = best
			for (qa,qb) in domain:
				mincompatible = 100000
				numcompatible = 0
				distance = self.robot.distance(self.Gq.node[qa]['config'],self.Gq.node[qb]['config'])
				for u in self.Gw.neighbors(a):
					e = (a,u) if a < u else (u,a)
					if e not in unassigned: 
						continue
					ncompatible = 0
					for qe in domains[e]:
						if qa in qe:
							ncompatible += 1
							if ncompatible >= mincompatible:
								break
					if ncompatible != 0: numcompatible += 1
					mincompatible = min(ncompatible,mincompatible)
				for w in self.Gw.neighbors(b):
					e = (b,w) if b < w else (w,b)
					if e not in unassigned: 
						continue
					ncompatible = 0
					for qe in domains[e]:
						if qb in qe:
							ncompatible += 1
							if ncompatible >= mincompatible:
								break
					if ncompatible != 0: numcompatible += 1
					mincompatible = min(ncompatible,mincompatible)
				#print "Number of values compatible with",(qa,qb),":",mincompatible
				score = (mincompatible,numcompatible,-distance)
				if score > bestScore:
					bestScore = score
					bestValue = (qa,qb)
			print "  to value",bestValue,"score %d min compatible, %d total compatible, distance %g"%bestScore
			#if bestScore[0] > 1:
			#    raw_input()
			if bestScore == (0,0):
				print "Warning, not compatible with any neighbors"
				print "Domain:",domain
			(qa,qb) = bestValue
			#now propagate domains so that they are compatible with (qa,qb) (forward checking)
			for u in self.Gw.neighbors(a):
				e = (a,u) if a < u else (u,a)
				if e not in unassigned: 
					continue
				newdomain = set(qe for qe in domains[e] if qa in qe)
				domains[e] = newdomain
			for w in self.Gw.neighbors(b):
				e = (b,w) if b < w else (w,b)
				bindex = 0 if b < w else 1
				if e not in unassigned: 
					continue
				newdomain = set(qe for qe in domains[e] if qb in qe)
				domains[e] = newdomain
			#run AC3
			for u in self.Gw.neighbors(a):
				e = (a,u) if a < u else (u,a)
				if e not in unassigned: 
					continue
				run_ac3(e)
			for w in self.Gw.neighbors(b):
				e = (b,w) if b < w else (w,b)
				bindex = 0 if b < w else 1
				if e not in unassigned: 
					continue
				run_ac3(e)
			#remove from unassigned list
			if a in vunassigned:
				vunassigned.remove(a)
				assert a not in vunassigned
				if self.Gq.node[qa]['windex']==a:
					self.Qsubgraph.add(qa)
				else:
					assert self.Gq.node[qb]['windex']==a
					self.Qsubgraph.add(qb)
			else:
				assert qa in self.Qsubgraph or qb in self.Qsubgraph
			if b in vunassigned:
				vunassigned.remove(b)
				assert b not in vunassigned
				if self.Gq.node[qa]['windex']==b:
					self.Qsubgraph.add(qa)
				else:
					assert self.Gq.node[qb]['windex']==b
					self.Qsubgraph.add(qb)
			else:
				assert qa in self.Qsubgraph or qb in self.Qsubgraph
			assert (qa in self.Qsubgraph and qb in self.Qsubgraph),"Uh... edge added without endpoints being added"

			unassigned.remove((a,b))

			#now update scores and degrees
			del scores[(a,b)]
			del degrees[a]
			del degrees[b]
			for u in self.Gw.neighbors(a):
				degrees[u] = getdegree(u)
			for u in self.Gw.neighbors(b):
				degrees[u] = getdegree(u)
			for u in self.Gw.neighbors(a):
				for v in self.Gw.neighbors(u):
					e = (u,v) if u < v else (v,u)
					if e not in unassigned: 
						continue
					scores[e] = getscore(*e)
			for u in self.Gw.neighbors(b):
				for v in self.Gw.neighbors(u):
					e = (u,v) if u < v else (v,u)
					if e not in unassigned: 
						continue
					scores[e] = getscore(*e)
			for e in unassigned:
				assert e in scores
			for e in scores:
				assert e in unassigned
		if len(failures) > 0:
			print "Unique resolution failed, missing",len(failures),"/",numPotentialEdges,"edges and",len(vunassigned),"/",numPotentialNodes,"vertices"
		recount = 0
		numtotal = 0
		for (i,j,d) in self.Gw.edges_iter(data=True):
			if len(d['qlist']) == 0:
				continue
			numtotal += 1
			nsubset = sum(1 for (ia,ib) in d['qlist'] if (ia in self.Qsubgraph and ib in self.Qsubgraph))
			if nsubset == 0:
				recount += 1
		print "Recounted missing",recount,"/",numtotal

	def randomDescentAssignment(self,randomize=True):
		"""Tries to solve the optimization max{S} cover(S) such that each point in Gw is assigned to
		at most one element of Gq, and cover(S) measures the number of edges of Gw covered.

		Each node in Gw is assigned to at most one node of Gq.  This is a constraint satisfaction
		optimization problem, where each node is a variable, and they must meet the
		constraint that the assignments to two adjacent nodes in Gw correspond to an edge in Gq

		Specifically, for node i, need to meet the constraint that for all j in Gw.neighbors(i),
			The edge (Val(i),Val(j)) exists in in Gq

		This heuristic samples a random assignment for each node (if randomize=True).  Then, for all nodes
		that conflict with their neighbors, it attempts to switch the assignment to a less-
		conflicting one.  This repeats until no progress can be made.

		#OLD
		#Each edge in Gw is assigned to at most one edge of Gq.  This is a constraint satisfaction
		#optimization problem, where each edge is a variable, and they must meet the
		#constraint that an adjacent edge's endpoint must match the endpoint of the edge.

		#Specifically, for edge (u,v), need to meet the constraint that for all t in Gw.neighbors(u),
		#    Val(t) in Gq.neighbors(Val(u))
		#and for all W in Gw.neighbors(V),
		#    Val(w) in Gq.neighbors(Val(v))

		#This heuristic samples a random assignment for each edge.  Then, for all edges that
		#conflict with their neighbors, it attempts to switch the assignment to a less-
		#conflicting one.  This repeats until no progress can be made.
		"""
		assignment = (self.encodeCSPAssignment() if not randomize else None)
		t0 = time.time()
		csp = self.makeCSP()
		print "Time to make CSP:",time.time()-t0
		assignment = csp.randomDescentAssignment(assignment,perturb=True)
		if assignment != None:
			self.decodeCSPAssignment(assignment)
		else:
			print "CSP solver failed"
		return

		assignment = dict()
		for i,d in self.Gw.nodes(data=True):
			if len(d['qlist']) == 0: 
				continue
			if randomize:
				assignment[i] = random.choice(d['qlist'])
			else:
				q = [eq for eq in d['qlist'] if eq in self.Qsubgraph]
				if len(q) == 0:
					assignment[i] = random.choice(d['qlist'])
				else:
					assert len(q)==1,"Weird, Q subgraph has multiple assignments to node %d, got %d values"%(i,len(q))
					assignment[i] = random.choice(q)
		def getNumConflicts(i,val=None):
			if val is None:
				val = assignment[i]
			nconflicts = 0
			for j in self.Gw.neighbors(i):
				if j not in assignment: continue
				if (val,assignment[j]) not in self.Gw.edge[i][j]['qlist'] and (assignment[j],val) not in self.Gw.edge[i][j]['qlist']:
					nconflicts+=1
			return nconflicts
		def quality(i,val=None):
			if val is None:
				val = assignment[i]
			d = 0
			for j in self.Gw.neighbors(i):
				if j not in assignment: continue
				if (val,assignment[j]) in self.Gw.edge[i][j]['qlist'] or (assignment[j],val) in self.Gw.edge[i][j]['qlist']:
					d += self.robot.distance(self.Gq.node[assignment[i]]['config'],self.Gq.node[assignment[i]]['config'])
			return d
		numConflicts = dict((v,getNumConflicts(v)) for v in assignment)
		if not randomize:
			pRandomizeConflicts = 0.1
			pRandomizeBorder = 0.0
			#randomize conflicting vertices
			for (v,nc) in numConflicts.iteritems():
				if nc > 0 and random.random() < pRandomizeConflicts:
					assignment[v] = random.choice(self.Gw.node[v]['qlist'])
				if nc > 0 and random.random() < pRandomizeBorder:
					w = random.choice([j for j in self.Gw.neighbors(v) if len(self.Gw.node[j]['qlist'])>0])
					assignment[w] = random.choice(self.Gw.node[w]['qlist'])



		totalConflicted = len([c for c in numConflicts.itervalues() if c != 0])
		active = set([k for k,v in numConflicts.iteritems()])
		while True:
			if len(active) == 0:
				print "Found a conflict-free setting!"
				#greedily build subgraph of conflict-free vertices
				self.Qsubgraph = set(assignment.values())
				return True
			print "Descent pass through active list of size",len(active)
			changed = False
			changeList = []
			order = list(active)
			random.shuffle(order)
			for i in order:
				iconflicts = numConflicts[i],quality(i)
				bestnewval = None
				bestconflicts = iconflicts
				for val in self.Gw.node[i]['qlist']:
					nc = getNumConflicts(i,val),quality(i,val)
					if nc < bestconflicts:
						bestconflicts = nc
						bestnewval = val
				if bestnewval != None:
					changed=True
					changeList.append(i)
					assignment[i] = bestnewval
					#now update neighbors' conflict counts, wake up
					#dormant neighbors
					for j in self.Gw.neighbors(i):
						if j in assignment:
							numConflicts[j] = getNumConflicts(j)
							active.add(j)
					numConflicts[i] = bestconflicts[0]
					#print "  Changed edge",e,"from",econflicts,"conflicts to",bestconflicts
				else:
					#print "  No change to edge",e,"still",econflicts,"conflicts"
					assert iconflicts[0] == numConflicts[i]
				if i in active:
					active.remove(i)
			newConflicted = len([c for c in numConflicts.itervalues() if c != 0])
			print "Original # of conflicts",totalConflicted,"now",newConflicted
			totalConflicted = newConflicted
			print "# of changed vertices",len(changeList)
			#raw_input()
			if not changed:
				#build subgraph of conflict-free edges
				self.Qsubgraph = set(assignment.values())
				nconflicted = 0
				ntotal = 0
				for (i,j,d) in self.Gw.edges_iter(data=True):
					if len(d['qlist'])==0: continue
					ntotal += 1
					if not any((iq in self.Qsubgraph and jq in self.Qsubgraph) for (iq,jq) in d['qlist']):
						nconflicted += 1
				print nconflicted,"edges of",ntotal,"still left in conflict"
				return False
		"""
		assignment = dict()
		for (a,b,d) in self.Gw.edges_iter(data=True):
			if (a < b and len(d['qlist']) > 0):
				if randomize:
					assignment[a,b] = random.choice(d['qlist'])
				else:
					q = [eq for eq in d['qlist'] if eq[0] in self.Qsubgraph and eq[1] in self.Qsubgraph]
					if len(q) == 0:
						assignment[a,b] = random.choice(d['qlist'])
					else:
						assert len(q)==1
						assignment[a,b] = random.choice(q)
						
		def getNumConflicts(e,val=None):
			if val is None:
				val = assignment[e]
			a,b = e 
			qa,qb = val
			nconflicts = 0
			for u in self.Gw.neighbors(a):
				e = (u,a) if u < a else (a,u)
				if e == (a,b): continue
				if e not in assignment: continue
				if qa not in assignment[e]:
					nconflicts+=1
			for u in self.Gw.neighbors(b):
				e = (u,a) if u < b else (b,u)
				if e == (a,b): continue
				if e not in assignment: continue
				if qb not in assignment[e]:
					nconflicts+=1
			return nconflicts
		def quality(e,val=None):
			if val is None:
				val = assignment[e]
			return self.robot.distance(self.Gq.node[val[0]]['config'],self.Gq.node[val[1]]['config'])
		numConflicts = dict((e,getNumConflicts(e)) for e in assignment)
		totalConflicted = len([c for c in numConflicts.itervalues() if c != 0])
		active = set([k for k,v in numConflicts.iteritems()])
		while True:
			if len(active) == 0:
				print "Found a conflict-free setting!"
				#greedily build subgraph of conflict-free edges
				self.Qsubgraph = set()
				vassigned = dict()
				for (e,v) in assignment.iteritems():
					if (e[0] not in vassigned or vassigned[e[0]] == v[0]) and (e[1] not in vassigned or vassigned[e[1]] == v[1]):
						vassigned[e[0]] = v[0]
						vassigned[e[1]] = v[1]
						self.Qsubgraph.add(v[0])
						self.Qsubgraph.add(v[1])
				for (e,v) in assignment.iteritems():
					assert numConflicts[e] == getNumConflicts(e,v)
				return True
			print "Descent pass through active list of size",len(active)
			changed = False
			changeList = []
			for e in list(active):
				econflicts = numConflicts[e],quality(e)
				bestnewval = None
				bestconflicts = econflicts
				bestdistance = self.robot.distance(self.Gq.node[assignment[e][0]]['config'],self.Gq.node[assignment[e][1]]['config'])
				for val in self.Gw.edge[e[0]][e[1]]['qlist']:
					nc = getNumConflicts(e,val),quality(e,val)
					if nc < bestconflicts:
						bestconflicts = nc
						bestnewval = val
						distance = self.robot.distance(self.Gq.node[val[0]]['config'],self.Gq.node[val[1]]['config'])
						bestdistance = distance
					elif nc == bestconflicts:
						distance = self.robot.distance(self.Gq.node[val[0]]['config'],self.Gq.node[val[1]]['config'])
						if distance < bestdistance:
							bestdistance = distance
							bestnewval = val
				if bestnewval != None:
					changed=True
					changeList.append(e)
					updatea = (bestnewval[0] != assignment[e][0])
					updateb = (bestnewval[1] != assignment[e][1])
					assignment[e] = bestnewval
					#now update neighbors' conflict counts, wake up
					#dormant neighbors
					if updatea:
						for u in self.Gw.neighbors(e[0]):
							e2 = (u,e[0]) if u < e[0] else (e[0],u)
							if e2 in assignment:
								numConflicts[e2] = getNumConflicts(e2)
								active.add(e2)
					if updateb:
						for u in self.Gw.neighbors(e[1]):
							e2 = (u,e[1]) if u < e[1] else (e[1],u)
							if e2 in assignment:
								numConflicts[e2] = getNumConflicts(e2)
								active.add(e2)
					numConflicts[e] = bestconflicts[0]
					#print "  Changed edge",e,"from",econflicts,"conflicts to",bestconflicts
				else:
					#print "  No change to edge",e,"still",econflicts,"conflicts"
					assert econflicts[0] == numConflicts[e]
				if e in active:
					active.remove(e)
			newConflicted = len([c for c in numConflicts.itervalues() if c != 0])
			print "Original # of conflicts",totalConflicted,"now",newConflicted
			totalConflicted = newConflicted
			print "# of changed edges",len(changeList)
			raw_input()
			if not changed:
				#build subgraph of conflict-free edges
				self.Qsubgraph = set()
				vassigned = dict()
				nconflicted = 0
				for (e,v) in assignment.iteritems():
					if (e[0] not in vassigned or vassigned[e[0]] == v[0]) and (e[1] not in vassigned or vassigned[e[1]] == v[1]):
						vassigned[e[0]] = v[0]
						vassigned[e[1]] = v[1]
						self.Qsubgraph.add(v[0])
						self.Qsubgraph.add(v[1])
					else:
						nconflicted += 1
				print nconflicted,"edges of",self.Gw.number_of_edges(),"still left in conflict"
				for (e,v) in assignment.iteritems():
					if numConflicts[e] != getNumConflicts(e,v):
						print e,numConflicts[e],getNumConflicts(e,v)
					assert numConflicts[e] == getNumConflicts(e,v)
				return False
		"""

	def pointwiseAssignment(self,NConfigsPerPoint=10):
		"""Moves pointwise through the workspace grid and generates a configuration by
		local optimization from the previous grid point.  If a configuration cannot be
		generated, then the edge is marked as infeasible and a random configuration is
		generated.
		"""
		print "Generating pointwise assignment"
		start_node_count = self.Gw.graph['qnodes']
		qmap = dict()
		emap = list()
		efailures = 0
		c = ProgressUpdater(self.Gw.number_of_nodes(),5)
		for i,d in self.Gw.nodes(data=True):
			c.update()
			adjacentWithConfigs = [j for j in self.Gw.neighbors(i) if j < i and j in qmap]
			solved = False
			if len(adjacentWithConfigs) > 0:
				self.setIKProblem(d['params'])
				self.ikSolverParams.startRandom = False
				qstart = robot_average(self.robot,[qmap[j] for j in adjacentWithConfigs])
				self.robot.setConfig(qstart)
				q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
				if q is not None:
					for j in adjacentWithConfigs:
						if self.validEdge(qmap[j],q,self.Gw.node[j]['params'],d['params']):
							emap.append((i,j))
							qmap[i] = q
							solved = True
						else:
							efailures += 1
			if not solved:
				efailures += len([j for j in self.Gw.neighbors(i) if j < i])
				#no edge between neighbors and this node
				self.setIKProblem(d['params'])
				self.ikSolverParams.startRandom = True
				for sample in xrange(NConfigsPerPoint):
					#solver will sample randomly
					q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
					if q is not None:
						qmap[i] = q
						break
		c.done()
		print "Sampled",len(qmap),"of possible",self.Gw.number_of_nodes()
		print "Connected",len(emap),"of possible",self.Gw.number_of_edges(),"failed to connect",efailures
		nodemap = dict()
		for (i,q) in qmap.iteritems():
			iq = self.addNode(i,q)
			nodemap[i] = iq
		qtow = dict((v,k) for (k,v) in nodemap.iteritems())
		for (i,j) in emap:
			iq,jq = nodemap[i],nodemap[j]
			self.addEdge(qtow[iq],iq,qtow[jq],jq)
		self.Qsubgraph = set(nodemap.values())


	def ikError(self,q,x):
		"""Returns the ik error (norm of residual) at config q for workspace parameter x"""
		self.setIKProblem(x)
		s = ik.solver(self.ikTemplate.objectives)
		self.robot.setConfig(q)
		r = s.getResidual()
		return vectorops.norm(r)

	def validEdgeSimple(self,qa,qb,wa,wb,epsilon=5e-2):
		ea = self.ikError(qb,wa)
		eb = self.ikError(qa,wb)
		if self.wRadius == None:
			self.autoSetWorkspaceRadius()
		emax = self.wRadius*0.5
		#midpoint fast reject test
		x = self.robot.interpolate(qa,qb,0.5)
		if self.ikError(x,workspace_interpolate(wa,wb,0.5)) > emax:
			#print "Reject deviation from line too large at midpoint"
			return False
		#if self.ikError(x,wa) > ea or self.ikError(x,wb) > eb:
		#	return False
		eaold = 0
		ebold = eb
		Ndivs = int(math.ceil(self.robot.distance(qa,qb)/epsilon))
		path = []
		for i in range(Ndivs):
			u = float(i+1)/(Ndivs+1)
			x = self.robot.interpolate(qa,qb,u)
			eu = self.ikError(x,workspace_interpolate(wa,wb,u))
			eau = self.ikError(x,wa)
			ebu = self.ikError(x,wb)
			#if eau < eaold:
			#	#print "Reject monotonic increase from a at",u
			#	return False
			#if ebu > ebold:
			#	#print "Reject monotonic decrease from b at",u,",",ebold,"->",ebu
			#	return False
			if eu > emax:
				#print "Reject deviation from line too large at",u
				return False
			#if eau > ea or ebu > eb:
			#	return False
			eaold = eau
			ebold = ebu
			path.append(x)
		if self.ikTemplate.feasibilityTest != None:
			for q in path:
				if not self.ikTemplate.feasibilityTest(q):
					return False
		return True

	def validEdgeLinear(self,qa,qb,wa,wb,epsilon=5e-2):
		ea = self.ikError(qb,wa)
		eb = self.ikError(qa,wb)
		qprev = qa
		sumdiff = 0
		maxdiff = 0
		Ndivs = int(math.ceil(self.robot.distance(qa,qb)/epsilon))
		path = []
		discontinuityThreshold = epsilon*10
		for i in range(Ndivs):
			u = float(i+1)/(Ndivs+1)
			x = self.robot.interpolate(qa,qb,u)
			self.robot.setConfig(x)
			q = self.solve(x,workspace_interpolate(wa,wb,u))
			if q == None:
				#print "Unable to solve",x
				return False
			path.append(q)
			d = self.robot.distance(q,qprev)
			sumdiff += d
			maxdiff = max(d,maxdiff)
			if maxdiff > discontinuityThreshold:
				#print "Discontinuity threshold exceeded",d,discontinuityThreshold
				return False
			qprev = q
		#test feasibility of path
		if self.ikTemplate.feasibilityTest != None:
			for q in path:
				if not self.ikTemplate.feasibilityTest(q):
					return False
		return True

	def validEdgeBisection(self,qa,qb,wa,wb,epsilon=5e-2,c=0.9):
		d0 = self.robot.distance(qa,qb)
		if d0 <= epsilon:
			return True
		wm = workspace_interpolate(wa,wb,0.5)
		qm = self.robot.interpolate(qa,qb,0.5)
		q = self.solve(qm,wm)
		if q is None:
			return False
		d1 = self.robot.distance(qa,q)
		d2 = self.robot.distance(q,qb)
		if max(d1,d2) > c*d0: return False
		if d1 > epsilon and not self.validEdgeBisection(qa,q,wa,wm,epsilon,c):
			return False
		if d2 > epsilon and not self.validEdgeBisection(q,qb,wm,wb,epsilon,c):
			return False
		return True

	def interpolateEdgeBisection(self,qa,qb,wa,wb,u,epsilon=5e-2,c=0.9):
		d0 = self.robot.distance(qa,qb)
		if d0 <= epsilon:
			return self.robot.interpolate(qa,qb,u)
		wm = workspace_interpolate(wa,wb,0.5)
		qm = self.robot.interpolate(qa,qb,0.5)
		q = self.solve(qm,wm)
		if q is None:
			return self.robot.interpolate(qa,qb,u)
		d1 = self.robot.distance(qa,q)
		d2 = self.robot.distance(q,qb)
		if max(d1,d2) > c*d0: return self.robot.interpolate(qa,qb,u)
		if u < 0.5:
			return self.interpolateEdgeBisection(qa,q,wa,wm,u*2,epsilon,c)
		else:
			return self.interpolateEdgeBisection(q,qb,wm,wb,(u-0.5)*2,epsilon,c)

	def validEdge(self,qa,qb,wa,wb):
		#return self.validEdgeBisection(qa,qb,wa,wb)
		return self.validEdgeLinear(qa,qb,wa,wb)
		#return self.validEdgeSimple(qa,qb,wa,wb)
					
	def score(self,iq,vleft,eleft):
		numEdgesAdded = 0
		degree = 0
		distance = float('inf')
		iw = self.Gq.node[iq]['windex']
		iconfig = self.Gq.node[iq]['config']
		jwlist = set([self.Gq.node[jq]['windex'] for jq in self.Gq.neighbors(iq)])
		for jw in jwlist:
			if jw not in vleft and ((iw,jw) in eleft or (jw,iw) in eleft):
				numEdgesAdded += 1
			degree += 1
			for jq in self.Gw.node[jw]['qlist']:
				if jq in self.Qsubgraph:
					distance = min(distance, robot.distance(self.Gq.node[jq]['config'],iconfig))
		return (numEdgesAdded,degree,-distance)

	def optimize(self,numIters=10):
		"""Optimizes the configurations of the current Qsubgraph to minimize
		joint-space path lengths using coordinate descent."""
		qs = dict((v,self.Gq.node[v]['config']) for v in self.Qsubgraph)
		numedges = 0
		neighbors = dict()
		for v in self.Qsubgraph:
			neighbors[v] = [w for w in self.Gq.neighbors(v) if w in self.Qsubgraph]
			numedges += len(neighbors[v])
		for iters in xrange(numIters):
			print "Iteration",iters
			sumdistances = 0
			c = ProgressUpdater(len(self.Qsubgraph),10)
			for v in self.Qsubgraph:
				c.update()
				if len(neighbors[v]) == 0: 
					continue
				x = self.Gw.node[self.Gq.node[v]['windex']]['params']
				q0 = qs[v]
				wneighbors = [self.Gq.node[w]['windex'] for w in neighbors[v]]
				qneighbors = [qs[w] for w in neighbors[v]]
				xneighbors = [self.Gw.node[i]['params'] for i in wneighbors]
				d0 = sum(self.robot.distance(q0,qw) for qw in qneighbors)
				qavg = robot_average(self.robot,qneighbors)
				#try to move toward average
				self.setIKProblem(x)
				self.ikSolverParams.startRandom = False
				maxTries = 10
				moved = False
				for tries in xrange(maxTries):
					self.robot.setConfig(qavg)
					q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
					if q != None:
						#check all edges from neighbors
						d = sum(self.robot.distance(q,qw) for qw in qneighbors)
						valid = True
						if d > d0:
							valid = False
						else:
							for qw,xw in zip(qneighbors,xneighbors):
								if not self.validEdge(q,qw,x,xw):
									valid = False
									break
						if valid:
							qs[v] = q
							sumdistances += d
							moved = True
							break
					#not moved, try subdividing
					qavg = self.robot.interpolate(q0,qavg,0.5)
				if not moved:
					#print "Unable to move configuration",v
					sumdistances += d0
			c.done()
			print "Changed average path length to",sumdistances/numedges
		#now add new q's and edges between to the graph
		start_node_count = self.Gw.graph['qnodes']
		iqmap = dict()
		for i,v in enumerate(self.Qsubgraph):
			iw = self.Gq.node[v]['windex']
			iqmap[v] = self.addNode(iw,qs[v])
		for v in self.Qsubgraph:
			for w in neighbors[v]:
				iw = self.Gq.node[v]['windex']
				jw = self.Gq.node[w]['windex']
				iq = iqmap[v]
				jq = iqmap[w]
				self.addEdge(iw,iq,jw,jq)
		self.Qsubgraph = set(iqmap.values())

	def sampleConflicts(self,NConfigsPerPoint=10):
		conflicts = set()
		for i,j,d in self.Gw.edges_iter(data=True):
			if len(self.Gw.node[i]['qlist']) == 0 or  len(self.Gw.node[j]['qlist']) == 0:
				continue
			found = any((eq[0] in self.Qsubgraph and eq[1] in self.Qsubgraph) for eq in d['qlist'])
			if not found:
				conflicts.add(i)
				conflicts.add(j)
		added = []
		for iw in conflicts:
			d = self.Gw.node[iw]
			self.setIKProblem(d['params'])
			self.ikSolverParams.startRandom = True
			for sample in xrange(NConfigsPerPoint):
				#solver will sample randomly
				q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
				if q is not None:
					iq = self.addNode(iw,q)
					added.append(iq)
			#try local solving from neighbors too
			self.ikSolverParams.startRandom = True
			Qseed = sum([self.Gw.node[jw]['qlist'] for jw in self.Gw.neighbors(iw) if jw in conflicts],[])
			if len(Qseed) > 0:
				for sample in xrange(NConfigsPerPoint):
					qseed = self.Gq.node[random.choice(Qseed)]['config']
					self.robot.setConfig(qseed)
					q = self.ikTemplate.solve(self.robot,self.ikSolverParams)
					if q is not None:
						iq = self.addNode(iw,q)
						added.append(iq)
		print len(added),"configs added for",len(conflicts),"workspace nodes"
		numEdgesAttempted = 0
		numEdgesAdded = 0
		c = ProgressUpdater(len(added),5)
		print "Connecting edges..."
		for iq in added:
			c.update()
			iw = self.Gq.node[iq]['windex']
			for jw in self.Gw.neighbors(iw):
				for jq in self.Gw.node[jw]['qlist']:
					numEdgesAttempted += 1
					if self.testAndConnect(iw,iq,jw,jq):
						numEdgesAdded += 1
		c.done()
		print "Attempted",numEdgesAttempted,"edge connections and added",numEdgesAdded

	def autoSetWorkspaceRadius(self):
		self.wRadius = 0
		for i,d in self.Gw.nodes(data=True):
			for j in self.Gw.neighbors(i):
				dist = workspace_distance(d['params'],self.Gw.node[j]['params'])
				self.wRadius = max(dist,self.wRadius)
		self.wRadius *= 1.01

	def interpolate(self,x,r='auto'):
		"""Uses inverse distance weighted interpolation at the workspace nodes to
		derive a reasonable guess for the IK solution at workspace setting x.
		"""
		q0 = self.robot.getConfig()
		self.setIKProblem(x)
		s = ik.solver(self.ikTemplate.objectives)
		if self.Gw.graph['qnodes'] == 0:
			return [q0]
		if r == 'auto':
			if self.wRadius == None:
				self.autoSetWorkspaceRadius()
			r = self.wRadius
			#if math.log(len(self.Gw))/len(self.Gw)*20*math.e
		nw = self.nnw.neighbors(x,r)
		if len(nw) == 0:
			xn,n = self.nnw.nearest(x)
			nw = [(xn,n)]
		if len(nw) == 1:
			#radius too small, look at neighbors
			nn = self.Gw.neighbors(nw[0][1])
			nw += [(self.Gw.node[n]['params'],n) for n in nn]
			r = max(workspace_distance(x,self.Gw.node[n]['params']) for (xw,n) in nw)
		#print "Point distances", [workspace_distance(self.Gw.node[n[1]]['params'],x) for n in nw]
		#print [self.Gw.node[n[1]] for n in nw]
		#if on the workspace graph, just interpolate as normal
		edgeTol = 1e-3
		Wsubgraph = self.Gw.subgraph([n[1] for n in nw])
		for (iw,jw) in Wsubgraph.edges_iter():
			xi = self.Gw.node[iw]['params']
			xj = self.Gw.node[jw]['params']
			assert len(xi) == len(x)
			(d,u) = segment_point_distance(xi,xj,x)
			#print "Edge distance", segment_point_distance(xi,xj,x)
			if d < edgeTol:
				for iq in self.Gw.node[iw]['qlist']:
					if iq in self.Qsubgraph:
						for jq in self.Gw.node[jw]['qlist']:
							if jq in self.Qsubgraph and self.Gq.has_edge(iq,jq):
								return [self.interpolateEdgeBisection(self.Gq.node[iq]['config'],self.Gq.node[jq]['config'],xi,xj,u)]

		#find the largest connected subgraph amongst the config nodes that
		#map to the k workspace nodes
		subset = []
		for (xw,iw) in nw:
			for iq in self.Gw.node[iw]['qlist']:
				if iq in self.Qsubgraph:
					subset.append(iq)
					#print iq,'=',iw,
		#print
		Sq = self.Gq.subgraph(subset)
		ccs = list(nx.connected_components(Sq))
		"""
		ccs = []
		used = dict((s,False) for s in subset)
		for s in subset:
			if used[s]: continue
			used[s] = True
			ccs.append([s])
			for t in Sq.neighbors(s):
				used[t] = True
				ccs[-1].append(t)
		"""
		#print ccs
		qs = []
		for cc0 in ccs:
			distances = []
			for iq in cc0:
				iw = self.Gq.node[iq]['windex']
				distances.append(workspace_distance(x,self.Gw.node[iw]['params']))
			mindist = min(distances)
			weights = {}
			if mindist < 1e-5:
				#very close to existing node, harmonic interpolation will give
				#numerical difficulty
				for (d,iq) in zip(distances,cc0):
					if d < 1e-5:
						weights[iq] = 1
			else:
				for (d,iq) in zip(distances,cc0):
					weights[iq] = ((r - d)/(r*d))**2
			if len(weights) == 1:
				iq = weights.keys()[0]
				q = self.Gq.node[iq]['config']
			else:
				sumw = 0
				q = [0.0]*len(self.Gq.node[0]['config'])
				for (w,iq) in sorted((-w,iq) for (iq,w) in weights.iteritems()):
					w = -w
					#print "weight",w,"config",self.Gq.node[iq]['config']
					if sumw == 0:
						q = self.Gq.node[iq]['config']
					else:
						#print q[1],
						q = self.robot.interpolate(q,self.Gq.node[iq]['config'],w/(w+sumw))
						#print "to",iq,"frac",w/(w+sumw),"res",q[1]
					sumw += w
				#print
				#print [str(iq)+":"+"%.2f"%(w/sumw,) for iq,w in weights.iteritems()]
			qs.append(q)
			#TEMP: only return the largest CC
			break
		robot = self.robot
		if hasattr(self,'lastqs'):
			for (a,b,cc) in zip(qs,self.lastqs,ccs):
				if self.robot.distance(a,b) > 1e-1:
					print "Big change in configuration, magnitude:",self.robot.distance(a,b)
					#print zip(a,b)
					#print cc
		self.lastqs = qs
		return qs

	def eval(self,x):
		"""Solves the IK problem specified by the workspace setting x"""
		"""
		res = self.nnw.nearest(x)
		if res is None: return None
		self.setIKProblem(x)
		#seed from res
		nw = res[1]
		qlist = self.Gw.node[nw]['qlist']
		if len(self.Qsubgraph) > 0:
			#have an assignment
			qlist = [nq for nq in qlist if nq in self.Qsubgraph]
		for nq in qlist:
			self.robot.setConfig(self.Gq.node[nq]['config'])
			self.ikSolverParams.startRandom = False
			res = self.ikTemplate.solve(self.robot,self.ikSolverParams)
			self.ikSolverParams.startRandom = True
			if res:
				return res
		"""
		seeds = self.interpolate(x)
		self.setIKProblem(x)
		if len(seeds) == 0:
			self.ikSolverParams.startRandom = False
			res = self.ikTemplate.solve(self.robot,self.ikSolverParams)
			self.ikSolverParams.startRandom = True
			return res
		qOrig = self.robot.getConfig()
		best = None
		dbest = float('inf')
		self.ikSolverParams.startRandom = False
		for q in seeds:
			self.robot.setConfig(q)
			res = self.ikTemplate.solve(self.robot,self.ikSolverParams)
			if res:
				if self.robot.distance(res,qOrig) < dbest:
					best = res
					dbest = self.robot.distance(res,qOrig)
		return best

	def prune(self,visibility=False,keepConflicts=False):
		"""Removes all q nodes not in the qsubgraph"""
		if keepConflicts:
			for a,b,d in self.Gw.edges_iter(data=True):
				if (a < b and len(d['qlist']) > 0):
					satisfied = False
					for (iq,jq) in d['qlist']:
						if iq not in self.Gw.node[a]['qlist']:
							#swap?
							iq,jq = jq,iq
						try:
							assert iq in self.Gw.node[a]['qlist']
							assert jq in self.Gw.node[b]['qlist']
							if iq in self.Qsubgraph and jq in self.Qsubgraph:
								satisfied = True
								break
						except AssertionError:
							print "Warning, edge configuration value",iq,jq,"is not valid index in nodes"
							print "Start node config list",self.Gw.node[a]['qlist']
							print "End node config list",self.Gw.node[b]['qlist']
					if not satisfied:
						for (iq,jq) in d['qlist']:
							self.Qsubgraph.add(iq)
							self.Qsubgraph.add(jq)
		#TODO update Gw's 'qnodes' counter
		newGq = self.Gq.subgraph(self.Qsubgraph)
		qmap = dict()
		for i,n in enumerate(self.Qsubgraph):
			qmap[n] = i
		for n,d in self.Gw.nodes(data=True):
			if len(d['qlist']) == 0: continue
			d['qlist'] = [qmap[i] for i in d['qlist'] if i in qmap]
			d['qccs'] = DisjointSet()
			for iq in d['qlist']:
				d['qccs'].add(iq)
		for u,v,d in self.Gw.edges_iter(data=True):
			if len(d['qlist']) == 0: continue
			d['qlist'] = [(qmap[i],qmap[j]) for i,j in d['qlist'] if (i in qmap and j in qmap)]
		self.Gq = nx.relabel_nodes(newGq,qmap)
		#self.infeasibleEdges = set([(qmap[i],qmap[j]) for i,j in self.infeasibleEdges if (i in qmap and j in qmap)])
		self.Qsubgraph = set(range(self.Gw.graph['qnodes']))
		if visibility:
			self.constructSelfMotionManifolds()
		print "Keeping",self.Gw.graph['qnodes'],"nodes and",self.Gq.number_of_edges(),"configuration edges"

	def select(self,x,nodeRadius,edgeRadius,cc_filter=True,use_resolution='auto'):
		if use_resolution == 'auto':
			if len(self.Qsubgraph) == 0:
				use_resolution = True
		demin = float('inf')
		emin = None
		for aw,bw,data in self.Gw.edges_iter(data=True):
			if len(data['qlist']) == 0: 
				continue
			a = self.Gw.node[aw]['params']
			b = self.Gw.node[bw]['params']
			(d,u) = segment_point_distance(a,b,x)
			if d < demin:
				demin = d
				emin = (aw,bw,u)
		if demin > edgeRadius:
			#print "Closest edge:",demin
			emin = None
		if emin:
			(aw,bw,u) = emin
			if use_resolution:
				ares = [aq for aq in self.Gw.node[aw]['qlist'] if aq in self.Qsubgraph]
				bres = [bq for bq in self.Gw.node[bw]['qlist'] if bq in self.Qsubgraph]
				res = []
				for aq in ares:
					for bq in bres:
						if self.Gq.has_edge(aq,bq) or self.Gq.has_edge(bq,aq):
							res.append(self.robot.interpolate(self.Gq.node[aq]['config'],self.Gq.node[bq]['config'],u))
						else:
							#whatever, do it anyway
							#res.append(self.robot.interpolate(self.Gq.node[aq]['config'],self.Gq.node[bq]['config'],u))
							pass
				if len(res) == 0:
					print "Nothing in Qsubgraph???"
				return res
			elif cc_filter:
				#get connected component
				configs = set(aq for (aq,bq) in self.Gw.edge[aw][bw]['qlist']) | set(bq for (aq,bq) in self.Gw.edge[aw][bw]['qlist'])
				Gsub = self.Gq.subgraph(configs)
				ccs = list(nx.connected_components(Gsub))
				touched = set()
				res = []
				for (aq,bq) in self.Gw.edge[aw][bw]['qlist']:
					aindex = [i for i,cc in enumerate(ccs) if aq in cc]
					bindex = [i for i,cc in enumerate(ccs) if bq in cc]
					assert len(aindex)==1 and len(bindex)==1
					assert aindex[0] == bindex[0]
					if aindex[0] in touched: continue
					touched.add(aindex[0])
					res.append(self.robot.interpolate(self.Gq.node[aq]['config'],self.Gq.node[bq]['config'],u))
				return res
			else:
				#return all
				res = []
				for (aq,bq) in self.Gw.edge[aw][bw]['qlist']:
					res.append(self.robot.interpolate(self.Gq.node[aq]['config'],self.Gq.node[bq]['config'],u))
				return res
		dnmin = float('inf')
		nmin = None
		for (iw,data) in self.Gw.nodes(data=True):
			if len(data['qlist']) == 0: 
				continue
			d = workspace_distance(x,data['params']) 
			if d < dnmin:
				nmin = iw
				dnmin = d
		if dnmin > nodeRadius:
			#print "Node distance",dnmin
			nmin = None
		if nmin:
			if use_resolution:
				qs = [self.Gq.node[iq]['config'] for iq in self.Gw.node[nmin]['qlist'] if iq in self.Qsubgraph]
				return qs
			elif cc_filter:
				#get connected component
				configs = set(self.Gw.node[nmin]['qlist'])
				for n in self.Gw.neighbors(nmin):
					configs |= set(self.Gw.node[n]['qlist'])
				Gsub = self.Gq.subgraph(configs)
				ccs = list(nx.connected_components(Gsub))
				res = []
				touched = set()
				for v in self.Gw.node[nmin]['qlist']:
					vindex = [i for i,cc in enumerate(ccs) if v in cc]
					assert len(vindex) == 1
					if vindex[0] in touched: continue
					touched.add(vindex[0])
					res.append(self.Gq.node[v]['config'])
				return res
			else:
				return [self.Gq.node[v]['config'] for v in self.Gw.node[nmin]['qlist']]
		return None

	def walkPath(self,x1,x2=None):
		if x2 == None:
			xsrc = self.getIKParameters(self.robot.getConfig())
			xtgt = x1
		else:
			xsrc = x1
			xtgt = x2
		if isinstance(xsrc,int):
			xsrc = self.Gw.node[xsrc]['params']
		if isinstance(xtgt,int):
			xtgt = self.Gw.node[xtgt]['params']
		print xsrc,xtgt

		#get graph of Qsubgraph connected nodes in workspace
		Gconnected = nx.Graph()
		for q in self.Qsubgraph:
			iw = self.Gq.node[q]['windex']
			assert not Gconnected.has_node(iw)
			Gconnected.add_node(iw,qindex=q,config=self.Gq.node[q]['config'])
		for q in self.Qsubgraph:
			iw = self.Gq.node[q]['windex']
			for jw in self.Gw.neighbors(iw):
				if Gconnected.has_node(jw):
					if self.Gq.has_edge(Gconnected.node[iw]['qindex'],Gconnected.node[jw]['qindex']):
						cost = linf_distance(Gconnected.node[iw]['config'],Gconnected.node[jw]['config'])
						Gconnected.add_edge(iw,jw,cost=cost)
						Gconnected.add_edge(jw,iw,cost=cost)
		path = []

		#find closest start workspace node
		res = self.nnw.nearest(xsrc)
		if res is None:
			print "No workspace node available at start point"
			return None
		nsrc = res[1]
		if xsrc != self.Gw.node[nsrc]['params']:
			if not Gconnected.has_node(nsrc):
				print "Start workspace node not in resolution"
				return None
			path.append(self.robot.getConfig())

		#find closest target workspace node
		res = self.nnw.nearest(xtgt)
		if res is None:
			print "No workspace node available at target point"
			return None
		ntgt = res[1]

		print "Src node index",nsrc
		print "Tgt node index",ntgt
		pathindices = nx.shortest_path(Gconnected,nsrc,ntgt,'cost')
		print "Path is",pathindices
		for idx in pathindices:
			path.append(Gconnected.node[idx]['config'])

		if xtgt != self.Gw.node[ntgt]['params']:
			self.robot.setConfig(Gconnected.node[xtgt]['config'])
			for q in self.interpolate(xtgt):
				qsolve = self.solve(q,x)
				if qsolve != None:
					path.append(qsolve)
					break
		return path

	def collapse(self):
		"""Collapses the number of configurations per workspace node, such that there is
		at most 1 configuration per 1-ring connected component.
		"""
		#all Gq nodes to keep
		nodes = set()
		#map from Gw nodes to kept Gq nodes
		wnodes = dict()
		for n,d in self.Gw.nodes(data=True):
			configs = set(d['qlist'])
			for m in self.Gw.neighbors(n):
				configs |= set(self.Gw.node[m]['qlist'])
			Gsub = self.Gq.subgraph(configs)
			ccs = list(nx.connected_components(Gsub))
			touched = set()
			nccs = [[] for cc in ccs]
			for v in d['qlist']:
				vindex = [i for i,cc in enumerate(ccs) if v in cc]
				assert len(vindex) == 1
				nccs[vindex[0]].append(v)
				if vindex[0] in touched: continue
				touched.add(vindex[0])
			wnodes[n] = []
			for cc in nccs:
				if len(cc)==0: continue
				#pick the node that has the largest degree OR is already in the nodes set
				ncount,degree,v = max([(len(set(self.Gq.neighbors(v)) & nodes),len(self.Gq.neighbors(v)),v) for v in cc])
				nodes.add(v)
				wnodes[n].append(v)
		#ensure that the nodes cover the 1-ring of each node 
		for i,j,d in self.Gw.edges_iter(data=True):
			ecount = 0
			for u,v in d['qlist']:
				if u in nodes and v in nodes:
					ecount += 1
			if len(d['qlist']) != 0 and ecount == 0:
				#empty, need to add 1 ring
				for u,v in d['qlist']:
					if u in nodes or v in nodes:
						nodes.add(u)
						nodes.add(v)
		qmap = dict()
		for i,n in enumerate(nodes):
			qmap[n] = i
		newGq = self.Gq.subgraph(nodes)
		self.Gq = nx.relabel_nodes(newGq,qmap)
		print "Keeping",self.Gw.graph['qnodes'],"nodes and",self.Gq.number_of_edges(),"configuration edges"
		for n,d in self.Gw.nodes(data=True):
			d['qlist'] = [qmap[v] for v in wnodes[n]]
		#subselect all edges
		for i,j,d in self.Gw.edges_iter(data=True):
			newe = []
			for u,v in d['qlist']:
				if u in nodes and v in nodes:
					newe.append((qmap[u],qmap[v]))
			d['qlist'] = newe
		self.Qsubgraph = set()
		



class RedundancyProgram(GLBaseClass):
	def __init__(self,world,robot):
		if KLAMPT_VERSION >= 0.7:
			GLBaseClass.__init__(self)
		else:
			GLBaseClass.__init__(self,"Redundancy resolution test")
		self.world = world
		self.robot = robot
		self.rr = RedunancyResolver(robot)
		self.mode = 'interpolate'
		self.configs = None
		self.folder = None
		self.settings = None
		self.drawCSpaceRoadmap = False
		self.drawWorkspaceRoadmap = False
		self.solveConstraint = True
		self.clippingplanes = (0.1,50)
		self.rotationAsDepth = False
		self.pointWidget = PointPoser()
		self.xformWidget = TransformPoser()
		self.roadmapDisplayList = CachedGLObject()
		self.disconnectionDisplayList = CachedGLObject()
		self.movie_frame = None
		self.movie_rotate = False
		self.walk_path = None
		self.walk_workspace_path = None
		self.walk_progress = None
		self.temp_config = None

	def initialize(self):
		GLBaseClass.initialize(self)
		self.clearColor = [1,1,1,1]

		assert self.rr.domain != None
		print self.rr.domain
		if len(self.rr.domain[0]) == 6:
			#if a 2D problem and want to show depth, turn this to true
			self.rotationAsDepth = True
			link = self.rr.ikTemplate.objectives[0].link()
			self.xformWidget.enableTranslation(True)
			self.xformWidget.enableRotation(True)
			print "Initial transform",self.rr.robot.link(link).getTransform()
			#self.xformWidget.set(*self.rr.robot.link(link).getTransform())
			if KLAMPT_VERSION >= 0.7:
				self.addWidget(self.xformWidget)
			else:
				self.widgetMaster.add(self.xformWidget)
		else:
			self.rotationAsDepth = False
			link = self.rr.ikTemplate.objectives[0].link()
			local,world = self.rr.ikTemplate.objectives[0].getPosition()
			print "Initial position",self.rr.robot.link(link).getWorldPosition(local)
			self.pointWidget.set(self.rr.robot.link(link).getWorldPosition(local))
			if KLAMPT_VERSION >= 0.7:
				self.addWidget(self.pointWidget)
			else:
				self.widgetMaster.add(self.pointWidget)
		return True

	def run(self):
		vis.run(self)

	def workspaceToPoint(self,x):
		if self.rotationAsDepth:
			return (x[0],x[4],x[2])
		else:
			return x[:3]

	def display(self):
		if self.walk_workspace_path != None:
			if self.temp_config:
				self.robot.setConfig(self.temp_config)
				glDisable(GL_DEPTH_TEST)
				for i in xrange(self.robot.numLinks()):
					self.robot.link(i).appearance().setColor(1,0,0,0.5)
				self.robot.drawGL()
				for i in xrange(self.robot.numLinks()):
					self.robot.link(i).appearance().setColor(0.5,0.5,0.5,1)
				glEnable(GL_DEPTH_TEST)
			glColor3f(1,1,0)
			glLineWidth(5.0)
			glBegin(GL_LINE_STRIP)
			for w in self.walk_workspace_path.milestones:
				if len(w) == 2:
					glVertex2f(w[0],w[1])
				else:
					glVertex3f(w[0],w[1],w[2])
			glEnd()
			glLineWidth(1.0)

		if self.configs == None:
			self.robot.drawGL()
		else:
			for q in self.configs:
				self.robot.setConfig(q)
				self.robot.drawGL()
		if self.drawCSpaceRoadmap:
			#draw workspace - c-space roadmap
			glDisable(GL_LIGHTING)
			glEnable(GL_BLEND)
			glDisable(GL_DEPTH_TEST)
			glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
			glPointSize(5.0)
			xshift = -4
			glBegin(GL_POINTS)
			for n,d in self.rr.Gq.nodes(data=True):
				x = self.rr.Gw.node[d['windex']]['params']
				q = d['config']
				depth = (-q[2] + 1.0)*0.25
				if n in self.rr.Qsubgraph:
					continue
				else:
					glColor4f(1,1,depth,0.2)
				glVertex3f(x[2]+xshift,q[2],q[1])
			glEnd()
			glBegin(GL_LINES)
			for i,j,d in self.rr.Gq.edges_iter(data=True):
				if i > j: continue
				xi = self.rr.Gw.node[self.rr.Gq.node[i]['windex']]['params']
				xj = self.rr.Gw.node[self.rr.Gq.node[j]['windex']]['params']
				qi = self.rr.Gq.node[i]['config']
				qj = self.rr.Gq.node[j]['config']
				depth = (-qi[2] + 1.0)*0.25
				a = 1.0/(1.0+5.0*self.rr.robot.distance(qi,qj))
				if i in self.rr.Qsubgraph and j in self.rr.Qsubgraph:
					continue
				else:
					glColor4f(1,1,depth,a)
				glVertex3f(xi[2]+xshift,qi[2],qi[1])
				glVertex3f(xj[2]+xshift,qj[2],qj[1])
			glEnd()
			#draw path
			glColor3f(1,0.5,0)
			glBegin(GL_POINTS)
			for n,d in self.rr.Gq.nodes(data=True):
				x = self.rr.Gw.node[d['windex']]['params']
				q = d['config']
				if n in self.rr.Qsubgraph:
					glVertex3f(x[2]+xshift,q[2],q[1])
			glEnd()
			glBegin(GL_LINES)
			for i,j,d in self.rr.Gq.edges_iter(data=True):
				if i > j: continue
				xi = self.rr.Gw.node[self.rr.Gq.node[i]['windex']]['params']
				xj = self.rr.Gw.node[self.rr.Gq.node[j]['windex']]['params']
				qi = self.rr.Gq.node[i]['config']
				qj = self.rr.Gq.node[j]['config']
				if i in self.rr.Qsubgraph and j in self.rr.Qsubgraph:
					glVertex3f(xi[2]+xshift,qi[2],qi[1])
					glVertex3f(xj[2]+xshift,qj[2],qj[1])
			glEnd()
			glEnable(GL_DEPTH_TEST)
			glDisable(GL_BLEND)

		#draw workspace graph
		if self.drawWorkspaceRoadmap:
			#draw Qsubgraph duplicates
			for n,d in self.rr.Gw.nodes(data=True):
					nsubset = [iq for iq in d['qlist'] if iq in self.rr.Qsubgraph]
					if len(nsubset) > 1:
						for iq in nsubset:
							self.robot.setConfig(self.rr.Gq.node[iq]['config'])
							self.robot.drawGL()
			def drawWorkspaceRoadmap():
				if self.rr.Gq.number_of_nodes() == 0:
					glDisable(GL_LIGHTING)
					glPointSize(5.0)
					glColor3f(1,0,0)
					glBegin(GL_POINTS)
					for n,d in self.rr.Gw.nodes(data=True):
						glVertex3fv(self.workspaceToPoint(d['params']))
					glEnd()
					glColor3f(1,0.5,0)
					glBegin(GL_LINES)
					for (i,j) in self.rr.Gw.edges_iter():
						glVertex3fv(self.workspaceToPoint(self.rr.Gw.node[i]['params']))
						glVertex3fv(self.workspaceToPoint(self.rr.Gw.node[j]['params']))
					glEnd()
				else:
					#draw workspace graph colored with the number of nodes in Gq
					maxn = max(len(d['qlist']) for n,d in self.rr.Gw.nodes(data=True))
					maxe = max(len(d['qlist']) for i,j,d in self.rr.Gw.edges_iter(data=True))
					maxn = max(maxn,1)
					maxe = max(maxe,1)
					glDisable(GL_LIGHTING)
					glPointSize(5.0)
					glBegin(GL_POINTS)
					for n,d in self.rr.Gw.nodes(data=True):
						if len(d['qlist']) == 0:
							continue
						u = float(len(d['qlist']))/float(maxn)
						nsubset = sum(1 for iq in d['qlist'] if iq in self.rr.Qsubgraph)
						if nsubset > 1:
							glColor3f(1,0,1)
						else:
							glColor3f(u,nsubset*0.5,0)
						glVertex3fv(self.workspaceToPoint(d['params']))
					glEnd()
					glBegin(GL_LINES)
					for (i,j,d) in self.rr.Gw.edges_iter(data=True):
						if len(self.rr.Gw.node[i]['qlist']) == 0 or len(self.rr.Gw.node[j]['qlist']) == 0:
							continue
						nsubset = sum(1 for (ia,ib) in d['qlist'] if (ia in self.rr.Qsubgraph and ib in self.rr.Qsubgraph))
						u = float(len(d['qlist']))/float(maxe)
						r,g,b = u,u,0
						if nsubset == 0:
							r,g,b = 1,0,1
						glColor3f(r,g,b)
						glVertex3fv(self.workspaceToPoint(self.rr.Gw.node[i]['params']))
						glVertex3fv(self.workspaceToPoint(self.rr.Gw.node[j]['params']))
					glEnd()
					"""
					glEnable(GL_LIGHTING)
					q0 = self.robot.getConfig()
					for iw,d in self.rr.Gw.nodes(data=True):
						qs = [iq for iq in d['qlist'] if iq in self.rr.Qsubgraph]
						if len(qs) > 1:
							for iq in qs:
								self.robot.setConfig(self.rr.Gq.node[iq]['config'])
								self.robot.drawGL()
					self.robot.setConfig(q0)
					glDisable(GL_LIGHTING)
					"""
			#self.roadmapDisplayList.draw(drawWorkspaceRoadmap)
			drawWorkspaceRoadmap()
		else:
			#render boundaries only
			def drawDisconnections():
				bmin,bmax = self.rr.domain
				active = [i for i,(a,b) in enumerate(zip(bmin,bmax)[:3]) if b!=a]
				if len(active)==3:
					glEnable(GL_LIGHTING)
					glEnable(GL_BLEND)
					glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
					if self.walk_workspace_path != None:
						gldraw.setcolor(1,0,1,0.25)
					else:
						gldraw.setcolor(1,0,1,0.5)
				else:
					glDisable(GL_LIGHTING)
					glEnable(GL_BLEND)
					glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
					glColor4f(1,0,1,0.5)
				def getpoint(i):
					return self.rr.Gw.node[i]['params'][:3]
				useboundary = (len(active)==2)
				pts = []
				epts = set()
				es = dict()
				vs = dict()
				for i,d in self.rr.Gw.nodes(data=True):
					if len(d['qlist']) == 0:
						if useboundary:
							epts.add(len(pts))
							es[len(pts)] = (-1,-1)
							vs[len(pts)] = i
							pts.append(getpoint(i))
						continue
					vs[len(pts)] = i
					pts.append(getpoint(i))
					disconnections = []
					if not useboundary:
						#don't want to draw boundary
						neighbors = [j for j in self.rr.Gw.neighbors(i) if len(self.rr.Gw.node[j]['qlist']) > 0 and j > i]
					else:
						#want to draw boundary
						neighbors = [j for j in self.rr.Gw.neighbors(i)]
					for j in neighbors:
						#if len(self.rr.Gw.node[j]['qlist']) == 0:
						#    continue
						nsubset = any(True for (ia,ib) in self.rr.Gw.edge[i][j]['qlist'] if (ia in self.rr.Qsubgraph and ib in self.rr.Qsubgraph))
						if not nsubset:
							disconnections.append(j)
					if len(disconnections) == 0:
						continue
					for j in disconnections:
						#add midpoints of disconnections
						epts.add(len(pts))
						es[len(pts)] = (i,j)
						pts.append(vectorops.interpolate(getpoint(i),getpoint(j),0.5))
						"""
						#add quarter points?
						epts.add(len(pts))
						es[len(pts)] = (i,j)
						pts.append(vectorops.interpolate(getpoint(i),getpoint(j),0.25))
						epts.add(len(pts))
						es[len(pts)] = (i,j)
						pts.append(vectorops.interpolate(getpoint(i),getpoint(j),0.75))
						"""
					for j in self.rr.Gw.neighbors(i):
						#add successful midpoints
						if j < i or j in disconnections: continue
						if len(active) == 3:
							pts.append(vectorops.interpolate(getpoint(i),getpoint(j),0.5))
				if len(epts)==0: return
				hpts = [[x[a] for a in active] for x in pts]

				try:
					delaunay = scipy.spatial.Delaunay(hpts)
					if len(active)!=3:
						glLineWidth(2.0)
						glBegin(GL_LINES)
					for simplex in delaunay.simplices:
						drawable = any(v in epts for v in simplex)
						#if it's a conflicting edge along the workspace graph, don't draw it
						if not drawable:
							continue
						for i in range(len(simplex)):
							if len(active)==3:
								tri = [v for j,v in enumerate(simplex) if i!=j]
								#if all(a not in epts for a in tri):
								#	continue
								if len([a for a in tri if a in epts]) < 1:
									continue
								conflictEdge = False
								vtri = [vs.get(v,-1) for v in tri]
								for a in tri:
									if a in es and (-1 in es[a] or es[a][0] in vtri or es[a][1] in vtri ):
										conflictEdge = True
										break
								if conflictEdge:
									continue
								centroid = vectorops.div(vectorops.add(*[hpts[v] for v in tri]),len(tri))
								params = [(x-a)/(b-a) for (x,a,b) in zip(centroid,self.rr.domain[0],self.rr.domain[1])]
								if self.walk_workspace_path != None:
									gldraw.setcolor(params[0],params[1],params[2],0.25)
								else:
									gldraw.setcolor(params[0],params[1],params[2],0.5)
								gldraw.triangle(*[hpts[v] for v in tri])
							else:
								n = (i+1)%len(simplex)
								a,b = simplex[i],simplex[n]
								#if not (a in epts or b in epts):
								#	continue
								if not (a in epts and b in epts):
									continue
								vseg = (vs.get(a,-1),vs.get(b,-1))
								conflictEdge = (a in epts) and (-1 in es[a] or (es[a][0] in vseg) or (es[a][1] in vseg))
								conflictEdge = conflictEdge or (b in epts) and (-1 in es[b] or (es[b][0] in vseg) or (es[b][1] in vseg))
								if conflictEdge:
									continue
								spts = hpts[a],hpts[b]
								glVertex3f(spts[0][0],0,spts[0][1])
								glVertex3f(spts[1][0],0,spts[1][1])
					if len(active)!=3:
						glEnd()
						glLineWidth(1.0)
						err = glGetError()
						if err != 0:
							raise ValueError("OpenGL had an error: %d"%(err,))
				except Exception as e:
					print "Error computing Delaunay triangulation"
					print e

				#glDisable(GL_BLEND)
				"""
				#this code draws quads at each failed edge
				gldraw.setcolor(1,0,1,0.5)
				for (i,j,d) in self.rr.Gw.edges_iter(data=True):
					if len(d['qlist']) == 0:
						continue
					nsubset = any(True for (ia,ib) in d['qlist'] if (ia in self.rr.Qsubgraph and ib in self.rr.Qsubgraph))
					if not nsubset:
						a = self.rr.Gw.node[i]['params'][:3]
						b = self.rr.Gw.node[j]['params'][:3]
						c = vectorops.interpolate(a,b,0.5)
						d = vectorops.sub(b,a)
						r = vectorops.norm(d)
						R = so3.canonical(vectorops.div(d,r))
						z,x,y = so3.matrix(so3.inv(R))
						r = r/2
						x,y = vectorops.add(x,y),vectorops.sub(x,y)
						q1 = vectorops.madd(c,x,-r)
						q2 = vectorops.madd(c,y,r)
						q3 = vectorops.madd(c,x,r)
						q4 = vectorops.madd(c,y,-r)
						gldraw.quad(q1,q2,q3,q4)
						glDisable(GL_LIGHTING)
						glColor3f(1,1,0)
						glBegin(GL_LINES)
						glVertex3fv(a)
						glVertex3fv(b)
						glEnd()
						glEnable(GL_LIGHTING)
				"""
			self.disconnectionDisplayList.draw(drawDisconnections)

		GLBaseClass.display(self)

	def mousefunc(self,button,state,x,y):
		GLBaseClass.mousefunc(self,button,state,x,y)
		self.do_click(x,y)

	def do_click(self,x,y):
		if self.pointWidget.hasFocus():
			p = self.pointWidget.get()
			if len(self.rr.domain[0]) == 6:
				x = p+[0,0,0]
			else:
				x = p
			if self.mode == 'interpolate':
				self.configs = self.rr.interpolate(x)
				if self.solveConstraint:
					solved_configs = [self.rr.solve(q,x) for q in self.configs]
					self.configs = [qs if qs is not None else qorig for (qs,qorig) in zip(solved_configs,self.configs)]
			else:
				self.configs = self.rr.select(x,0.05,0.02)
			self.refresh()
		if self.xformWidget.hasFocus():
			R,t = self.xformWidget.get()
			m = so3.moment(R)
			x = t + m
			if self.mode == 'interpolate':
				self.configs = self.rr.interpolate(x)
				if self.solveConstraint:
					solved_configs = [self.rr.solve(q,x) for q in self.configs]
					self.configs = [qs if qs is not None else qorig for (qs,qorig) in zip(solved_configs,self.configs)]
			else:
				self.configs = self.rr.select(x,0.05,0.02)
			self.refresh()

	def motionfunc(self,x,y,dx,dy):
		GLBaseClass.motionfunc(self,x,y,dx,dy)
		self.do_click(x,y)
	#def motionfunc(self,dx,dy):
	#	GLBaseClass.motionfunc(self,dx,dy)
	#	self.do_click(self.lastx,self.lasty)

	def keyboardfunc(self,c,x,y):
		if c == 'h':
			print "Keyboard help:"
			print "- [space]: samples more points in the workspace (not very functional)"
			print "- q: samples more points in the configuration space (10 per workspace point)"
			print "- Q: samples more points in the configuration space attempting to add more in poorly-sampled areas"
			print "- C: samples more configurations at conflicting workspace edges"
			print "- c: collapses the nearby connected components of the configuration space graph"
			print "- u: performs a CSP assignment"
			print "- d: min-conflicts descent of the assignment"
			print "- r: randomizes the assignment and then descends"
			print "- p: prunes the entire configuration space graph except for the assignment"
			print "- P: performs pointwise redundancy resolution"
			print "- o: 10 iterations of coordinate descent optimization of overall path length"
			print "- i: toggles between interpolation mode and graph inspection mode"
			print "- g: toggles drawing the workspace graph"
			print "- G: toggles drawing the C-space roadmap (only valid for planar problems)"
			print "- s: saves roadmap and resolution to disk"
			print "- w: performs a walk to a random workspace node"
			print "- m: saves a real-time movie"
			print "- M: saves a 360 spin movie"
		if c == ' ':
			print "Sampling workspace..."
			self.rr.sampleWorkspace([-3,0,-3],[3,0,3],10)
		elif c == 'q':
			print "Sampling configuration space..."
			self.rr.sampleConfigurationSpace(10)
		elif c == 'Q':
			print "Sampling configuration space..."
			self.rr.sampleConfigurationSpace(10,biased=True)
		elif c == 'C':
			print "Sampling configurations at conflicting edges..."
			self.rr.sampleConflicts()
			self.rr.printStats()
		elif c == 'u':
			print "Performing unique assignment..."
			self.rr.uniqueAssignment()
			self.rr.printStats()
		elif c == 'r':
			print "Doing random descent..."
			self.rr.randomDescentAssignment(True)
			self.rr.printStats()
		elif c == 'd':
			print "Doing descent..."
			self.rr.randomDescentAssignment(False)
			self.rr.printStats()
		elif c == 'i':
			if self.mode == 'interpolate':
				self.mode = 'inspect'
			else:
				self.mode = 'interpolate'
			print "Toggled visualization mode to",self.mode
		elif c == 's':
			print "Saving results..."
			if self.folder == None:
				raise RuntimeError("folder element was not set?")
			self.rr.save(self.folder)
			if self.settings != None:
				f = open(os.path.join(self.folder,'settings.json'),'w')
				json.dump(self.settings,f)
				f.close()
		elif c == 'p':
			print "Pruning..."
			self.rr.prune()
			self.rr.printStats()
		elif c == 'c':
			print "Collapsing..."
			self.rr.collapse()
			self.rr.printStats()
		elif c == 'P':
			print "Pointwise assignment..."
			self.rr.pointwiseAssignment()
			self.rr.printStats()
		elif c == 'o':
			print "Optimizing..."
			self.rr.optimize()
			self.rr.printStats()
		elif c == 'g':
			self.drawCSpaceRoadmap = not self.drawCSpaceRoadmap
		elif c == 'G':
			self.drawWorkspaceRoadmap = not self.drawWorkspaceRoadmap
		elif c == 'm':
			if self.movie_frame is None:
				self.movie_frame = 0
				self.movie_rotate = False
			else:
				self.movie_frame = None
		elif c == 'M':
			if self.movie_frame is None:
				self.movie_frame = 0
				self.movie_rotate = True
			else:
				self.movie_frame = None
		elif c == 'w':
			for iters in range(10):
				qtgt = random.choice([w for w in self.rr.Qsubgraph])
				wtgt = self.rr.Gq.node[qtgt]['windex']
				#update widget
				xtgt = self.rr.Gw.node[wtgt]['params']
				xtgt = [20]*3
				self.pointWidget.set(xtgt)
				R,t = self.xformWidget.get()
				self.xformWidget.set(R,xtgt)

				#get current config
				self.robot.setConfig(self.configs[0])
				try:
					p = self.rr.walkPath(wtgt)
				except:
					print "Trying with a new random point"
					continue
				self.walk_workspace_path = None
				if p != None:
					t = 0
					times = []
					for i,q in enumerate(p):
						times.append(t)
						if i+1 < len(p):
							t += linf_distance(q,p[i+1])
						#t += 0.1
					xw = []
					for q in p:
						xw.append(self.rr.getIKParameters(q))
					self.walk_workspace_path = trajectory.Trajectory(times,xw)
					self.walk_path = trajectory.RobotTrajectory(self.robot,times,p)
					self.walk_progress = 0
					if self.temp_config == None:
						self.temp_config = p[0]
				break
		self.refresh()
		self.roadmapDisplayList.markChanged()
		self.disconnectionDisplayList.markChanged()

	def idle(self):
		t0 = time.time()
		if self.movie_frame is not None:
			numframes = 180
			if self.movie_rotate:
				self.camera.rot[2] += math.pi*2 / numframes
			self.save_screen("frame%03d.ppm"%(self.movie_frame))
			self.movie_frame += 1
			if self.movie_rotate and self.movie_frame >= numframes:
				self.movie_frame = None
		if self.walk_path != None:
			self.walk_progress += 0.02
			if self.walk_progress >= self.walk_path.times[-1]:
				self.configs = [self.walk_path.milestones[-1]]
				self.walk_path = None
			else:
				#self.configs = [self.walk_path.eval(self.walk_progress)]
				self.configs = [self.rr.eval(self.walk_workspace_path.eval(self.walk_progress))]
				if self.configs[0] == None:
					self.configs = []
				u = self.walk_progress / self.walk_path.times[-1]
				qstraight = self.rr.solve(self.temp_config,vectorops.interpolate(self.walk_workspace_path.milestones[0],self.walk_workspace_path.milestones[-1],u))
				if qstraight and (self.rr.ikTemplate.feasibilityTest==None or self.rr.ikTemplate.feasibilityTest(qstraight)):
					self.temp_config = qstraight
			self.refresh()
			t1 = time.time()
			if t1 - t0 < 0.02:
				time.sleep(0.02 - (t1-t0))

def toUtf8(input):
    if isinstance(input, dict):
        return {toUtf8(key): toUtf8(value)
                for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [toUtf8(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def make_program(problem,options,settings_file=None,load_output=True):
	"""Loads a file from disk and returns an appropriately configured
	RedundancyProgram.
	"""
	f = None
	if settings_file == None:
		import glob
		#try opening the first json file there?
		files = glob.glob(os.path.join('problems',problem,"*.json"))
		if len(files) == 0:
			raise ValueError("No valid json files in folder problems/"+problem)
		settings_file = files[0]
	else:
		if problem != None:
			settings_file = os.path.join('problems',problem,settings_file)
	f = open(settings_file,'r')
	pdef = toUtf8(json.load(f))
	f.close()
	#override with options
	pdef.update(options)

	klampt_directory = '/home/motion/Klampt/'
	#klampt_directory = '/Program Files (x86)/Klampt/'
	orientation = pdef.get('orientation','free')
	filename = pdef['filename']
	if not os.path.exists(filename) and os.path.exists(os.path.join(klampt_directory,filename)):
		filename = os.path.join(klampt_directory,filename)
	domain = pdef['domain']
	eelocal = pdef.get('eelocal',(0,0,0))
	useCollision = pdef.get('useCollision',True)
	link = pdef.get('link',None)
	links = pdef.get('links',None)
	fixedOrientation = pdef.get('fixedOrientation',None)
	if isinstance(fixedOrientation,str):
		#assume some klamp't code
		fixedOrientation = eval(fixedOrientation)

	Nw=pdef.get('Nw',1000)
	method=pdef.get('workspace_graph','staggered_grid')

	world = WorldModel()
	world.readFile(filename)
	robot = world.robot(0)
	program = RedundancyProgram(world,robot)

	#setup defaults
	if link == None:
		link = robot.link(robot.numLinks()-1)
	else:
		link = robot.link(link)
	if links:
		program.rr.ikTemplate.activeDofs = [robot.link(l).index for l in links]

	if orientation == 'fixed':
		if fixedOrientation == None:
			fixedOrientation = link.getTransform()[0]


	def collisionFree(q):
		robot.setConfig(q)
		if robot.selfCollides():
			return False
		return True

	#now set up the problem
	if orientation == 'fixed':
		program.rr.addIKConstraint(link,eelocal,orientation=fixedOrientation)
	else:
		program.rr.addIKConstraint(link,eelocal,orientation=orientation)
		if orientation == 'variable':
			if len(domain[0])==3:
				active = [1 if (a!=b) else 0 for (a,b) in zip(*domain)]
				if sum(active)==2:
					#pick the one non-variable axis
					rmin = [0,0,0]
					rmax = [0,0,0]
					for i,a in enumerate(active):
						if a==0:
							rmin[i] = -math.pi
							rmax[i] = math.pi
					domain = (domain[0]+rmin,domain[1]+rmax)
				else:
					domain = (domain[0]+[-math.pi]*3,domain[1]+[-math.pi]*3)
			#need to change the domain
	if useCollision:
		program.rr.ikTemplate.feasibilityTest = collisionFree

	if 'output' in pdef:
		folder = pdef['output']
	else:
		if 'output_prefix' in pdef:
			folder = "output/%s/"%(problem,)+pdef['output_prefix']
		else:
			folder = "output/%s/"%(problem,)
			if settings_file != 'settings.json':
				folder = folder + os.path.splitext(os.path.basename(settings_file))[0]
		folder = folder + "_W%d"%(Nw,)
		if method == 'grid':
			folder = folder + '_Ggrid'
		elif method == 'random':
			folder = folder + '_Grandom'
	program.folder = folder

	if load_output:
		try:
			print "Loading from",folder,"..."
			program.rr.load(folder)
			#TEMP: DO PADDING FOR BAXTER
			for iq,d in program.rr.Gq.nodes(data=True):
				if len(d['config']) < robot.numLinks():
					d['config'] = d['config'] + [0.0]*(robot.numLinks()-len(d['config']))
			program.rr.printStats()
		except IOError as e:
			print "Did not successfully load from folder %s, generating from scratch"%(folder,)
			program.rr.sampleWorkspace(domain[0],domain[1],Nw,method=method)
			print "Outputs will be saved to folder",folder
	else:
		program.rr.sampleWorkspace(domain[0],domain[1],Nw,method=method)
		print "Outputs will be saved to folder",folder

	#TEMP: make workspace grid following exact
	#if problem.startswith('planar'):
	#	for n,data in program.rr.Gw.nodes(data=True):
	#		data['params'][1] = 0

	program.settings = pdef
	program.settings['output'] = folder
	return program

def test_set_cover():
	universe = range(14)
	sets = {'A':range(7),
		'B':range(7,14),
		'C':[0,1,2,3,7,8,9,10],
		'D':[4,5,11,12],
		'E':[6,13],
		'F':[1,4,7,12],
		'G':[16,17]}
	#print set_cover(sets,universe)
	print set_cover(sets,None)

if __name__ == "__main__":
	import sys
	if len(sys.argv) > 1:
		problem = sys.argv[1]
	settings_file = None
	options = {}
	#parse command line options
	for i in sys.argv[2:]:
		parts = i.split('=')
		if len(parts) == 1:
			if i.endswith('json'):
				settings_file = i
			else:
				raise RuntimeError("Invalid command line argument, must be of form OPTION=VAL")
		else:
			opts = ["Nw","workspace_graph","output","output_prefix","link","links","domain","orientation"]
			stringopts = ["workspace_graph","output","output_prefix","link","orientation"]
			if parts[0] not in opts:
				print "Command line option must be one of:",opts
				raise RuntimeError("Invalid command line option")
			if parts[1] not in stringopts:
				options[parts[0]] = eval(parts[1])
			else:
				options[parts[0]] = parts[1]

	if problem.endswith('json'):
		#for loading settings.json files saved in the output folders
		settings_file = problem
		problem = None

	program = make_program(problem,options,settings_file)

	if KLAMPT_VERSION >= 0.7:
		#Square screen
		vp = vis.getViewport()
		#vp.w,vp.h = 800,800
		#For saving HD quality movies
		vp.w,vp.h = 1024,768
		vp.clippingplanes = (0.1,50)
		vis.setViewport(vp)
		#vis.run(program)
		vis.setPlugin(program)
		vis.show()
		while vis.shown():
			time.sleep(0.1)
		vis.setPlugin(None)
		vis.kill()
	else:
		#Square screen
		#program.width,program.height = 800,800
		#For saving HD quality movies
		program.width,program.height = 1024,768
		program.run()

	exit(0)

