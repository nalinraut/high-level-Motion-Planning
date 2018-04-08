import pkg_resources
pkg_resources.require("klampt>=0.6.2")
if pkg_resources.get_distribution("klampt").version >= '0.7':
	#Klampt v0.7.x
	from klampt import *
	from klampt.math import vectorops,so3,se3
	KLAMPT_VERSION = 0.7
else:
	#Klampt v0.6.x
	from klampt import *
	KLAMPT_VERSION = 0.6
from metric import *
from disjointset import *
from collections import defaultdict
from ikdb.utils import mkdir_p
from csp import *
from redundancygraph import RedundancyResolutionGraph
from redundancyvisualization import *
import networkx as nx
from OpenGL.GL import *
from utils import *
from nearestneighbors import *
import random
import math
import time

def nearest_point(G,nodes,weight=None,p=1):
	"""Finds the node closest to the given nodes on the graph, where the
	node-node distances are combined using an L_p metric.  If no node is
	reachable by all the nodes, this returns the one that is nearest to
	the maximum number of nodes.
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


class RedundancyResolver:
	"""Performs the redundancy resolution procedure described in:

	K. Hauser, Continuous pseudoinversion of a multivariate function: application
	to global redundancy resolution, Workshop on the Algorithmic Foundations of Robotics, 2016.

	Attributes:
	- resolution: a RedundancyResolutionGraph structure defining the problem
	- robot: the robot to be resolved (must be same as resolution.robot)
	- Gw: the workspace graph, with an equivalent structure to the one in resolution.Gw
	  but with node attributes:
	  * 'params': the point in the workspace (same as in resolution.Gw)
	  * 'qlist': a list of configuration space node id's associated with this node
	  * 'qccs': a DisjointSet structure organizing the configurations in qlist, if self-motion
	     manifold construction is enabled
	  and edge attributes:
	  * 'qlist': a list of feasible configuration space edges associated with this edge
	- Gq: the configuration space graph.  Nodes have attributes:
	  * 'config': the configuration
	  * 'windex': the index of the associated workspace node
	  and edges have attribute:
	  * 'windex': the edge in the workspace graph associated with this edge (pair of ids)
	- cacheInfeasibleEdges: set this to True if infeasible edges should be remembered.
	  This is useful for incremental building, but should be turned off if self-motion manifolds
	  are being constructed.
	- infeasibleEdges: the list of previously detected infeasible edges
	- Qsubgraph: (internal) the subgraph of Gq that should be exported to self.resolution.
	- QccSubgraph: (internal) the subgraph of the connected components that should be collapsed to
	  determine self.resolution.  Used if self-motion manifolds are being used.

	Progress can be saved/loaded to a folder via save/load.
	"""
	def __init__(self,resolution):
		self.resolution = resolution
		self.robot = resolution.robot
		self.cacheInfeasibleEdges = False
		self.infeasibleEdges = set()
		self.Qsubgraph = set()
		self.QccSubgraph = set()
		self.initWorkspaceGraph()

	def initWorkspaceGraph(self,clear=False):
		"""Given a structure in resolution.Gw, creates the values of the workspace graph.  Gw and Gq are cleared.
		If clear = True, the resolution configurations are cleared
		"""
		self.Gw = nx.Graph()
		self.Gq = nx.Graph()
		#copy Gw structure from resolution
		for i,d in self.resolution.Gw.nodes_iter(data=True):
			self.Gw.add_node(i,params=d['params'],qlist=[],qccs=DisjointSet())
		for i,j,d in self.resolution.Gw.edges_iter(data=True):
			self.Gw.add_edge(i,j,qlist=[])
		if clear:
			self.resolution.clearResolution()

	def save(self,folder):
		mkdir_p(folder)
		nx.write_gpickle(self.Gw,os.path.join(folder,"Gw.pickle"))
		nx.write_gpickle(self.Gq,os.path.join(folder,"Gq.pickle"))
		self.resolution.save(folder)
		if len(self.Qsubgraph) > 0:
			f = open(os.path.join(folder,"Qsubgraph.txt"),"w")
			for i in sorted([v for v in self.Qsubgraph]):
				f.write(str(i)+" ")
			f.write("\n")
			f.close()
		if len(self.QccSubgraph) > 0:
			f = open(os.path.join(folder,"QccSubgraph.txt"),"w")
			for i in sorted([v for v in self.QccSubgraph]):
				f.write(str(i)+" ")
			f.write("\n")
			f.close()
		return

	def load(self,folder):
		self.Gw = nx.read_gpickle(os.path.join(folder,"Gw.pickle"))
		self.Gq = nx.read_gpickle(os.path.join(folder,"Gq.pickle"))
		try:
			self.resolution.load(folder)
		except IOError:
			#no resolution saved, may be old-style.  Need to copy from Gw
			self.resolution.clear()
			for i,d in self.Gw.nodes_iter(data=True):
				self.resolution.Gw.add_node(i,params=d['params'])
			for i,j in self.Gw.edges_iter():
				self.resolution.Gw.add_edge(i,j)
			self.resolution.buildNearestNeighbors()

			try:
				#OLD STYLE FOLDERS
				f = open(os.path.join(folder,"Qsubgraph.txt"),"r")
				items = f.readline().split()
				self.Qsubgraph = set(int(v) for v in items)
				f.close()
				self.calculateResolutionFromSubgraph()
				res = raw_input("Old-style folder detected, do you want to convert? (y/n) >")
				if res == 'y':
					#copy self.Gw to self.resolution.Gw
					print "Saving new-style resolution to disk"
					self.resolution.save(folder)
			except Exception as e:
				import traceback
				print "Warning, Qsubgraph could not be loaded"
				print "Exception:",e
				traceback.print_exc()
				#print "Making assignment..."
				#self.uniqueAssignment()
				pass
		try:
			f = open(os.path.join(folder,"Qsubgraph.txt"),"r")
			items = f.readline().split()
			self.Qsubgraph = set(int(v) for v in items)
			f.close()
		except IOError:
			self.Qsubgraph = set()
		try:
			f = open(os.path.join(folder,"QccSubgraph.txt"),"r")
			items = f.readline().split()
			self.QccSubgraph = set(int(v) for v in items)
			f.close()
		except IOError:
			self.QccSubgraph = set()
		
		self.sanityCheck()
		"""
		#setup domain
		bmin = self.Gw.node[0]['params'][:]
		bmax = self.Gw.node[0]['params'][:]
		for i,d in self.Gw.nodes_iter(data=True):
			x = d['params']
			for j,v in enumerate(x):
				bmin[j] = min(bmin[j],v)
				bmax[j] = max(bmax[j],v)
		self.domain = (bmin,bmax)
		#setup nearest neighbors
		self.nnw = NearestNeighbors(L2Metric,'kdtree')
		for n,d in self.Gw.nodes_iter(data=True):
			self.nnw.add(d['params'],n)
		"""

	def sanityCheck(self):
		#sanity check
		for i,d in self.Gw.nodes_iter(data=True):
			for iq in d['qlist']:
				assert iq >= 0 and iq < len(self.Gq.node)
				assert self.Gq.node[iq]['windex'] == i
		for iq,d in self.Gq.nodes_iter(data=True):
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
		#check consistency with resolution
		for i,d in self.Gw.nodes_iter(data=True):
			assert self.resolution.Gw.has_node(i)
			assert self.resolution.Gw.node[i]['params'] == d['params']
			assert any(q in self.Qsubgraph for q in d['qlist']) == self.resolution.isResolvedNode(i),"Mismatch between resolution and Qsubgraph"
		for i,j in self.resolution.Gw.edges_iter():
			eqlist = self.Gw.edge[i][j]['qlist']
			insubgraph = any(a in self.Qsubgraph and b in self.Qsubgraph for (a,b) in eqlist)
			assert self.resolution.isResolvedEdge(i,j) == insubgraph,"Mismatch between resolution edges and Qsubgraph edges: %d vs %d"%(int(self.resolution.isResolvedEdge(i,j)),int(insubgraph))
		for iq in self.Qsubgraph:
			iw = self.Gq.node[iq]['windex']
			for jw in self.Gw.neighbors(iw):
				if any(jq in self.Qsubgraph and ((iq,jq) in self.Gw.edge[iw][jw]['qlist'] or (jq,iq) in self.Gw.edge[iw][jw]['qlist']) for jq in self.Gw.node[jw]['qlist']):
					assert self.resolution.isResolvedEdge(iw,jw),"Mismatch between resolution edges and Qsubgraph edges"
		"""
		#do edge validity checks (slow)
		for i,j in self.resolution.Gw.edges():
			if self.resolution.isResolvedEdge(i,j):
				assert self.resolution.validEdge(self.resolution.Gw.node[i]['config'],self.resolution.Gw.node[j]['config'],self.resolution.Gw.node[i]['params'],self.resolution.Gw.node[j]['params'])
		"""
		

	def printStats(self):
		print "Roadmap has",self.Gw.number_of_nodes(),"workspace nodes and",self.Gw.number_of_edges(),"edges"
		print "  and",self.Gq.number_of_nodes(),"configuration space nodes and",self.Gq.number_of_edges(),"edges"
		#if len(self.Qsubgraph) == 0:
		if not self.resolution.hasResolution():
			print "  Resolution is empty."
			return
		numNodes = 0
		for i,d in self.Gw.nodes_iter(data=True):
			if len(d['qlist']) > 0:
				numNodes += 1
		print "  of %d nodes, %d have no configuration, and %d / %d are not in redundancy resolution"%(self.Gw.number_of_nodes(),self.Gw.number_of_nodes()-numNodes,numNodes-self.resolution.resolvedCount,numNodes)
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
			if i < j and  self.resolution.isResolvedEdge(i,j):
				numEdgesInResolution += 1
				sumdistances += self.robot.distance(self.resolution.Gw.node[i]['config'],self.resolution.Gw.node[j]['config'])
				sumwsdistances += workspace_distance(self.resolution.Gw.node[i]['params'],self.resolution.Gw.node[j]['params'])
		print "  of %d edges, %d  are missing a configuration space edge, and %d / %d are not in redundancy resolution"%(numPotentialEdges,numPotentialEdges-numEdgesWithConfigurations,numPotentialEdges-numEdgesInResolution,numPotentialEdges)
		if numEdgesInResolution != 0:
			print "  Average configuration space distance:",sumdistances / numEdgesInResolution
			print "  Average configuration space distance / workspace distance:",sumdistances / sumwsdistances

	def clearSamples(self):
		"""Reinitializes workspace and configuration space graphs."""
		self.Gw = nx.Graph()
		self.Gq = nx.Graph()
		self.resolution.clear()
		self.Qsubgraph = set()

	def addNode(self,iw,q):
		"""Adds a configuration node q corresponding to workspace node index iw.  Returns the index of the
		configuration space node"""
		assert iw < self.Gw.number_of_nodes()
		iq = self.Gq.number_of_nodes()
		self.Gq.add_node(iq,config=q,windex=iw)
		self.Gw.node[iw]['qlist'].append(iq)
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
		assert iq < self.Gq.number_of_nodes()
		assert jq < self.Gq.number_of_nodes()
		assert iw == self.Gq.node[iq]['windex']
		assert jw == self.Gq.node[jq]['windex']
		assert iq in self.Gw.node[iw]['qlist']
		assert jq in self.Gw.node[jw]['qlist']
		#TODO will case of iw == jw introduce bug on 'windex'?
		self.Gq.add_edge(iq,jq,windex=(iw,jw))
		if iw == jw:
			self.Gw.node[iw]['qccs'].merge(iq,jq)
		else:
			self.Gw.edge[iw][jw]['qlist'].append((iq,jq))
			
	def removeEdge(self,iw,iq,jw,jq):
		assert iw != jw
		if iw > jw:
			self.removeEdge(jw,jq,iw,iq)
			return
		assert iw < self.Gw.number_of_nodes()
		assert jw < self.Gw.number_of_nodes()
		assert iq < self.Gq.number_of_nodes()
		assert jq < self.Gq.number_of_nodes()
		assert iw == self.Gq.node[iq]['windex']
		assert jw == self.Gq.node[jq]['windex']
		assert iq in self.Gw.node[iw]['qlist']
		assert jq in self.Gw.node[jw]['qlist']
		assert (iq,jq) in self.Gw.edge[iw][jw]['qlist']
		self.Gq.remove_edge(iq,jq)
		self.Gw.edge[iw][jw]['qlist'].remove((iq,jq))

	def testAndConnectAll(self,iw,jw):
		"""Tries to connect two workspace nodes in configuration space by testing all pairs of configurations."""
		res = False
		for na in self.Gw.node[iw]['qlist']:
			for nb in self.Gw.node[jw]['qlist']:
				if self.testAndConnect(iw,na,jw,nb):
					res = True
		return res

	def testAndConnect(self,iw,iq,jw,jq):
		"""Tries to connect two configurations in the configuration map"""
		assert iw == self.Gq.node[iq]['windex']
		assert jw == self.Gq.node[jq]['windex']
		if (iq,jq) in self.infeasibleEdges:
			#already tested
			return False
		if self.Gq.has_edge(iq,jq):
			#already tested
			return True
		a = self.Gq.node[iq]['config']
		b = self.Gq.node[jq]['config']
		if self.resolution.validEdge(a,b,self.Gw.node[iw]['params'],self.Gw.node[jw]['params']):
			#add it
			self.addEdge(iw,iq,jw,jq)
			return True
		else:
			if self.cacheInfeasibleEdges:
				self.infeasibleEdges.add((iq,jq))
				self.infeasibleEdges.add((jq,iq))
		return False

	def testAndAddEdge(self,iw,iq,jw,q):
		"""Tries to connect an existing configuration to a new one.  If the connection
		is feasible, then the node is added and connected to iw,iq.  The index of
		the new node is returned.  If it's infeasible, None is returned."""
		assert iw == self.Gw.node[iq]['windex']
		assert jw < self.Gw.number_of_nodes()
		a = self.Gq.node[iq]['config']
		if self.resolution.validEdge(a,q,self.Gw.node[iw]['params'],self.Gw.node[jw]['params']):
			#add it
			jq = self.addNode(jw,q)
			self.addEdge(iw,iq,jw,jq)
			return jq
		return None

	def testAndConnectQccs(self,iw,iq,jw,jq):
		"""Tries to link two configuration-space connected components of different workspace points"""
		assert iw != jw
		assert iw == self.Gq.node[iq]['windex']
		assert jw == self.Gq.node[jq]['windex']
		iqccs = self.Gw.node[iw]['qccs']
		jqccs = self.Gw.node[jw]['qccs']
		#TODO: prioritize existing anchors
		distances = [(self.robot.distance(self.Gq.node[i]['config'],self.Gq.node[j]['config']),i,j) for i in iqccs.iterate(iq) for j in jqccs.iterate(jq)]
		distances = sorted(distances)
		numtests = 0
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


	def getSelfMotionCcsStats(self):
		"""Returns statistics describing the connected components in the self-motion graph
		of each workspace node."""
		sumConfigs = 0        
		sumNumQccs = 0
		sumMaxSize = 0
		sumAvgSize = 0.0
		sumMinSize = 0
		configuredW = 0
		for w,d in self.Gw.nodes_iter(data=True):
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
		for w,d in self.Gw.nodes_iter(data=True):
			c.update()
			#construct visibility prm
			wqnodes = d['qlist']
			wqccs = d['qccs']
			if k is not None:
				nnq = NearestNeighbors(L2Metric,'kdtree')
				for iq in wqnodes:
					nnq.add(self.Gq.node[iq]['config'],iq)
				for iq in wqnodes:
					knn = nnq.knearest(self.Gq.node[iq]['config'],k)
					for q,jq in knn:
						if not wqccs.same(iq,jq):
							self.testAndConnect(w,iq,w,jq)
			else:
				#visibility prm with all-pairs connections 
				for i,iq in enumerate(wqnodes):
					for j in xrange(i):
						jq = wqnodes[j]
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
			numPotentialNaiveEdges = 0
			for (i,j) in self.Gw.edges_iter():
				numPotentialNaiveEdges += len(self.Gw.node[i]['qlist'])*len(self.Gw.node[j]['qlist'])
			print "(Non-visibility graph method has " + str(numPotentialNaiveEdges) + " edges)"
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
			numPotentialEdges = 0
			for (i,j) in self.Gw.edges_iter():
				numPotentialEdges += len(self.Gw.node[i]['qlist'])*len(self.Gw.node[j]['qlist'])
			print "Potentially " + str(numPotentialEdges) + " edges"
			t0 = time.time()
			for (i,j,d) in self.Gw.edges_iter(data=True):
				c.update()
				for na in self.Gw.node[i]['qlist']:
					for nb in self.Gw.node[j]['qlist']:
						if na < start_node_count and nb < start_node_count:
							#existed previously, don't check it
							#print "Not checking",na,nb,"start",start_node_count
							continue
						self.testAndConnect(i,na,j,nb)
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
			ccs = list(nx.connected_components(self.Gq))
			print "Configuration space graph has",self.Gq.number_of_nodes(),"nodes and",len(ccs),"connected components"
			print "CC sizes:"
			for i in range(len(ccs)):
				if len(ccs[i]) == 1:
					print 
					print len(ccs)-i,"isolated nodes"
					break
				print "  ",len(ccs[i]),
			print
			print self.Gq.number_of_edges(),"edges out of possible",numPotentialEdges
		return numNondegenerateEdges, self.Gw.number_of_edges(), self_motion_time, inter_wnode_time

	def sampleConfigurationSpace(self,NConfigsPerPoint=100,connect=True,biased=False,seedFromAdjacent=True):
		print "Sampling",NConfigsPerPoint,"configurations at each workspace point"
		start_node_count = self.Gq.number_of_nodes()
		number_of_nodes_with_configurations = len([j for j,d in self.Gw.nodes_iter(data=True) if len(d['qlist']) > 0])
		c = ProgressUpdater(self.Gw.number_of_nodes(),5)
		start = time.clock()
		for i,d in self.Gw.nodes_iter(data=True):
			c.update()
			adjacentWithConfigs = [j for j in self.Gw.neighbors(i) if j < i and len(self.Gw.node[j]['qlist']) > 0]
			fracAdjacent = float(len(adjacentWithConfigs)) / float(len(self.Gw.neighbors(i)))
			numAdjacentSamples = int(fracAdjacent*NConfigsPerPoint)
			numRandomSamples = NConfigsPerPoint - numAdjacentSamples 
			if biased and start_node_count != 0:
				#numconfigs * average # of configs per workspace point / # of configs for this point
				if len(d['qlist']) == 0:
					numRandomSamples = 0
				else:
					numRandomSamples = NConfigsPerPoint * self.Gq.number_of_nodes() / (number_of_nodes_with_configurations * len(d['qlist']))
			#print fracAdjacent,"adjacent"

			self.resolution.setIKProblem(d['params'])

			#try starting from existing neighboring configurations
			self.resolution.ikSolverParams.startRandom = False
			numFailed = 0
			if seedFromAdjacent:
				for it in xrange(numAdjacentSamples):
					j = random.choice(adjacentWithConfigs)
					jq = random.choice(self.Gw.node[j]['qlist'])
					qstart = self.Gq.node[jq]['config']
					self.robot.setConfig(qstart)
					q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
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
					else:
						numFailed += 1

			self.resolution.ikSolverParams.startRandom = True
			for it in xrange(numRandomSamples+numFailed):
				#solver will sample randomly
				q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
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
		print "Sampled",self.Gq.number_of_nodes(),"of possible",self.Gw.number_of_nodes()*NConfigsPerPoint

		if connect:
			self.connectConfigurationSpace()
		
		return sampling_time

	def makeCSP(self,visibility=False):
		"""Creates a CSP whereby each workspace node is a variable and configs are values.
		Each neighboring node must satisfy the constraint that its workspace edge corresponds to
		a feasible configuration space edge.
		"""
		csp = CSP()
		for n,d in self.Gw.nodes_iter(data=True):
			if len(d['qlist']) > 0:
				if visibility:
					csp.addVariable(list(d['qccs'].getReps()),name="q_"+str(n))
				else:
					csp.addVariable(d['qlist'],name="q_"+str(n))
		for a,b,d in self.Gw.edges_iter(data=True):
			if (a < b and len(d['qlist']) > 0):
				validValues = set()
				for (iq,jq) in d['qlist']:
					if iq not in self.Gw.node[a]['qlist']:
						#swap?
						iq,jq = jq,iq
					if visibility:
						irep = self.Gw.node[a]['qccs'].getSetRep(iq)
						jrep = self.Gw.node[b]['qccs'].getSetRep(jq)
						assert irep in self.Gw.node[a]['qlist']
						assert jrep in self.Gw.node[b]['qlist']
						validValues.add((irep,jrep))
					else:
						try:
							assert iq in self.Gw.node[a]['qlist']
							assert jq in self.Gw.node[b]['qlist']
							validValues.add((iq,jq))
						except AssertionError:
							print "Warning, edge configuration value",iq,jq,"is not valid index in nodes"
							print "Start node config list",self.Gw.node[a]['qlist']
							print "End node config list",self.Gw.node[b]['qlist']
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
		for n,d in self.Gq.nodes_iter(data=True):
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
					#if we've resolved a previous workspace node in a prior iteration, don't consider other anchors on that node
					if jw in resolvedAnchors and jq != resolvedAnchors[jw]: continue
					if jw in resolutionFailures: continue
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
								self.removeEdge(iw, iq2, self.Gq.node[kq]['windex'], kq)
						anchors = {iq:list(neighbors)}
						totalCollapsed += 1
						#print "  Anchor",iq,"is a hub to",list(neighbors),"on nodes",[self.Gq.node[jq]['windex'] for jq in neighbors]
						break
					else:
						#print "  Anchor",iq,"is not a hub"
						pass
			
			if len(anchors) == 0:
				#this case can be reached when a workspace node is isolated
				self.Qsubgraph.add(rep)
				print "Note: no anchors for workspace node",iw,", must be isolated"
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
						#print "Moving anchors up from",iq,"to",pq
						if pq not in anchors:
							anchors[pq] = set()
						anchors[pq] |= anchors[iq]
						del anchors[iq] 
				print "  Collapsed further to",len(anchors),"anchors using spanning tree"

				#do we keep around the edges?
				for iq,jq in newEdges:
					if iq not in anchors or jq not in anchors[iq]:
						self.removeEdge(self.Gq.node[iq]['windex'], iq, self.Gq.node[jq]['windex'], jq)

				if len(anchors) > 1:
					#can't resolve this CC, fix it using full resolution afterward
					resolutionFailures.add(iw)
					continue

			assert len(anchors) <= 1
			for iq in anchors:
				resolvedAnchors[iw] = iq
				self.Qsubgraph.add(iq)

		#some resolutions failed -- try doing a CSP only on these and their neighbors
		if len(resolutionFailures) > 0:
			print "TRYING NEW RESOLUTION TECHNIQUE"
			failureWNodes = list(resolutionFailures)
			failureManifolds = dict()
			for rep in self.QccSubgraph:
				iw = self.Gq.node[rep]['windex']
				failureManifolds[iw] = self.Gw.node[iw]['qccs'].getSetRef(rep)
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
				if jw not in resolvedAnchors:
					#jw might be isolated...
					continue
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
					for v in csp.conDomains[c]:
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
			
		self.calculateResolutionFromSubgraph()	
		if len(self.QccSubgraph) > 0:
			return float(totalAnchors)/float(len(self.QccSubgraph)),float(totalNeighbors)/float(len(self.QccSubgraph)),totalCollapsed,triedToCollapse
		else:
			return -1,-1,-1,-1
			
	def decodeCSPAssignment(self,csp_assignment,visibility=False):
		"""For an assignment produced by makeCSP, calculates self.Qsubgraph and the resolution."""
		if visibility:		
			self.QccSubgraph = set()
		else:
			self.Qsubgraph = set()
		vcount = 0
		for n,d in self.Gw.nodes_iter(data=True):
			if len(d['qlist']) > 0:
				assert vcount < len(csp_assignment)
				if csp_assignment[vcount] != None:
					if visibility:
						self.QccSubgraph.add(csp_assignment[vcount])
					else:
						self.Qsubgraph.add(csp_assignment[vcount])
				vcount += 1
		if visibility:
			print "Now collapsing QccSubgraph"
			return self.collapseQccSubgraph()
		else:
			self.calculateResolutionFromSubgraph()
			return None,None,None,None

	def encodeCSPAssignment(self):
		"""Given self.Qsubgraph, returns an assignment for the CSP produced by makeCSP."""
		if len(self.Qsubgraph) == 0:
			return None
		res = []
		for n,d in self.Gw.nodes_iter(data=True):
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

	def pointwiseAssignment(self,NConfigsPerPoint=10):
		"""Moves pointwise through the workspace grid and generates a configuration by
		local optimization from the previous grid point.  If a configuration cannot be
		generated, then the edge is marked as infeasible and a random configuration is
		generated.
		"""
		print "Generating pointwise assignment"
		start_node_count = self.Gq.number_of_nodes()
		qmap = dict()
		emap = list()
		efailures = 0
		c = ProgressUpdater(self.Gw.number_of_nodes(),5)
		for i,d in self.Gw.nodes_iter(data=True):
			c.update()
			adjacentWithConfigs = [j for j in self.Gw.neighbors(i) if j < i and j in qmap]
			solved = False
			if len(adjacentWithConfigs) > 0:
				self.resolution.setIKProblem(d['params'])
				self.resolution.ikSolverParams.startRandom = False
				qstart = robot_average(self.robot,[qmap[j] for j in adjacentWithConfigs])
				self.robot.setConfig(qstart)
				q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
				if q is not None:
					for j in adjacentWithConfigs:
						if self.resolution.validEdge(qmap[j],q,self.Gw.node[j]['params'],d['params']):
							emap.append((i,j))
							qmap[i] = q
							solved = True
						else:
							efailures += 1
			if not solved:
				efailures += len([j for j in self.Gw.neighbors(i) if j < i])
				#no edge between neighbors and this node
				self.resolution.setIKProblem(d['params'])
				self.resolution.ikSolverParams.startRandom = True
				for sample in xrange(NConfigsPerPoint):
					#solver will sample randomly
					q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
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
			self.addEdge(i,iq,j,jq)
		self.Qsubgraph = set(nodemap.values())
		self.calculateResolutionFromSubgraph()

	def optimize(self,numIters=10):
		"""Optimizes the configurations of the current resolution to minimize
		joint-space path lengths using coordinate descent."""
		Gresolution = self.resolution.getResolutionGraph()
		for iters in xrange(numIters):
			print "Iteration",iters,"/",numIters
			sumdistances = 0
			c = ProgressUpdater(Gresolution.number_of_nodes(),10)
			for v in Gresolution.nodes_iter():
				c.update()
				if len(Gresolution.neighbors(v)) == 0: 
					continue
				x = self.resolution.Gw.node[v]['params']
				q0 = self.resolution.Gw.node[v]['config']
				wneighbors = [w for w in Gresolution.neighbors(v)]
				qneighbors = [self.resolution.Gw.node[w]['config'] for w in Gresolution.neighbors(v)]
				xneighbors = [self.resolution.Gw.node[i]['params'] for i in wneighbors]
				d0 = sum(self.robot.distance(q0,qw) for qw in qneighbors)
				qavg = robot_average(self.robot,qneighbors)
				#try to move toward average
				self.resolution.setIKProblem(x)
				self.resolution.ikSolverParams.startRandom = False
				maxTries = 10
				moved = False
				for tries in xrange(maxTries):
					self.robot.setConfig(qavg)
					q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
					if q != None:
						#check all edges from neighbors
						d = sum(self.robot.distance(q,qw) for qw in qneighbors)
						valid = True
						if d > d0:
							valid = False
						else:
							for qw,xw in zip(qneighbors,xneighbors):
								if not self.resolution.validEdge(q,qw,x,xw):
									valid = False
									break
						if valid:
							self.resolution.Gw.node[v]['config'] = q
							sumdistances += d
							moved = True
							break
					#not moved, try subdividing
					qavg = self.robot.interpolate(q0,qavg,0.5)
				if not moved:
					#print "Unable to move configuration",v
					sumdistances += d0
			c.done()
			print "Changed average path length to",sumdistances/Gresolution.number_of_edges()
		#now add new q's and edges between to the graph
		self.fromResolutionToGraph()

	def sampleConflicts(self,NConfigsPerPoint=10):
		conflicts = set()
		for i,j,d in self.Gw.edges_iter(data=True):
			if self.resolution.isResolvedEdge(i,j):
				continue
			if not self.resolution.isResolvedNode(i) or not self.resolution.isResolvedNode(j):
				continue
			conflicts.add(i)
			conflicts.add(j)
		added = []
		for iw in conflicts:
			d = self.Gw.node[iw]
			self.resolution.setIKProblem(d['params'])
			self.resolution.ikSolverParams.startRandom = True
			for sample in xrange(NConfigsPerPoint):
				#solver will sample randomly
				q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
				if q is not None:
					iq = self.addNode(iw,q)
					added.append(iq)
			#try local solving from neighbors too
			self.resolution.ikSolverParams.startRandom = True
			Qseed = sum([self.Gw.node[jw]['qlist'] for jw in self.Gw.neighbors(iw) if jw in conflicts],[])
			if len(Qseed) > 0:
				for sample in xrange(NConfigsPerPoint):
					qseed = self.Gq.node[random.choice(Qseed)]['config']
					self.robot.setConfig(qseed)
					q = self.resolution.ikTemplate.solve(self.robot,self.resolution.ikSolverParams)
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

	def solveCSP(self,visibility=False,numRandomDescents=2,numInitialGuesses=10):
		csp = self.makeCSP(visibility=visibility)
		start = time.clock()
		#assignment = csp.gecodeAssignment(maximize=True)
		#assignment = csp.sugarMaxAssignment()
		#assignment = csp.backtrackingAssignment()
		#if assignment == None:						
		assignment = csp.heuristicMaxAssignment()
		for i in xrange(numRandomDescents):
			assignment = csp.randomDescentAssignment(assignment)
		bestMethod = 'heuristic max'
		bestAssignment = assignment
		bestConflicts = csp.numConflicts(assignment)
		for guess in xrange(numInitialGuesses-1):
			assignment = csp.randomDescentAssignment()
			for i in xrange(numRandomDescents):
				assignment = csp.randomDescentAssignment(assignment)
			conflicts = csp.numConflicts(assignment)
			if conflicts < bestConflicts:
				bestMethod = 'random'
				bestAssignment = assignment
				bestConflicts = conflicts
		end = time.clock()
		print "CSP solve time",end-start,"minimum # of conflicts",bestConflicts,"using method",bestMethod
		self.decodeCSPAssignment(bestAssignment,visibility=visibility)
		end2 = time.clock()
		if visibility:
			print "QCC CSP resolve time",end2-end
		return bestConflicts

	def buildVisibilityGraph(self,numSamplesPerNode=100,knearest=20):
		self.sampleConfigurationSpace(numSamplesPerNode,connect=False)
							
		nondegenerate_workspace_nodes = 0
		for n,d in program.rr.Gw.nodes(data=True):
			if len(d['qlist']) > 0:
				nondegenerate_workspace_nodes += 1
		print "Creating self-motion manifolds..."
		nondegenerate_workspace_edges, total_workspace_edges, self_motion_time, inter_wnode_time = self.connectConfigurationSpace(visibility=True,k=knearest)
		print "Self motion manifold construction took time",self_motion_time
		print "Connecting workspace edges took time",inter_wnode_time
		#avgNumConfigs, avgNumQccs, avgMaxSize, avgAvgSize, avgMinSize = self.getSelfMotionCcsStats()

	def select(self,x,nodeRadius,edgeRadius,cc_filter=True,use_resolution='auto'):
		if use_resolution == 'auto':
			if len(self.Qsubgraph) == 0:
				use_resolution = False
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
		for (iw,data) in self.Gw.nodes_iter(data=True):
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

	def calculateResolutionFromSubgraph(self):
		"""Computes the resolution member from the Qsubgraph set."""
		self.resolution.clearResolution()
		for iq in self.Qsubgraph:
			iw = self.Gq.node[iq]['windex']
			self.resolution.setConfig(iw, self.Gq.node[iq]['config'])
		for iq in self.Qsubgraph:
			iw = self.Gq.node[iq]['windex']
			for jw in self.Gw.neighbors(iw):
				if self.resolution.isResolvedNode(jw) and any(jq in self.Qsubgraph and (self.Gq.has_edge(iq,jq) or self.Gq.has_edge(jq,iq))  for jq in self.Gw.node[jw]['qlist']):
					self.resolution.markConnected(iw,jw)
		self.sanityCheck()

	def fromResolutionToGraph(self):
		"""Adds all resolved configs/edges in the resolution member to Gq, and
		updates Qsubgraph"""
		iqmap = dict()
		for i,d in self.resolution.Gw.nodes_iter(data=True):
			if d.get('config',None) is not None:
				iqmap[i] = self.addNode(i,d['config'])
		for i,j,d in self.resolution.Gw.edges_iter(data=True):
			if d.get('connected',False):
				iq = iqmap[i]
				jq = iqmap[j]
				self.addEdge(i,iq,j,jq)
		self.Qsubgraph = set(iqmap.values())
		self.sanityCheck()


class RedundancyProgram(GLRedundancyProgram):
	def __init__(self,world,robot):
		GLRedundancyProgram.__init__(self,world,robot)
		self.rr = RedundancyResolver(self.resolution)
		self.folder = None
		self.settings = None
		self.drawCSpaceRoadmap = False

	def display(self):
		GLRedundancyProgram.display(self)
		if self.drawCSpaceRoadmap:
			#draw workspace - c-space roadmap
			glDisable(GL_LIGHTING)
			glEnable(GL_BLEND)
			glDisable(GL_DEPTH_TEST)
			glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
			glPointSize(5.0)
			xshift = -4
			glBegin(GL_POINTS)
			for n,d in self.rr.Gq.nodes_iter(data=True):
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
			for n,d in self.rr.Gq.nodes_iter(data=True):
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

	def keyboardfunc(self,c,x,y):
		if c == 'h':
			print "Keyboard help:"
			#print "- [space]: samples more points in the workspace (not very functional)"
			print "INTERACTIVE SOLVER"
			print "- q: samples more points in the configuration space (10 per workspace point)"
			print "- Q: samples more points in the configuration space attempting to add more in poorly-sampled areas"
			print "- c: samples more configurations at conflicting workspace edges"
			print "- u: performs a CSP assignment"
			print "- d: min-conflicts descent of the assignment"
			print "- r: randomizes the assignment and then descends"
			print "- p: performs pointwise redundancy resolution"
			print "- o: 10 iterations of coordinate descent optimization of overall path length"
			print "VISIBILITY GRAPH SOLVER"
			print "- v: build the network using the visibility graph algorithm."
			print "- R: solves a network built using the visibility graph algorithm."
			print "INSPECTION / VISUALIZATION"
			print "- s: saves roadmap and resolution to disk"
			print "- i: toggles between interpolation mode and graph inspection mode"
			print "- g: toggles drawing the workspace graph"
			print "- G: toggles drawing the C-space roadmap (only valid for planar problems)"
			print "- w: performs a walk to a random workspace node"
			print "- m: saves a real-time movie"
			print "- M: saves a 360 spin movie"
		if c == ' ':
			pass
			#print "Sampling workspace..."
			#self.resolution.sampleWorkspace([-3,0,-3],[3,0,3],10)
		elif c == 'q':
			print "Sampling configuration space..."
			self.rr.sampleConfigurationSpace(10)
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'Q':
			print "Sampling configuration space..."
			self.rr.sampleConfigurationSpace(10,biased=True)
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'c':
			print "Sampling configurations at conflicting edges..."
			self.rr.sampleConflicts()
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'u':
			print "Performing unique assignment..."
			self.rr.uniqueAssignment()
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'r':
			print "Doing random descent..."
			self.rr.randomDescentAssignment(True)
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'd':
			print "Doing descent..."
			self.rr.randomDescentAssignment(False)
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
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
			print "Pointwise assignment..."
			self.rr.pointwiseAssignment()
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'o':
			print "Optimizing..."
			self.rr.optimize()
			self.rr.printStats()
		elif c == 'v':
			self.rr.buildVisibilityGraph(100,knearest=20)
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		elif c == 'R':
			self.rr.solveCSP(visibility=True)
			self.rr.printStats()
			self.roadmapDisplayList.markChanged()
			self.disconnectionDisplayList.markChanged()
		else:
			GLRedundancyProgram.keyboardfunc(self,c,x,y)
		self.refresh()

def make_program(pdef):
	"""Loads a file from disk and returns an appropriately configured
	RedundancyProgram, given a JSON structure defining the problem.
	"""
	filename = pdef['filename']
	folder = pdef['output']
	Nw=pdef.get('Nw',1000)
	method=pdef.get('workspace_graph','staggered_grid')

	world = WorldModel()
	world.readFile(filename)
	robot = world.robot(0)
	program = RedundancyProgram(world,robot)
	program.resolution.readJsonSetup(pdef)
	program.folder = folder

	if not pdef.get('clean',False):
		try:
			print "Loading from",folder,"..."
			program.rr.load(folder)
			#TEMP: DO PADDING FOR BAXTER
			for iq,d in program.rr.Gq.nodes_iter(data=True):
				if len(d['config']) < robot.numLinks():
					d['config'] = d['config'] + [0.0]*(robot.numLinks()-len(d['config']))
			for iq,d in program.resolution.Gw.nodes_iter(data=True):
				if 'config' in d and len(d['config']) < robot.numLinks():
					d['config'] = d['config'] + [0.0]*(robot.numLinks()-len(d['config']))
			program.rr.printStats()
		except IOError as e:
			print "Did not successfully load from folder %s, generating from scratch"%(folder,)
			program.resolution.sampleWorkspace(Nw,method=method)
			program.rr.initWorkspaceGraph()
			print "Outputs will be saved to folder",folder
	else:
		program.resolution.sampleWorkspace(Nw,method=method)
		program.rr.initWorkspaceGraph()
		print "Outputs will be saved to folder",folder

	#TEMP: make workspace grid following exact
	#if problem.startswith('planar'):
	#	for n,data in program.rr.Gw.nodes_iter(data=True):
	#		data['params'][1] = 0

	program.settings = pdef
	return program


if __name__ == "__main__":
	#TESTING so3_grid function
	#from so3_grid import so3_grid_test
	#so3_grid_test()
	import sys
	if len(sys.argv) == 0:
		print "USAGE: redundancy.py problem [settings json file] [option1=val1 option2=val2 ...]"
	
	opts = parse_args(sys.argv)
	program = make_program(opts)
	program.run()

	exit(0)

