from klampt import *
from klampt import loader
from klampt.trajectory import *
from learning.decisiontree_missing import *
from learning.nn_missing import *
import random
import math
import numpy as np
from database import *
import sys

Nlinks = 4
workspace_bounds = [(-4,4),(-4,4)]
W = 50
H = 50
obstacle_min_size = 1
obstacle_max_size = 20
obstacle_num_min = 0
obstacle_num_max = 6

world = WorldModel()
res = world.loadElement("planar_4R.rob")
assert res>=0,"failed to read robot file"
robot = world.robot(0)


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
				print "  %d%%.."%((100*self.cnt) / self.n,)
	def done(self):
		pass


def random_environment_blocks():
	env = np.zeros((W,H))
	nobs = random.randint(obstacle_num_min,obstacle_num_max)
	print nobs,"obstacles"
	for o in range(nobs):
		w = random.randint(obstacle_min_size,obstacle_max_size)
		h = random.randint(obstacle_min_size,obstacle_max_size)
		x = random.randint(0,W-1)
		y = random.randint(0,H-1)
		if x+w > W: w=W-x
		if y+h > H: h=H-y
		env[x:x+w,y:y+h] = 1
	return env

def random_environment_independent():
	frac = float(obstacle_num_min+obstacle_num_max)/2*float(obstacle_min_size+obstacle_max_size)**2/4 / (W*H)
	frac = 1.0-pow(1.0-frac,1.0/float(W/2))
	env = np.zeros((W,H))
	for i in range(W):
		for j in range(H):
			if random.random() < frac:
				env[i,j] = 1
	return env

def block_geom(i,j):
	global W,H,workspace_bounds,world
	l = (float(i)/float(W),float(j)/float(H))
	u = (float(i+1)/float(W),float(j+1)/float(H))
	plow = [bnd[0] + v*(bnd[1]-bnd[0]) for bnd,v in zip(workspace_bounds,l)]
	phigh = [bnd[0] + v*(bnd[1]-bnd[0]) for bnd,v in zip(workspace_bounds,u)]
	p = GeometricPrimitive()
	p.setAABB([plow[0],-0.05,plow[1]],[phigh[0],0.05,phigh[1]])
	t = world.makeTerrain("block(%d,%d)"%(i,j))
	t.geometry().setGeometricPrimitive(p)
	t.appearance().setColor(1,0.5,0,1)
	return t

global blocks
blocks = {}
for i in xrange(W):
	for j in xrange(H):
		blocks[i,j] = block_geom(i,j)

global motion_database
motion_database = []

def discretize_path(path,stepsize=0.1):
	res = [path[0]]
	for i in range(1,len(path)):
		L = robot.distance(path[i-1],path[i])
		ndivs = int(math.ceil(L/stepsize))
		for div in xrange(1,ndivs+1):
			u = float(div)/float(ndivs)
			res.append(robot.interpolate(path[i-1],path[i],u))
	return res

def collides(robot,geom):
	for i in range(robot.numLinks()):
		if geom.collides(robot.link(i).geometry()):
			return True
	return False

def collides_list(robot,geoms):
	for i in range(robot.numLinks()):
		if any(g.collides(robot.link(i).geometry()) for g in geoms):
			return True
	return False

def collision_iter(robot,geoms,ids):
	for g,id in zip(geoms,ids):
		if any(g.collides(robot.link(i).geometry()) for i in range(robot.numLinks())):
			yield id
	return

def error(problem,config=None):
	"""Returns an error value that we'd like to minimze. The score currently is a pair 
	(d,l) where d is the distance to the goal in 2d and l is the path length."""
	global robot
	if config == None:
		tol = 1e-3
		goal = ik.objective(robot.link(robot.numLinks()-1),local=[1,0,0],world=[problem.goal[0],0.0,problem.goal[1]])
		return (max(vectorops.norm(ik.residual(goal)),tol),robot.distance(problem.start,robot.getConfig()))
	else:
		robot.setConfig(config)
		return error(problem)

def last_feasible(problem,path):
	"""Finds the last feasible configuration on the given path"""
	global robot,blocks
	for i,q in enumerate(path):
		if i==0: continue
		robot.setConfig(q)
		if collides_list(robot,problem.blockgeoms):
			return path[i-1]
	return path[-1]

def adapt_plan(problem,path):
	"""Adjusts a previously planned path to a new problem"""
	global robot,blocks
	problem.cache_blocklist()
	goal = ik.objective(robot.link(robot.numLinks()-1),local=[1,0,0],world=[problem.goal[0],0.0,problem.goal[1]])
	robot.setConfig(path[-1])
	ik.solve(goal)
	end = robot.getConfig()
	steps = discretize_path([problem.start]+path[1:]+[end])
	return [problem.start,last_feasible(problem,steps)]

def motion_plan(problem):
	"""Generates a plan for a given problem."""
	global robot,blocks
	problem.cache_blocklist()
	#check start config
	if collides_list(robot,problem.blockgeoms):
		return None
	goal = ik.objective(robot.link(robot.numLinks()-1),local=[1,0,0],world=[problem.goal[0],0.0,problem.goal[1]])
	robot.setConfig(problem.start)
	ik.solve(goal)
	tol = 1e-3
	desconfigs = [(error(problem),robot.getConfig())]
	for i in range(100):
		robot.randomizeConfig()
		ik.solve(goal)
		desconfigs.append((error(problem),robot.getConfig()))
	desconfigs = sorted(desconfigs)
	robot.setConfig(problem.start)
	bestQuality = error(problem)
	best = problem.start
	for (e,d),qd in desconfigs:
		if e > bestQuality[0]:
			break
		if bestQuality[0] <= tol and d > bestQuality[1]:
			break
		#interpolate toward qd, check collisions
		steps = discretize_path([problem.start,qd])
		robot.setConfig(last_feasible(problem,steps))
		quality = error(problem)
		if quality < bestQuality:
			bestQuality = quality
			best = robot.getConfig()
	return [problem.start,best]

def swept_volume(path):
	global blocks
	hitblocks = set()
	steps = discretize_path(path)
	for index in collision_iter(robot,[t.geometry() for t in blocks.itervalues()],blocks.keys()):
		hitblocks.add(index)
	return list(hitblocks)

class Problem:
	def __init__(self,start=None,goal=None,env=None):
		self.start = start
		self.goal = goal
		self.env = env
		self.solution = None
		self.solution_index = None
		self.blocklist = None
		self.blockgeoms = None

	@staticmethod
	def random():
		robot.randomizeConfig()
		s = robot.getConfig()
		g = [random.uniform(*bnd) for bnd in workspace_bounds]
		e = random_environment_blocks()
		return Problem(s,g,e)

	@staticmethod
	def random2():
		s = (random.randint(0,W-1),0)
		g = (random.randint(0,W-1),H-1)
		e = random_environment_independent()
		return Problem(s,g,e)

	@staticmethod
	def featureNames():
		global W,H
		bnames = []
		for i in range(W):
			for j in range(H):
				bnames.append("b(%d,%d)"%(i,j))
		return ["q%d"%(i,) for i in range(Nlinks)]+['tx','ty']+bnames

	def features(problem):
		return list(problem.start)+list(problem.goal)+[int(v) for v in problem.env.flatten().tolist()]

	def cache_blocklist(self):
		if self.blocklist is None:
			self.blocklist = []
			for i in range(self.env.shape[0]):
				for j in range(self.env.shape[1]):
					if self.env[i,j]:
						self.blocklist.append((i,j))
			global blocks
			self.blockgeoms = [blocks[index].geometry() for index in self.blocklist]

	@staticmethod
	def from_features(features):
		p = Problem()
		p.start = [float(v) for v in features[:Nlinks]]
		p.goal = [float(v) for v in features[Nlinks:Nlinks+2]]
		ofs = Nlinks+2
		p.env = np.zeros((W,H))
		for i in range(W):
			for j in range(H):
				p.env[i,j] = int(features[ofs])
				ofs += 1
		return p

	def concept(problem):
		global motion_database
		res = motion_plan(problem)
		if res == None:
			return -1
		problem.solution = res
		#add to motion database
		motion_database.append(problem.solution)
		problem.solution_index = len(motion_database)-1
		return problem.solution_index

	def relevance(problem):
		global robot,blocks
		blockrelevant = np.zeros((W,H),dtype=np.int_)
		if problem.solution == None:
			#start is infeasible
			robot.setConfig(problem.start)
			for index,t in blocks.iteritems():
				if problem.env[index] and collides(robot,t.geometry()):
					blockrelevant[index] = 1
		else:
			#have a feasible solution -- does it touch an obstacle?
			for index in swept_volume(problem.solution):
				blockrelevant[index] = 1
		return [1]*len(problem.start)+[1]*len(problem.goal) + [int(v) for v in blockrelevant.flatten().tolist()]

	def show(problem):
		global robot,blocks
		from klampt import visualization
		visualization.clear()
		robot.setConfig(problem.start)
		visualization.add("robot",robot)
		visualization.add("target",[problem.goal[0],0.0,problem.goal[1]])
		for i in xrange(W):
			for j in xrange(H):
				if problem.env[i,j]:
					visualization.add("block %d,%d"%(i,j),blocks[i,j])
		if problem.solution != None:
			visualization.animate("robot",RobotTrajectory(robot,times=range(len(problem.solution)),milestones=problem.solution))
		visualization.dialog()


def write(dbf,dbr,dbc,motions,prefix='',suffix=''):
	print "Writing %d features to %sfeatures%s.csv"%(len(dbf.entries),prefix,suffix)
	print "Writing feature relevance to %srelevance%s.csv"%(prefix,suffix)
	print "Writing concept to %sconcept%s.csv"%(prefix,suffix)
	print "Writing %d motions to %smotions%s.txt"%(len(motions),prefix,suffix)
	if dbf != None:
		dbf.writeCSV("%sfeatures%s.csv"%(prefix,suffix))
	if dbr != None:
		dbr.writeCSV("%srelevance%s.csv"%(prefix,suffix))
	if dbc != None:
		dbc.writeCSV("%sconcept%s.csv"%(prefix,suffix))
	f = open("%smotions%s.txt"%(prefix,suffix),"w")
	for i,m in enumerate(motions):
		f.write(str(i)+"\t")
		for milestone in m:
			f.write(loader.writeVector(milestone))
			f.write("\t")
		f.write("\n")
	f.close()


def read(prefix='',suffix='',maxentries=None):
	"""Returns a tuple containing the feature database, relevance database, label database,
	and solved motions.
	"""
	try:
		dbf = Database()
		dbf.readCSV("%sfeatures%s.csv"%(prefix,suffix),maxentries=maxentries,storage='numpy',dtype=float)
		print "Read",len(dbf.entries),"examples from","%sfeatures%s.csv"%(prefix,suffix)
	except Exception:
		print "Failed to open/parse","%sfeatures%s.csv"%(prefix,suffix)
		dbf = None

	try:
		dbr = Database()
		dbr.readCSV("%srelevance%s.csv"%(prefix,suffix),maxentries=maxentries,storage='numpy',dtype=int)
		print "Read",len(dbr.entries),"examples from","%srelevance%s.csv"%(prefix,suffix)
	except Exception:
		print "Failed to open/parse","%srelevance%s.csv"%(prefix,suffix)
		dbr = None
		
	try:
		dbc = Database()
		dbc.readCSV("%sconcept%s.csv"%(prefix,suffix),maxentries=maxentries,dtype=int)
		print "Read",len(dbc.entries),"concepts from","%sconcepts%s.csv"%(prefix,suffix)
	except Exception:
		print "Failed to open/parse","%sconcept%s.csv"%(prefix,suffix)
		dbc = None
	
	motions = []
	try:
		f = open("%smotions%s.txt"%(prefix,suffix),'r')
		for line in f.readlines():
			index,path=line.split(None,1)
			index = int(index)
			assert index >= 0
			motion = loader.readVectorList(path)
			motions.append(motion)
			if maxentries != None and len(motions) >= maxentries:
				break
		print "Read",len(motions),"motions from","%smotions%s.txt"%(prefix,suffix)
	except IOError:
		print "Failed to open/parse","%smotions%s.txt"%(prefix,suffix)
		pass
	return (dbf,dbr,dbc,motions)

def read_motions(prefix='',suffix='',maxentries=None):
	"""Returns a tuple containing the database of solved motions.
	"""	
	motions = []
	try:
		f = open("%smotions%s.txt"%(prefix,suffix),'r')
		for line in f.readlines():
			index,path=line.split(None,1)
			index = int(index)
			assert index >= 0
			motion = loader.readVectorList(path)
			motions.append(motion)
			if maxentries != None and len(motions) >= maxentries:
				break
		print "Read",len(motions),"motions from","%smotions%s.txt"%(prefix,suffix)
	except IOError:
		print "Failed to open/parse","%smotions%s.txt"%(prefix,suffix)
		pass
	return motions

def make_problems(dbf,dbr,dbc,motions):
	dbf.cast()
	dbc.cast()
	problems = []
	for f,c in zip(dbf.entries,dbc.entries):
		p = Problem.from_features(f)
		p.solution_index = c[0]
		if c[0] < 0:
			p.solution = None
		else:
			p.solution = motions[c[0]]
		problems.append(p)
	return problems

def generate(count=1000,prefix='',suffix=''):
	problems = [Problem.random() for i in range(count)]
	updater = ProgressUpdater(len(problems))
	concepts = []
	for p in problems:
		updater.update()
		concepts.append(p.concept())
	updater.done()

	features = [p.features() for p in problems]
	relevance = [p.relevance() for p in problems]
	dbf = Database()
	dbf.keys = Problem.featureNames()
	dbf.entries = features
	dbr = Database()
	dbr.keys = Problem.featureNames()
	dbr.entries = relevance
	dbc = Database()
	dbc.keys = ['motion_index']
	dbc.entries = [[c] for c in concepts]
	write(dbf,dbr,dbc,motion_database,prefix,suffix)

def merge(mergeSuffixes,prefix=''):
	global motion_database
	dbf = Database()
	dbr = Database()
	dbc = Database()
	dbc.keys = ['motion_index']
	for suffix in mergeSuffixes:
		dbfi,dbri,dbci,motionsi = read(prefix,suffix)
		if not dbf.keys:
			dbf.keys = dbfi.keys
		else:
			assert dbf.keys == dbfi.keys
		dbf.entries += dbfi.entries

		if not dbr.keys:
			dbr.keys = dbri.keys
		else:
			assert dbr.keys == dbri.keys
		dbr.entries += dbri.entries

		offset = len(motion_database)
		for e in dbci.entries:
			if e[0]=='-1':
				dbc.entries.append(e)
			else:
				dbc.entries.append([offset + int(e[0])])

		motion_database += motionsi
	write(dbf,dbr,dbc,motion_database,prefix,'')

def test(learner,prefix='',suffix=''):
	global motion_database
	dbf,_,dbc,testmotions = read(prefix,suffix)
	fn = 0
	tn = 0
	fp = 0
	tp = 0
	num_immediately_feasible = 0
	average_optimized_quality = (0,0)
	average_initial_quality = (0,0)
	average_adapted_quality = (0,0)
	#check that the test motions are ok
	for f,c in zip(dbf.entries,dbc.entries):
		print c[0]
		assert c[0] >= -1 and c[0] < len(testmotions)
	for f,c in zip(dbf.entries,dbc.entries):
		motion_index = learner.predict(f)
		assert motion_index >= -1 and motion_index < len(motion_database),"Invalid prediction, did you set up the motion database?"
		if motion_index < 0 and c[0] >= 0:
			fn += 1
		elif motion_index < 0 and c[0] < 0:
			tn += 1
		elif c[0] < 0:
			fp += 1
		else:
			tp += 1
			#have a predicted motion, see how well it adapts to the problem
			p = Problem.from_features(f)
			m = motion_database[motion_index]
			assert m != None
			mtest = testmotions[c[0]]
			average_optimized_quality = vectorops.add(average_optimized_quality,error(p,mtest[-1]))
			p.cache_blocklist()
			if last_feasible(p,m) == m[-1]:
				num_immediately_feasible += 1
			average_initial_quality = vectorops.add(average_initial_quality,error(p,m[-1]))
			res = adapt_plan(p,m)
			average_adapted_quality = vectorops.add(average_adapted_quality,error(p,res[-1]))
	if tp > 0:
		average_optimized_quality = vectorops.div(average_optimized_quality,tp)
		average_initial_quality = vectorops.div(average_initial_quality,tp)
		average_adapted_quality = vectorops.div(average_adapted_quality,tp)
	N = len(dbf.entries)
	print "Tested on",N,"examples"
	print "Feasibility prediction accuracy",float(tn+tp)/N
	print "True + %g, True - %g"%(float(tp)/N,float(tn)/N)
	print "False + %g, False - %g"%(float(fp)/N,float(fn)/N)
	if tp > 0:
		print "Over true positives:"
		print "  Fraction immediately feasible:",float(num_immediately_feasible)/tp
		print "  Optimized quality (ground truth)",average_optimized_quality
		print "  No-adaptation quality",average_initial_quality
		print "  Post-adaptation quality",average_adapted_quality

def make_learner(method):
	if method.startswith("irrelevance_"):
		method = method[len("irrelevance_"):]
	if method == "decision_tree":
		return DecisionTree()
	elif method.endswith("nn"):
		if method == "nn":
			return NearestNeighbor()
		else:
			return NearestNeighbor(k=int(method[:-2]))
	else:
		raise ValueError("Invalid learning method specified")

def make_sparse_db(db,dbrel):
	dbsparse = Database()
	dbsparse.keys = db.keys
	for (e,r) in zip(db.entries,dbrel.entries):
		assert len(e) == len(r),"feature / relevance lengths %d and %d do not match"%(len(e),len(r))
		rowdict = {}
		for i,(v,rv) in enumerate(zip(e,r)):
			if rv!=0:
				rowdict[i] = v
		dbsparse.entries.append(rowdict)
	return dbsparse

if __name__ == '__main__':
	if len(sys.argv) <= 1:
		print "Valid operations: gen N [filenum], merge filecount, learn method, test method"
		exit(0)
	op = sys.argv[1]
	if op == 'gen':
		N = 1000
		if len(sys.argv) > 2:
			N = int(sys.argv[2])
		if len(sys.argv) > 3:
			suffix = "_"+sys.argv[3]
		else:
			suffix = ""
		generate(N,prefix='data/planar_arm_',suffix=suffix)
	elif op == 'merge':
		filecount = int(sys.argv[2])
		merge(['_'+str(i+1) for i in range(filecount)],prefix='data/planar_arm_')
	elif op == 'learn':
		method = sys.argv[2]
		learner = make_learner(method)

		print "Reading training set..."
		dbf,dbr,dbc,motions = read('data/planar_arm_')
		if dbr == None:
			print "Generating relevance vectors"
			problems = make_problems(dbf,dbr,dbc,motions)
			dbr = Database()
			updater = ProgressUpdater(len(problems))
			for p in problems:
				updater.update()
				dbr.entries.append(p.relevance())
			updater.done()
			dbr.writeCSV("data/planar_arm_relevance.csv")
		labels = [int(e[0]) for e in dbc.entries]

		#clear memory
		if "irrelevance" in method:
			print "Making sparse dataset..."
			db = make_sparse_db(dbf,dbr)
			dbf = None
			dbr = None
			dbc = None
			print "Learning..."
			learner.learn(db,labels)
		else:
			dbr = None
			dbc = None
			print "Learning..."
			learner.learn(dbf,labels)
		fn = "planar_arm_%s.pickle"%(method,)
		print "saving to",fn
		learner.save(fn)
	elif op == 'test':
		global motion_database
		method = sys.argv[2]
		motion_database = read_motions('data/planar_arm_')
		learner = make_learner(method)
		learner.load("planar_arm_%s.pickle"%(method,))
		test(learner,'data/planar_arm_','_test')
	else:
		print "Invalid operation",op
		

