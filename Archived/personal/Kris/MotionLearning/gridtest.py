import random
import numpy as np
import math
from klampt import vectorops
import sys
import learning.tester

W = 20
H = 20
obstacle_min_size = 1
obstacle_max_size = 7
obstacle_num_min = 0
obstacle_num_max = 5
#W = 10
#H = 10
#obstacle_min_size = 1
#obstacle_max_size = 5
#obstacle_num_min = 0
#obstacle_num_max = 5
label_obstacle_threshold = 1
hierarchical = False

def purely_random_environment():
	return np.random.randint(0,1,(W,H))

def random_environment_blocks():
	env = np.zeros((W,H))
	nobs = random.randint(obstacle_num_min,obstacle_num_max)
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

def build_hierarchy(mat,op=max):
	h = [mat]
	while mat.shape > (1,1):
		newshape = tuple(int(math.ceil(v*0.5)) for v in mat.shape)
		newmat = np.zeros(newshape)
		for i in xrange(newmat.shape[0]):
			for j in xrange(newmat.shape[1]):
				imax = min(mat.shape[0],(i+1)*2)
				jmax = min(mat.shape[1],(j+1)*2)
				vals = [v for v in mat[i*2:imax,j*2:jmax].flatten()]
				newmat[i,j] = op(vals)
		h.append(newmat)
		mat = newmat
	return h[::-1]

class Problem:
	def __init__(self,start=None,goal=None,env=None):
		self.start = start
		self.goal = goal
		self.env = env
	@staticmethod
	def random():
		s = (random.randint(0,W-1),0)
		g = (random.randint(0,W-1),H-1)
		e = random_environment_blocks()
		return Problem(s,g,e)

	@staticmethod
	def random2():
		s = (random.randint(0,W-1),0)
		g = (random.randint(0,W-1),H-1)
		e = random_environment_independent()
		return Problem(s,g,e)

	def features(problem):
		if hierarchical:
			features = list(problem.start)+list(problem.goal)
			h = build_hierarchy(problem.env)
			for m in h:
				features += [int(v) for v in m.flatten().tolist()]
			return features
		return list(problem.start)+list(problem.goal)+[int(v) for v in problem.env.flatten().tolist()]

	def concept(problem):
		d = vectorops.distance(problem.start,problem.goal)
		h = 0.2
		nsteps = int(math.ceil(d / h))
		collisions = 0
		#hacky "rasterization" by marching down segment
		for i in range(nsteps+1):
			if nsteps == 0:
				x = problem.start
			else:
				x = vectorops.interpolate(problem.start,problem.goal,float(i)/nsteps)
			if problem.env[int(x[0]),int(x[1])]:
				collisions += 1
				if collisions >= label_obstacle_threshold:
					return 0
		#collision free
		return 1

	def relevance(problem):
		feasible = problem.concept()
		d = vectorops.distance(problem.start,problem.goal)
		h = 0.2
		nsteps = int(math.ceil(d / h))
		env_relevance = np.zeros((W,H))
		#try including some adjacent features
		#footprint = [(0,0),(-1,0),(1,0),(0,-1),(0,1)]
		footprint = []
		for i in range(nsteps+1):
			if nsteps == 0:
				x = problem.start
			else:
				x = vectorops.interpolate(problem.start,problem.goal,float(i)/nsteps)
			x = (int(x[0]),int(x[1]))
			if feasible or problem.env[x]:
				env_relevance[x] = 1
				for (dx,dy) in footprint:
					if 0<=x[0]+dx<W and 0<=x[1]+dy<H:
						env_relevance[x[0]+dx,x[1]+dy] = 1
		if hierarchical:
			features = [1]*2+[1]*2
			h = build_hierarchy(env_relevance)
			for m in h:
				features += [int(v) for v in m.flatten().tolist()]
			return features
		return [1]*2+[1]*2+[int(v) for v in env_relevance.flatten().tolist()]


if __name__ == '__main__':
	import sys
	N = 1
	if len(sys.argv) > 1:
		N = int(sys.argv[1])
	learning.tester.set_problem("grid(%d,%d)"%(W,H))
	learning.tester.set_output_file("grid_%dx%d.json"%(W,H))
	learning.tester.set_data_folder("gridtest")
	if not hierarchical:
		learning.tester.add_sampler("blocks",Problem.random)
		learning.tester.add_sampler("independent",Problem.random2)
		#learning.tester.add_dataset("blocks",'train_blocks_10000.csv','test_blocks_10000.csv','train_relevance_blocks_10000.csv')
		#learning.tester.add_dataset("independent",None,'test_independent_10000.csv')
		suffix = ""
	else:
		learning.tester.add_sampler("blocks_hierarchical",Problem.random)
		learning.tester.add_sampler("independent_hierarchical",Problem.random2)
		#learning.tester.add_dataset("blocks_hierarchical",'train_blocks_hierarchical_10000.csv','test_blocks_hierarchical_10000.csv','train_relevance_blocks_hierarchical_10000.csv')
		#learning.tester.add_dataset("independent_hierarchical",None,'test_independent_hierarchical_10000.csv')
		suffix = "_hierarchical"
	learning.tester.add_method('decision tree')
	#learning.tester.add_method('irrelevance expanded decision tree')
	learning.tester.add_method('irrelevance decision tree',maxnodes=20000)
	#learning.tester.add_method('nearest neighbor')
	#learning.tester.add_method('nearest neighbor',k=3)
	#learning.tester.add_method('nearest neighbor',k=5)
	learning.tester.add_method('nearest neighbor',k=9)
	#learning.tester.add_method('irrelevance nearest neighbor')
	#learning.tester.add_method('irrelevance nearest neighbor',k=3)
	#learning.tester.add_method('irrelevance nearest neighbor',k=5)
	learning.tester.add_method('irrelevance nearest neighbor',k=9)
	#learning.tester.add_method('random forest',Ntrees=10)
	#learning.tester.add_method('random forest',Ntrees=50)
	#learning.tester.add_method('random forest',Ntrees=100)
	#learning.tester.add_method('irrelevance random forest',Ntrees=10)
	#learning.tester.add_method('irrelevance random forest',Ntrees=50)
	#learning.tester.add_method('irrelevance random forest',Ntrees=100)
	learning.tester.add_scenario(Ntrain=1000,Ntest=1000,testing='blocks'+suffix)
	learning.tester.add_scenario(Ntrain=5000,Ntest=1000,testing='blocks'+suffix)
	learning.tester.add_scenario(Ntrain=10000,Ntest=1000,testing='blocks'+suffix)
	#learning.tester.add_scenario(Ntrain=50000,Ntest=10000,testing='blocks'+suffix)
	#learning.tester.add_scenario(Ntrain=100000,Ntest=10000,testing='blocks'+suffix)
	learning.tester.add_scenario(Ntrain=1000,Ntest=1000,testing='independent'+suffix)
	learning.tester.add_scenario(Ntrain=5000,Ntest=1000,testing='independent'+suffix)
	learning.tester.add_scenario(Ntrain=10000,Ntest=1000,testing='independent'+suffix)
	#learning.tester.add_scenario(Ntrain=50000,Ntest=10000,testing='independent'+suffix)
	#learning.tester.add_scenario(Ntrain=100000,Ntest=10000,testing='independent'+suffix)
	learning.tester.run(N)
