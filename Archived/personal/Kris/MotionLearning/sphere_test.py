import random
import numpy as np
import math
from klampt import vectorops
import sys
import learning.tester

robot_radius = 3
workspace_size = (20,20,20)
obstacle_min_size = 1
obstacle_max_size = 7
obstacle_num_min = 0
obstacle_num_max = 5
#workspace_size = (10,10,10)
#obstacle_min_size = 1
#obstacle_max_size = 5
#obstacle_num_min = 0
#obstacle_num_max = 5
label_obstacle_threshold = 1

def purely_random_environment():
	return np.random.randint(0,1,workspace_size)

def random_environment_blocks():
	env = np.zeros(workspace_size)
	nobs = random.randint(obstacle_num_min,obstacle_num_max)
	for o in range(nobs):
		d = np.random.randint(obstacle_min_size,obstacle_max_size,(3,))
		p = np.array([random.randint(0,size-1) for size in workspace_size])
		q = np.minimum(p+d,np.array(workspace_size)-1)
		env[p[0]:q[0],p[1]:q[1],p[2]:q[2]] = 1
	return env

def random_environment_independent():
	frac = float(obstacle_num_min+obstacle_num_max)/2*float(obstacle_min_size+obstacle_max_size)**3/8 / np.prod(workspace_size)
	frac = 1.0-pow(1.0-frac,1.0/float(workspace_size[0]/2))
	return np.random.choice([0, 1], size=workspace_size, p=[1.0-frac,frac])

def sphere_collision(x,radius,grid):
	bmin = vectorops.sub(x,[radius]*len(x))
	bmax = vectorops.add(x,[radius]*len(x))
	imin = [max(0,int(v)) for v in bmin]
	imax = [min(int(v),b-1) for v,b in zip(bmax,workspace_size)]
	for i in range(imin[0],imax[0]):
		for j in range(imin[1],imax[1]):
			for k in range(imin[2],imax[2]):
				if vectorops.distanceSquared((i,j,k),x) < radius**2 and grid[i,j,k]:
					return True
	return False

def sphere_collision_tests(x,radius,grid):
	bmin = vectorops.sub(x,[radius]*len(x))
	bmax = vectorops.add(x,[radius]*len(x))
	imin = [max(0,int(v)) for v in bmin]
	imax = [min(int(v),b-1) for v,b in zip(bmax,workspace_size)]
	for i in range(imin[0],imax[0]):
		for j in range(imin[1],imax[1]):
			for k in range(imin[2],imax[2]):
				if vectorops.distanceSquared((i,j,k),x) < radius**2:
					yield (i,j,k)
	return

class Problem:
	def __init__(self,start=None,goal=None,env=None):
		self.start = start
		self.goal = goal
		self.env = env
	@staticmethod
	def random():
		s = (random.randint(0,workspace_size[0]-1),workspace_size[1]/2,0)
		g = (workspace_size[0]/2,random.randint(1,workspace_size[1]-1),workspace_size[2]-1)
		e = random_environment_blocks()
		return Problem(s,g,e)

	@staticmethod
	def random2():
		s = (random.randint(0,workspace_size[0]-1),workspace_size[1]/2,0)
		g = (workspace_size[0]/2,random.randint(1,workspace_size[1]-1),workspace_size[2]-1)
		e = random_environment_independent()
		return Problem(s,g,e)

	def features(problem):
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
			if sphere_collision(x,robot_radius,problem.env):
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
		env_relevance = np.zeros(workspace_size)
		for i in range(nsteps+1):
			if nsteps == 0:
				x = problem.start
			else:
				x = vectorops.interpolate(problem.start,problem.goal,float(i)/nsteps)
			for (a,b,c) in sphere_collision_tests(x,robot_radius,problem.env):
				if feasible or problem.env[(a,b,c)]:
					env_relevance[(a,b,c)] = 1 
		return [1]*3+[1]*3+env_relevance.flatten().tolist()


if __name__ == '__main__':
	import sys
	N = 1
	if len(sys.argv) > 1:
		N = int(sys.argv[1])
	learning.tester.set_problem("sphere(%d,%d,%d)"%workspace_size)
	learning.tester.set_output_file("sphere_%dx%dx%d.json"%workspace_size)
	learning.tester.set_data_folder("sphere_test")
	learning.tester.add_sampler("blocks",Problem.random)
	learning.tester.add_sampler("independent",Problem.random2)
	#learning.tester.add_dataset("blocks",'train_blocks_1000.csv','test_blocks_1000.csv','train_relevance_blocks_1000.csv')
	#learning.tester.add_dataset("independent",None,'test_independent_10000.csv')
	learning.tester.add_method('decision tree')
	#learning.tester.add_method('irrelevance expanded decision tree')
	learning.tester.add_method('irrelevance decision tree')
	#learning.tester.add_method('random forest',Ntrees=10)
	learning.tester.add_method('random forest',Ntrees=50)
	#learning.tester.add_method('random forest',Ntrees=100)
	#learning.tester.add_method('irrelevance random forest',Ntrees=10)
	#learning.tester.add_method('irrelevance random forest',Ntrees=50)
	#learning.tester.add_method('irrelevance random forest',Ntrees=100)
	learning.tester.add_scenario(Ntrain=1000,Ntest=1000,testing='blocks')
	learning.tester.add_scenario(Ntrain=5000,Ntest=1000,testing='blocks')
	learning.tester.add_scenario(Ntrain=10000,Ntest=1000,testing='blocks')
	#learning.tester.add_scenario(Ntrain=50000,Ntest=1000,testing='blocks')
	#learning.tester.add_scenario(Ntrain=100000,Ntest=1000,testing='blocks')
	#learning.tester.add_scenario(Ntrain=1000,Ntest=1000,testing='independent')
	#learning.tester.add_scenario(Ntrain=5000,Ntest=1000,testing='independent')
	#learning.tester.add_scenario(Ntrain=10000,Ntest=1000,testing='independent')
	#learning.tester.add_scenario(Ntrain=50000,Ntest=1000,testing='independent')
	#learning.tester.add_scenario(Ntrain=100000,Ntest=1000,testing='independent')
	learning.tester.run(N)
