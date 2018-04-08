from sklearn import tree
import json
import graphviz
from manual import * 
from playground import *

# masks = {}#json.load(open(DATA_PATH + "/masks.json"))
# initiation_set = json.load(open(DATA_PATH + "/initiation_set.json"))
# effect_set = json.load(open(DATA_PATH + "/effect_set.json"))
groundings = json.load(open(DATA_PATH + "/groundings.json"))

class Opt_Learner:
	############ Takes in a list of Option objects ##############
	def __init__(self,agent,options):
		self.agent=agent
		self.trees={}
		self,masks={}
		self.initiation_set=[]
		self.effect_set=[]
		self.groundings={}

	############ Call this to extract init and effect sets ############# 
	def ExtractAllSets():
		self.ExtractMasks()
		assert bool(self.masks)
		for k,v in enumerate(self.masks): 
			trees[o]=extractInitSet(o)



	############ Create a tree train it; classify sets to init or non-init ############# 
	def extractInitSet(self,o,visualize=False):
		clf = tree.DecisionTreeClassifier()
		batchSize=100	
		data=[]
		labels=[]
		for i in range(batchSize*10):
			s0,_,_=self.agent.randGenSt()
			self.agent.State=s0
			data.append(s0)
			self.agent.exe(o)
			if self.agent.state_chg:
				labels.append(0)
			else
				labels.append(1)
			if (i+1)%batchSize==0:
				clf.fit(data,labels)
				labels=[]
				data=[]
		if visualize:
			visualize_tree(clf)

	########### Visualize trained tree ############
	def visualize_tree(clf):
		dot_data = tree.export_graphviz(clf, out_file=None) 
		graph = graphviz.Source(dot_data) 
		graph.render("test") 

	def randGenSt(self):
		return State()


	########### Extract masks ###########
	def ExtractMasks(self):
		masks={}
		for k,v in enumerate(self.agent.agent_options)
			state=[False]*self.agent.noStates
			for i in range(batchSize*10):
				s0,_,_=self.randGenSt()
				_,mask=self.agent.exe(v)
				state=[1 if (state[i] or mask[i]) else 0 for i in range(len(mask))]
			masks[k]=state


	def partition_options():
		pass
