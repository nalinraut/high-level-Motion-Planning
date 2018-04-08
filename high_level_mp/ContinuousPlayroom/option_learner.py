from sklearn import tree
from sklearn.tree import export_graphviz
import json
from manual import * 
from playground import *
from definitions import *

import pickle
from StringIO import StringIO
from io import BytesIO
import matplotlib.pyplot as plt
import pygraphviz as pgv
import matplotlib.image as img
import matplotlib.pyplot as plt

# masks = json.load(open(DATA_PATH + "/masks.json"))
# initiation_set = json.load(open(DATA_PATH + "/initiation_set.json"))
# effect_set = json.load(open(DATA_PATH + "/effect_set.json"))
groundings = json.load(open(DATA_PATH + "/groundings.json"))

class Opt_Learner:
	############ Takes in a list of Option objects ##############
	def __init__(self,agent):
		self.agent=agent
		self.trees={}
		self.initiation_set=[]
		self.effect_set=[]
		self.groundings={}

	############ Call this to extract init and effect sets ############# 
	def ExtractAllSets(self):
		self.ExtractMasks()
		print ""
		assert bool(self.masks)
		for k,v in self.masks.items():
			option = self.agent.agent_options[k]
			self.trees[k]=self.extractInitSet(option,False)
		print(self.trees)
		tree_pickle = open('tree_pickle.pkl', 'wb')
		pickle.dump(self.trees, tree_pickle)
		tree_pickle.close()
		tree.export_graphviz(self.trees['interact_green_button'], out_file="interact_green_button.dot")
	
	############ Create a tree train it; classify sets to init or non-init ############# 
	def extractInitSet(self,o,visualize=False):
		clf = tree.DecisionTreeClassifier()
		batchSize=1000	
		data=[]
		labels=[]
		count = 0
		for i in range(batchSize*1000):
			s0=self.randGenSt()
			self.agent.state=s0
			data.append(s0.list_states())
			self.agent.execute(o)
			if self.agent.state_chg:
				labels.append(0)
				te=0
			else:
				count = count + 1
				labels.append(1)
				te=1
			if (i+1)%batchSize==0:
				clf.fit(data,labels)
				labels=[]
				data=[]
		
		print self.agent.agent_options.keys()[self.agent.agent_options.values().index(o)]
		print "Positives = "+str(batchSize*1000-count)
		print "Negatives = "+str(count)
		print ""

		if visualize:
			self.visualize_tree(clf)
		return clf

	########### Visualize trained tree ############
	def visualize_tree(self,clf):
		dot_file = StringIO()
		image_file = BytesIO()

	    # Get the dot graph of our decision tree
		export_graphviz(clf, out_file=dot_file, rounded=True, filled=True,
	                    special_characters=True)
		dot_file.seek(0)

	    # Convert this dot graph into an image
		g = pgv.AGraph(dot_file.read())
		g.layout('dot')
	    # g.draw doesn't work when the image object doesn't have a name (with a proper extension)
		image_file.name = "image.png"
		image_file.seek(0)
		g.draw(path=image_file)
		image_file.seek(0)

	    # Plot it
		plt.figure().set_size_inches(*size)
		plt.axis('off')
		plt.imshow(img.imread(fname=image_file))
		plt.show()

	def randGenSt(self):
		return State()


	########### Extract masks ###########
	def ExtractMasks(self):
		self.masks={}
		batchSize=10
		for k,v in self.agent.agent_options.items():
			state=[0]*self.agent.noStates
			for i in range(batchSize*10):
				s0=self.randGenSt()
				self.agent.state=s0
				mask=self.agent.execute(v)
				state=[max(state[j],mask[j]) for j in range(len(mask))] 
			self.masks[k]=state
		
	def partition_options():
		pass


ol=Opt_Learner(Agent(State()))
ol.ExtractAllSets()