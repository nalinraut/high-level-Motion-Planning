from decisiontree_missing import vote,DecisionTree,DecisionTreeNode
from database import Database
import random
import math
from utils import sample_weighted
import multiprocessing
import signal
import time

class RandomDecisionTree(DecisionTree):
    def __init__(self,numSplitFeatures):
        self.numSplitFeatures = numSplitFeatures
        self.featureWeights = None
        DecisionTree.__init__(self)

    def feature_subset(self,node,db,labels,ids):
        if self.numSplitFeatures == None:
            return None
        else:
            if self.featureWeights == None:
                self.featureWeights = [len(db.entryLists[i]) for i in range(db.numFeatures())]
            if len(ids) < db.numFeatures():
                #look at all present features in the training set
                features = db.getPresentFeatures(ids)
            else:
                features = range(db.numFeatures())
            Nfeatures = len(features)
            N = None
            if self.numSplitFeatures == 'sqrt':
                N = math.sqrt(Nfeatures)
            elif isinstance(self.numSplitFeatures,int):
                N = self.numSplitFeatures
            elif isinstance(self.numSplitFeatures,float):
                N = self.numSplitFeatures*Nfeatures
            weights = [self.featureWeights[i] for i in features]
            return [features[sample_weighted(weights)] for i in range(int(N))]

def _learn_dt(numSplits,db,labels):
    t = RandomDecisionTree(numSplits)
    dbt = Database()
    dbt.keys = db.keys
    choice = [random.randint(0,len(db.entries)-1) for i in range(len(db.entries))]
    dbt.entries = [db.entries[c] for c in choice]
    if not isinstance(labels,(str,int)):
        assert hasattr(labels,'__iter__')
        labelst = [labels[c] for c in choice]
    else:
        labelst = labels
    #t.maxnodes = self.maxNodesPerTree
    t.maxnodes = len(db.entries)*2
    #print "Beginning decision tree learning"
    t.learn(dbt,labelst)
    return t

def _learn_dt2(args):
    tree = _learn_dt(*args)
    return tree.pack()

def _init_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)

def parallel_execute(func,args,Nthreads):
    """A better way to do parallel execution of a function.  Handles Ctrl+C properly."""
    if Nthreads == 1:
        return [func(a) for a in args]

    p = multiprocessing.Pool(Nthreads,_init_worker)
    res = p.map_async(func,args)
    try:
        while not res.ready():
            time.sleep(1)
    except KeyboardInterrupt:
        print "parallel_execute: Caught KeyboardInterrupt, terminating workers"
        p.terminate()
        while multiprocessing.active_children():
            print "Waiting for active children..."
            time.sleep(1)
        p.join()
        raise 
    else:
        p.close()
        p.join()
    return res.get()

class RandomForest:
    """A random forest that handles None values as 'don't cares'
    that match both sides of each DT split during learning.
    """
    def __init__(self,Ntrees=50):
        self.trees = None
        self.Ntrees = Ntrees
        #self.numSplits = 'sqrt'
        self.numSplits = 0.5
        self.maxNodesPerTree = None
        self.parallelization = 4
    
    def predict(self,entry):
        """Predicts a label for a feature vector"""
        assert self.trees is not None,"Random forest classifier is not initialized"
        predictions = [t.predict(entry) for t in self.trees]
        if any(isinstance(v,float) for v in predictions):
            #it's a continuous prediction
            return sum(predictions)/len(predictions)
        else:
            #it's an integer prediction
            return vote(predictions)
    
    def learn(self,db,labels):
        """Learns from a Database instance. Each entry is given a label.

        Arguments:
        - db: a Database instance containing the data
        - labels: either a list of labels or a key.  If it is a list,
          it contains one label for each entry in the database.  If it is a
          string or integer key, it gives the column key or index in the database
          containing label values.
        - maxnodes: a maximum number of nodes for the learned tree
        """
        if self.parallelization == 1:
            self.trees = [_learn_dt(self.numSplits,db,labels) for i in range(self.Ntrees)]
        else:
            print "Learning random forest with",self.Ntrees,"trees and",self.parallelization,"parallel processes"
            self.trees = [DecisionTree() for i in range(self.Ntrees)]
            tmaps = parallel_execute(_learn_dt2,[(self.numSplits,db,labels)]*self.Ntrees,self.parallelization)
            for (t,m) in zip(self.trees,tmaps):
                t.unpack(m)
        return
