"""A decision tree where each example is attached to sets of potentially valid labels.  The loss
function is 0 if the decision tree predicts an element in an example's set and 1 otherwise.

The greedy learning algorithm acts as usual, but label voting is performed by looking over the
most present label in all of the sets.
"""
import math
import weakref
from collections import defaultdict
import database
import numpy as np
import heapq
import itertools
from decisiontree_missing import *

def set_split_misclassification_error(split_labelsets):
    """A list of split data where each entry is a list of labelsets for all
    nodes that would go down that split.
    """
    return sum(set_misclassification_error(labelset) for labelset in split_labelsets)

def greedy_set_cover(items,sets):
    """
    Input:
    - items: a list of items to be covered
    - sets: a list of sets containing items in the list items

    Returns the pair (covers,counts) where covers is the list of sets covering the
    given items and counts is the number of uncovered items added by the i'th set.

    If the sets do not cover the given items, a subset of sets covering the most
    items as possible is returned.

    This is the standard greedy algorithm with an approximation factor of 2 and
    running time O(mn log n) where n is the number of items and n is the number of
    sets.  
    """
    uncovered = set(items)
    cover = []
    counts = []
    sets = sets[:]
    while len(uncovered) > 0 and len(sets) > 0:
        cnt,i,s = max((len(uncovered & s),i,s) for i,s in enumerate(sets))
        if cnt == 0: break
        cover.append(s)
        counts.append(cnt)
        uncovered -= s
        del sets[i]
    return cover,counts

def greedy_set_cover2(labelsets):
    alllabels = set()
    for s in labelsets:
        alllabels |= s
    indices = dict((l,i) for (i,l) in enumerate(alllabels))
    labelcovers = [[] for i in indices]
    for (i,s) in enumerate(labelsets):
        for l in s:
            labelcovers[indices[l]].append(i)
    for i,v in enumerate(labelcovers):
        labelcovers[i] = set(v)
    return greedy_set_cover(range(len(labelsets)),labelcovers)

def set_split_information_gain(split_labelsets):
    """Input: a list of split data where each entry is a list of labelsets for all
    nodes that would go down that split.

    Output: the information gain of splitting a single-layer decision tree
    where each child node votes for the best label
    """
    ig = 0
    totallabels = []
    for i,s in enumerate(split_labelsets):
        cnt = len(s)
        if cnt == 0: continue
        totallabels += s
        sets,counts = greedy_set_cover2(s)
        ps = [float(count)/cnt for count in counts]
        ig += cnt*entropy(ps)
    cnt = len(totallabels)
    if cnt == 0: return 0
    sets,counts = greedy_set_cover2(totallabels)
    ps = [float(count)/cnt for count in counts]
    ig -= cnt*entropy(ps)
    return ig

def set_split_cost(split_labelsets):
    """For a list of split data where each entry is a list of labelsets for
    all nodes that would go down that split.

    Currently uses the negative of information gain.
    """
    return -set_split_information_gain(split_labelsets)
    #this cost value is the misclassification error.
    return set_split_misclassification_error(split_labelsets)

def set_best_split(values,labelsets,nonelabels=None):
    """Given a list of values and associated labels, optimizes the best split
    threshold z where dividing the values into <= z and > z has the
    lowest split cost.

    Returns a pair (z,cost) where cost is the split_cost of the threshold z.

    If nonelabels is given, this indicates the labels of missing values that
    must be passed down to both subtrees.  This does not affect the output
    z but it does affect the output cost value.
    """
    assert len(values) >= 2
    assert len(values) == len(labelsets)
    N = len(values)
    ilist = sorted((v,l) for (v,l) in zip(values,labelsets))
    leftsets = []
    rightsets = []
    for v,s in ilist:
        rightsets.append(s)
    bestindex = -1
    bestcost = set_split_cost([leftsets,rightsets])

    cost = bestcost
    #costs = [cost]
    #print "Split costs:"
    for i in xrange(len(ilist)):
        v,s = ilist[i]
        rightsets.pop(0)
        leftsets.append(s)            
        if i+1 >= len(ilist) or v == ilist[i+1][0]:
            #no splits when v is equal to the next value
            continue
        cost = set_split_cost([leftsets,rightsets])
        #print "  ",v,leftcount.values(),rightcount.values(),cost
        #costs.append(cost)
        if cost < bestcost:
            bestcost = cost
            bestindex = i
    #raw_input()
    if bestindex < 0:
        #no split found... try splitting in half
        splitval = (ilist[0][0]+ilist[-1][0])*0.5
    else:
        splitval = (ilist[bestindex][0] + ilist[bestindex+1][0])*0.5
    if nonelabels is None:
        return (splitval,bestcost)
    #reevaluate counts
    leftsets = []
    rightsets = []
    for l in nonelabels:
        leftsets.append(l)
        rightsets.append(l)
    for v,s in ilist:
        if v <= splitval:
            leftsets.append(s)
        else:
            rightsets.append(s)
    print "Left sets",leftsets
    print "Right sets",rightsets
    return splitval,set_split_cost([leftsets,rightsets])

def set_vote(items):
    cnt = defaultdict(int)
    for s in items:
        for v in s:
            cnt[v] += 1
    return argmax(cnt)

def set_misclassification_error(items,val=None):
    if val is not None:
        nval = 0
        for s in items:
            if val not in s:
                nval += 1
        return nval
    else:
        bestlabel = set_vote(items)
        return set_misclassification_error(items,bestlabel)


class SetDecisionTreeNode(DecisionTreeNode):
    """
    Same as a DecisionTreeNode but adds extra methods
    """
    def __init__(self,parent=None):
        DecisionTreeNode.__init__(self,parent)
    
    def pick_best_label(self,db,labelsets,ids):
        """Given a indexed database db, a list of labelsets (one for
        each id), and a list of ids to test, sets this node to
        the best label.
        """
        self.type = 'v'
        if len(labelsets) > 0:
            self.value = set_vote([labelsets[id] for id in ids])
        else:
            self.value = None
        return

    def pick_best_split(self,db,labelsets,ids,features=None):
        """Given an index database db, a list of labelsets (one for
        each id), and a list of ids to train on, computes the optimal split
        value.

        It modifies this node to have the optimal split type and value,
        and then returns the quality of the split as computed by the 
        split_cost function.

        If features != None, it is a list of available feature indices
        to use in this split, or a function of 0 arguments that can be
        called to get a list of features.
        """
        idlabels = [labelsets[id] for id in ids]
        if set_misclassification_error(idlabels) == 0:
            #base case: no misclassifications
            self.type = 'v'
            self.value = set_vote(idlabels)
            return 0
        best = None
        bestCost = 0
        splitval = None
        discrete = True
        if features == None:
            if len(ids) < db.numFeatures():
                #look at all present features in the training set
                features = db.getPresentFeatures(ids)
                #print len(features),"of",db.numFeatures(),"features selected"
            else:
                features = range(db.numFeatures())
        elif callable(features):
            features = features()
        for i in features:
            if len(db.entryLists[i]) == 0: continue
            idiscrete = db.discreteFeature[i]
            if idiscrete:
                #set of labels of a certain value
                splitter = defaultdict(list)
                #subset of labels for missing values
                nmissing = []
                for id in ids:
                    val = db[i,id]
                    if val is None:
                        #missing values go down to all splits
                        nmissing.append(labelsets[id])
                        continue
                    splitter[val].append(labelsets[id])
                    if len(splitter) > continuous_variable_threshold:
                        #print "Determined to be a continuous variable"
                        idiscrete = False
                        break
            if idiscrete:
                if len(splitter) <= 1:
                    #only a single value
                    continue
                #count number of missing values in all splits
                cmax = 0
                for k in splitter:
                    splitter[k] += nmissing
                    cmax = max(cmax,len(splitter[k]))
                #shrink by fraction of (# of ids - largest child)/(# of ids)
                scale = (1.0-float(cmax)/float(len(ids)))*len(splitter)
                #evaluate cost
                cost = set_split_cost(splitter.values())*scale
                print "Split on",i,"information gain",-cost
                #,splitter.values()
            else:
                #continuous, need to learn the best split
                vals = []
                presentlabels = []
                nonelabels = []
                for id in ids:
                    val = db[i,id]
                    if val is None:
                        nonelabels.append(labelsets[id])
                        continue
                    vals.append(val)
                    presentlabels.append(labelsets[id])
                if len(vals) <= 1:
                    print "No values for feature",i,"?"
                    print vals
                    continue
                print "Considering continuous split on",i
                s,cost = set_best_split(vals,presentlabels,nonelabels)
                scale = (1.0-float(len(presentlabels)/2+len(nonelabels))/float(len(ids)))*2
                cost *= scale
                print "Result",s,"Information gain",-cost
            
            if cost < bestCost:
                best = i
                bestCost = cost
                discrete = idiscrete
                if not idiscrete:
                    splitval = s
        
        if best is None:
            self.type = 'v'
            if len(ids) > 0:
                self.value = set_vote(idlabels)
                return set_misclassification_error(idlabels)
            else:
                self.value = None
                return 0
        else:
            self.feature = best
            #discrete or inequality split
            if discrete:
                self.type = 's'
            else:
                self.type = 'i'
                self.value = splitval
        return bestCost
            
class SetDecisionTree (DecisionTree):
    """A decision tree learner that accepts label sets.

    It also handles None values as 'don't cares'
    that match both sides of each DT split during learning.

    Attributes (same as DecisionTree):
    - root: a SetDecisionTreeNode at the root of the tree
    - keys: a list of string valued keys for the database featuers
    - maxdepth: a depth limit
    - minexamples: if a training set has fewer than this number of values,
      just vote.  A higher value will improve robustness to noise.
    - maxnodes: a maximum number of nodes for the learned tree.
    """
    def __init__(self):
        DecisionTree.__init__(self)
    
    def learn(self,db,labelsets):
        """Learns from a Database instance. Each entry is given a label.

        Arguments:
        - db: a Database instance containing the data
        - labelsets: either a list of label sets.
        - issparse: if True, each entry of the database is a dict.

        Returns the number of misclassified training examples
        """
        self.keys = db.keys[:]
        assert len(labelsets) == len(db.entries)
        warned = False
        for i,v in enumerate(labelsets):
            if isinstance(v,(list,tuple)):
                labelsets[i] = set(v)
            elif not hasattr(v,'__iter__'):
                labelsets[i] = set([v])
                if not warned:
                    print "Warning, converting single label to 1-item label set"
                    warned = True
        self.root = SetDecisionTreeNode()
        db = IndexedDatabase(db)
        if self.maxnodes != None:
            return self.greedy_learn_search(db,labelsets)
        else:
            return self.greedy_learn(self.root,db,labelsets,range(len(labelsets)))

    def greedy_learn(self,node,db,labels,ids):
        """Given a indexed database, greedily and recursively learns the
        split value for the subtree of the indicated node.

        Return value is the number of mistakes made by the decision tree.

        Missing values are handled properly as indicating a 'don't care'
        value that gets passed down to both sides of the tree.
        """
        if node.depth >= self.maxdepth or len(ids) <= self.minexamples:
            #terminate recursion
            node.pick_best_label(db,labels,ids)
            err = set_misclassification_error([labels[id] for id in ids])
            if err > 0:
                print "Reached a leaf and had to make some sacrifices, cost",err
                print "  depth",node.depth
                print "  labels",[labels[id] for id in ids]
            return err

        features = self.feature_subset(node,db,labels,ids)
        cost = node.pick_best_split(db,labels,ids,features)
        
        #do a split
        if node.type == 'v':
            #base case: no misclassifications
            """
            if cost>0:
                print "greedy_learn: Warning, pick_best_split indicates a leaf but the cost is nonzero"
                print "cost=",cost,"misclassification=",misclassification_error([labels[id] for id in ids])
                print "# of ids:",len(ids)
                for i in ids:
                    print "id",i,",",
                    for k in range(db.numFeatures()):
                        if db[k,i] != None:
                            print k,"=",db[k,i],",",
                    print "label",labels[i]
                raw_input()
            """
            return 0
        elif node.type == 's':
            #print "Picked feature",node.feature,"split"
            #do a discrete split
            node.children = dict()
            #select sub-indices
            Eids = defaultdict(list)
            noneids = []
            for id in ids:
                v = db[node.feature,id]
                if v is None:
                    #item doesn't exist, it's a missing value
                    noneids.append(id)
                else:
                    Eids[v].append(id)
            #print "  split sizes:",[len(x) for x in Eids.values()]
            #print "  None ids:",len(noneids)
            ids = None
            errors = 0
            for v,vids in Eids.iteritems():
                #recurse
                c = SetDecisionTreeNode(node)
                #print "Recursing on value",v
                #print "   ids:",vids
                errors += self.greedy_learn(c,db,labels,vids+noneids)
                node.children[v] = c
            return errors
        else:
            #do an inequality split
            assert node.type == 'i'
            #print "Picked feature",node.feature,"inequality value",node.value,"cost",cost
            leftids = []
            rightids = []
            for id in ids:
                if db[node.feature,id] is not None:
                    if db[node.feature,id] <= node.value: leftids.append(id)
                    else: rightids.append(id)
                else:
                    leftids.append(id)
                    rightids.append(id)
            if len(rightids) == len(ids) or len(leftids) == len(ids):
                #due to missing values, this split is useless
                errors = set_misclassification_error([labels[id] for id in ids])
                print "useless split on feature",node.feature,"value",node.value,"misclassification error",errors
                print "Left size",len(leftids),"right size",len(rightids)
                raw_input()
                node.pick_best_label(db,labels,ids)
                return errors
            #clear memory associated with ids list
            del ids[:]
            ids = None
            #print "Left size",len(leftids),"right size",len(rightids)
            c1 = SetDecisionTreeNode(node)
            c2 = SetDecisionTreeNode(node)
            #left side
            errors = self.greedy_learn(c1,db,labels,leftids)
            #right side
            errors += self.greedy_learn(c2,db,labels,rightids)
            #restore index
            node.children = {0:c1,1:c2}
            return errors

    def greedy_learn_search(self,db,labels):
        """Same as greedy learn, but with a maximum number of nodes.
        Rather than a DFS, this uses a priority queue that at each step 
        splits the node with the maximum improvement in misclassification error.
        At most maxnodes are in the resulting tree, and the depth is limited to maxdepth.

        Returns the total number of misclassifications of the training set.

        There is a low-memory mode when self.lowmem == True or self.lowmem == 'auto' 
        and the number of saved ids at a node grows beyond a certain number
        (self.lowmem_threshold, 10m by default).  In low-memory mode, the subset of
        of examples at a given node is determined dynamically, which incurs a O(|D|d)
        cost per node, where d is the depth of the node.  Overall this raises running
        time by a factor of approximately O(|D| log_2 |D|).
        """
        queue = PriorityQueue()
        dolowmem = (self.lowmem == True)
        numidsets = 0
        root_ids = range(len(labels))
        queue.push((self.root,root_ids),len(labels))
        numnodes = 1
        err = 0
        while len(queue) > 0 and numnodes+2 <= self.maxnodes:
            #print "%d nodes, priority %d"%(numnodes,queue.nextkey())
            nerr = queue.nextkey()
            (node,trainingset) = queue.pop()
            #print "Greedy learn",len(trainingset)
            if trainingset is None:
                trainingset = self.identify_examples(db,labels,node)
            if node.depth >= self.maxdepth or len(trainingset) <= self.minexamples:
                #print "  Hit depth or training set limit"
                node.pick_best_label(db,labels,trainingset)
                err += set_misclassification_error([labels[id] for id in trainingset])
                continue
            features = self.feature_subset(node,db,labels,trainingset)
            cost = node.pick_best_split(db,labels,trainingset,features)
            numidsets -= len(trainingset)
            #do a split
            if node.type == 'v':
                continue
            elif node.type == 's':
                #discrete split
                node.children  = dict()
                #select sub-indices
                Eids = defaultdict(list)
                noneids = []
                for id in trainingset:
                    v = db[node.feature,id]
                    if v is None:
                        #item doesn't exist, it's a missing value
                        noneids.append(id)
                    else:
                        Eids[v].append(id)
                #determine whether to switch to low-memory mode
                if not dolowmem and self.lowmem=='auto':
                    for v,vids in Eids.iteritems():
                        numidsets += len(vids)+len(noneids)
                    if numidsets > self.lowmem_threshold:
                        print "Decision tree learner switching to low-memory mode"
                        dolowmem = True
                trainingset = None


                numnodes += len(Eids)
                #print "Split sizes",[len(v) for v in Eids.itervalues()]
                #print "None size",len(noneids)
                for v,vids in Eids.iteritems():
                    #print "->",len(vids),"+",len(noneids)
                    #recurse
                    c = SetDecisionTreeNode(node)
                    node.children[v] = c
                    err = set_misclassification_error([labels[id] for id in vids+noneids])
                    cids = (None if dolowmem else vids+noneids)
                    queue.push((c,cids),err)
            else:
                #do an inequality split
                assert node.type == 'i',"Got a weird type? "+str(node.type)
                leftids = []
                rightids = []
                for id in trainingset:
                    val = db[node.feature,id]
                    if val is not None:
                        if val <= node.value: leftids.append(id)
                        else: rightids.append(id)
                    else:
                        leftids.append(id)
                        rightids.append(id)
                if len(leftids)==0 or len(rightids)==0:
                    print "node feature "+str(node.feature)+" doesn't have a valid split value "+str(node.value)
                    vals = [db[node.feature,id] for id in trainingset if db[node.feature,id]!=None]
                    print "min,max of training set:",min(vals),max(vals)
                    print "cost is",cost
                    raw_input()
                assert len(leftids) > 0 and len(rightids) > 0
                if not dolowmem and self.lowmem=='auto':
                    numidsets += len(leftids) + len(rightids)
                    if numidsets > self.lowmem_threshold:
                        print "Decision tree learner switching to low-memory mode"
                        dolowmem = True
                trainingset = None
                numnodes += 2
                c1 = SetDecisionTreeNode(node)
                c2 = SetDecisionTreeNode(node)
                node.children = {0:c1,1:c2}
                #print "->",len(leftids)
                #print "->",len(rightids)
                err1 = set_misclassification_error([labels[id] for id in leftids])
                err2 = set_misclassification_error([labels[id] for id in rightids])
                if dolowmem:
                    leftids = None
                    rightids = None
                queue.push((c1,leftids),err1)
                queue.push((c2,rightids),err2)
        #end of recursion. for the rest of the nodes still in the queue, make them leaf nodes
        if len(queue) > 0:
            print "%d nodes remaining in queue, setting to leaves"%(len(queue),)
            for (node,trainingset) in queue:
                node.pick_best_label(db,labels,trainingset)
                err += set_misclassification_error([labels[id] for id in trainingset])
        return err


def test_basic():
    db = database.Database()
    db.keys = ['x1','x2','x3']
    db.entries = [[1,0,0],
                  [0,1,0.1],
                  [0,0,0.2],
                  [1,1,0.3],
                  [None,0,0.4],
                  [1,0,0.5],
                  [1,1,0.6],
                  [1,0,0.7]]
    labelsets = [1,
                0,
                0,
                1,
                1,
                1,
                1,
                1]
    labelsets = [[0,4,5,7],
                [1],
                [2],
                [3,6],
                [0,4,5,7],
                [0,4,5,7],
                [3,6],
                [0,4,5,7]]
    tree = SetDecisionTree()
    errors = tree.learn(db,labelsets)
    print "Decision tree makes",errors,"errors"
    print "Depth",tree.depth(),"nodes",tree.numNodes()
    if tree.numNodes() < 100:
        tree.pprint()

    print "Prediction on 0,0,-1:",tree.predict([0,0,-1])
    x = LazyList([(lambda : 0), (lambda : 0), (lambda : -1)])
    print "Same prediction, lazy list:",tree.predict(x)
    print "Probabilistic",tree.predict([0,0,None])


def hamming_distance(x,y):
    return sum(1 for (a,b) in zip(x,y) if (a!=b))

def test_decision_tree(train,trainlabels,test,testlabels,maxnodes=None):
    """Given a training and testing dataset, builds a decision tree and tests it"""
    tree = SetDecisionTree()
    tree.maxnodes = maxnodes
    errors = tree.learn(train,trainlabels)
    print "Decision tree makes",errors,"errors"
    print "Depth",tree.depth(),"nodes",tree.numNodes()
    if tree.numNodes() < 100:
        tree.pprint()
    if errors > 0:
        print "Training errors:"
        for id,(e,l) in enumerate(zip(train.entries,trainlabels)):
            res = tree.predict(e)
            if res not in l:
                if len(e) > 10:
                    print "  Error on",id,"prediction",res
                else:
                    print "  Error on",e,"prediction",res
    print "Testing error:"
    correct = 0
    for e,l in zip(test.entries,testlabels):
        res = tree.predict(e)
        success = (res == l if not hasattr(l,'__iter__') else res in l)
        if success:
            correct += 1
        else:
            print "Error on",e,"prediction",train.entries[res],"distance",hamming_distance(e,train.entries[res])
            print "Res",res,"labels",l,"max distance",max(hamming_distance(e,train.entries[v]) for v in l)
    Ntest = len(test.entries)
    print "Accuracy: %g"%(float(correct)/Ntest,)

def test_knn(Ntrain,Ntest,dims,k,relevantdims=None,fractionDontCare=0):
    import random
    from sklearn.neighbors import NearestNeighbors,BallTree
    import numpy as np
    from database import Database
    if relevantdims == None:
        relevantdims = dims
    train = Database()
    train.keys = ['d']*dims
    for i in xrange(Ntrain):
        train.entries.append([random.uniform(0,1) for j in range(dims)])
    test = Database()
    test.keys = ['d']*dims
    for i in xrange(Ntrain):
        test.entries.append([random.uniform(0,1) for j in range(dims)])
    nn = BallTree(np.array([v[:relevantdims] for v in train.entries]))
    dist,ind = nn.query([v[:relevantdims] for v in train.entries],k)
    trainlabels = []
    for i in xrange(len(train.entries)):
        trainlabels.append( ind[i,:].tolist() )
    dist,ind = nn.query([v[:relevantdims] for v in test.entries],k)
    testlabels = []
    for i in xrange(len(test.entries)):
        testlabels.append( ind[i,:].tolist() )
    test_decision_tree(train,trainlabels,test,testlabels)

def test_hamming(Ntrain,Ntest,dims,threshold,relevantdims=None,fractionDontCare=0):
    import random
    from sklearn.neighbors import NearestNeighbors,BallTree
    import numpy as np
    from database import Database
    if relevantdims == None:
        relevantdims = dims
    train = Database()
    train.keys = ['d']*dims
    for i in xrange(Ntrain):
        train.entries.append([random.choice([0,1]) for j in range(dims)])
    test = Database()
    test.keys = ['d']*dims
    for i in xrange(Ntrain):
        test.entries.append([random.choice([0,1]) for j in range(dims)])
    nn = BallTree(np.array([v[:relevantdims] for v in train.entries]))
    trainlabels = []
    ind = nn.query_radius([v[:relevantdims] for v in train.entries],math.sqrt(threshold)+0.1)
    print math.sqrt(threshold)+0.1
    for i in xrange(len(train.entries)):
        trainlabels.append( ind[i].tolist() )
    print "Average training label size",float(sum(len(l) for l in trainlabels))/len(trainlabels)
    raw_input("Press enter to continue")
    testlabels = []
    ind = nn.query_radius([v[:relevantdims] for v in test.entries],math.sqrt(threshold)+0.1)
    numNN = 0
    for i in xrange(len(test.entries)):
        if len(ind[i]) == 0:
            d,inds = nn.query([test.entries[i][:relevantdims]],1)
            ind[i] = inds[0,:]
            numNN += 1
        testlabels.append( ind[i].tolist() )
    if numNN > 0:
        print "Augmented testing set with nearest neighbor for",numNN,"examples"
        raw_input("Press enter to continue")
    test_decision_tree(train,trainlabels,test,testlabels)


if __name__ == '__main__':
    #A small test...
    #test_basic()
    
    #testing a knn learning problem
    #test_knn(Ntrain=1000,Ntest=1000,dims=5,k=10,relevantdims=2,fractionDontCare=0)

    #testing a hamming distance learning problem
    test_hamming(Ntrain=10000,Ntest=1000,dims=100,threshold=35,relevantdims=100,fractionDontCare=0)    
