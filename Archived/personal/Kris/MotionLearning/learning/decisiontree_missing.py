import math
import weakref
from collections import defaultdict
import database
import numpy as np
import heapq
import itertools

#if a discrete variable has more than this number of possible values, it is treated as continuous
continuous_variable_threshold = 4

class PriorityQueue:
    """Stores a queue in decreasing key order."""
    def __init__(self):
        self.heap = []
    def push(self,item,key):
        heapq.heappush(self.heap,(-key,item))
    def pop(self):
        """Returns the item with the highest valued key"""
        if len(self.heap)==0:
            raise ValueError("Tried popping empty heap")
        return heapq.heappop(self.heap)[1]
    def next(self):
        return self.heap[0][1]
    def nextkey(self):
        return -self.heap[0][0]
    def __len__(self):
        return len(self.heap)
    def __iter__(self):
        return itertools.starmap((lambda x,y:y),self.heap.__iter__())

class IndexedDatabase:
    def __init__(self,db):
        """Given a normal feature matrix, creates a transpose feature matrix,
        a list of discrete features, and a list of non-empty features for
        each id.

        If the dataset is dense, this uses a Numpy matrix to save on space.
        Otherwise, it uses a list-of-dicts structure.
        """
        self._numFeatures = len(db.keys)
        self._numEntries = len(db.entries)
        numMissing = 0
        if isinstance(db.entries[0],dict):
            #already sparse database given as input
            self.featureMatrix = None
            self.featureDicts = [{} for i in range(self._numFeatures)]
            self.discreteFeature = [True]*self._numFeatures
            for i in xrange(self._numFeatures):
                for j in xrange(self._numEntries):
                    if i in db.entries[j]:
                        v = db.entries[j][i]
                        if v != int(v):
                            self.discreteFeature[i] = False
                            break
            self.entryLists = [[] for i in range(self._numFeatures)]
            self.featureSets = []
            for i in xrange(self._numEntries):
                flist = []
                for j in xrange(self._numFeatures):
                    if j in db.entries[i]:
                        flist.append(j)
                        self.entryLists[j].append(i)
                        self.featureDicts[j][i] = db.entries[i][j]
                    else:
                        numMissing += 1
                self.featureSets.append(set(flist))
        else:
            featureMatrix = np.array(db.entries,dtype=np.float_)
            self.featureMatrix = np.asfortranarray(featureMatrix).T
            self.featureDicts = [{} for i in range(self._numFeatures)]
            self.discreteFeature = []
            for i in xrange(self.featureMatrix.shape[0]):
                self.discreteFeature.append(not any(v != int(v) for v in self.featureMatrix[i,:] if not np.isnan(v)))
            self.entryLists = [[] for i in range(self._numFeatures)]
            self.featureSets = []
            for i in xrange(self._numEntries):
                flist = []
                for j in xrange(self._numFeatures):
                    if not np.isnan(featureMatrix[i,j]):
                        flist.append(j)
                        self.entryLists[j].append(i)
                        self.featureDicts[j][i] = featureMatrix[i,j]
                    else:
                        numMissing += 1
                self.featureSets.append(set(flist))
        if numMissing == 0:
            self.featureSets = None
            self.featureDicts = None
        else:
            self.featureMatrix = None
        self.sparsity = float(numMissing) / (self._numFeatures*self._numEntries)

    def __getitem__(self,index):
        if self.featureDicts is None:
            res = self.featureMatrix[index]
            if np.isnan(res):
                return None
            return res
        else:
            return self.featureDicts[index[0]].get(index[1],None)
        
    def numFeatures(self):
        return self._numFeatures
    def numEntries(self):
        return self._numEntries
    def getPresentFeatures(self,ids):
        if self.featureSets == None:
            return range(self._numFeatures)
        fs = set()
        for i in ids:
            fs |= self.featureSets[i]
        return list(fs)


def argmax(table):
    """Given a dict, returns the key that has maximum value (arg max)"""
    return max((v,k) for k,v in table.iteritems())[1]

def split_misclassification_error(label_count_list):
    """Input: a list of splits, where each split is a dictionary mapping
    label values to counts.
    Output: the number of misclassifications in a single-layer decision-tree
    where each child node votes for the best label.
    """
    cnt = 0
    for s in label_count_list:
        if len(s) > 0:
            cnt += sum(s.values()) - max(s.values())
    return cnt

def entropy(p_list):
    """Given a distribution, given by the list p_list, returns the entropy
    of the distribution."""
    assert len(p_list) > 0
    E = 0.0
    for p in p_list:
        if p == 0.0:
            continue
        E += p*math.log(p)
    return E

def split_information_gain(label_count_list):
    """Input: a list of splits, where each split is a dictionary mapping
    label values to counts.
    Output: the information gain of splitting a single-layer decision tree
    where each child node votes for the best label"""
    ig = 0
    totalcounts = defaultdict(int)
    for s in label_count_list:
        for (l,v) in s.iteritems():
            totalcounts[l] += v
        cnt = sum(s.itervalues())
        if cnt == 0: continue
        best = argmax(s)
        p = float(s[best]) / float(cnt)
        ig += cnt*entropy([p,1-p])
    cnt = sum(totalcounts.itervalues())
    if cnt == 0: return 0
    best = argmax(totalcounts)
    p = float(totalcounts[best]) / float(cnt)
    ig -= cnt*entropy([p,1-p])
    return ig

def split_cost(label_count_list):
    """For a list of dictionaries mapping values to counts, returns a
    cost used for DT splitting that is optimal at 0.

    Currently uses the negative of information gain.
    """
    return -split_information_gain(label_count_list)
    #this cost value is the misclassification error.
    return split_misclassification_error(label_count_list)


def best_split(values,labels,nonelabels=None):
    """Given a list of values and associated labels, optimizes the best split
    threshold z where dividing the values into <= z and > z has the
    lowest split cost.

    Returns a pair (z,cost) where cost is the split_cost of the threshold z.

    If nonelabels is given, this indicates the labels of missing values that
    must be passed down to both subtrees.  This does not affect the output
    z but it does affect the output cost value.
    """
    assert len(values) >= 2
    assert len(values) == len(labels)
    N = len(values)
    ilist = sorted((v,l) for (v,l) in zip(values,labels))
    leftcount = defaultdict(int)
    rightcount = defaultdict(int)
    for v,l in ilist:
        rightcount[l] += 1
    bestindex = -1
    bestcost = split_cost([leftcount,rightcount])

    cost = bestcost
    #costs = [cost]
    #print "Split costs:"
    for i in xrange(len(ilist)):
        v,l = ilist[i]
        rightcount[l] -= 1
        leftcount[l] += 1
        if i+1 >= len(ilist) or v == ilist[i+1][0]:
            #no splits when v is equal to the next value
            continue
        cost = split_cost([leftcount,rightcount])
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
    leftcount = defaultdict(int)
    rightcount = defaultdict(int)
    for l in nonelabels:
        leftcount[l] += 1
        rightcount[l] += 1
    for v,l in ilist:
        if v <= splitval:
            leftcount[l] += 1
        else:
            rightcount[l] += 1
    return splitval,split_cost([leftcount,rightcount])

def vote(items):
    cnt = defaultdict(int)
    for v in items:
        cnt[v] += 1
    return argmax(cnt)

def misclassification_error(items,val=None):
    if val is not None:
        nval = 0
        for v in items:
            if v == val: nval += 1
        return len(items) - nval
    else:
        cnt = defaultdict(int)
        for v in items:
            cnt[v] += 1
        return len(items) - max(cnt.values())

def normalize(counts):
    """Normalizes an unnormalized histogram / probability distribution"""
    numvals = sum(counts.itervalues())
    if numvals <= 0:
        return counts
    res = dict()
    for (k,cnt) in counts.iteritems():
        res[k] = float(cnt)/float(numvals)
    return res

class DecisionTreeNode:
    """
    Attributes:
    - type: either 's' (split), 'i' (inequality split), 'v' (value)
    - feature: for types 's' and 'i', the index of the feature in the feature
      vector
    - value: if type == 'i', the inequality split threshold.  If type == 'v',
      the predicted value.
    - children: for types 's' and 'i', the dict of children.  In case 'i', the
      keys are 0 and 1 corresponding to below / above the threshold.
    """
    def __init__(self,parent=None):
        self.type = 'v'
        self.feature = None
        self.value = None
        self.children = None
        if parent is None:
            self.depth = 0
            self.parent = None
        else:
            self.depth = parent.depth+1
            self.parent = weakref.ref(parent)

    def pack(self):
        res = {'type':self.type}
        if self.type in ['i','v']:
            res['value'] = self.value
        if self.type in ['i','s']:
            res['feature'] = self.feature
        if self.children is not None:
            res['children'] = dict((k,c.pack()) for k,c in self.children.iteritems())
        return res

    def unpack(self,map):
        self.type = map['type']
        self.feature = map.get('feature',None)
        self.value = map.get('value',None)
        if 'children' in map:
            cmap = map['children']
            self.children = {}
            for k,cm in cmap.iteritems():
                self.children[k] = DecisionTreeNode(self)
                self.children[k].unpack(cm)
        else:
            self.children = None

    def predict(self,entry):
        """Predicts the label of the given entry.  If it contains None
        elements (missing values), the return value is a probability
        distribution over possible outcomes (given by a dict)."""
        if self.type == 'v':
            return self.value
        v = entry[self.feature]
        if v is None:
            #multiple childrens' predictions
            counts = defaultdict(int)
            labels = self.predict_all(entry,counts)
            if len(counts) == 1:
                return counts.keys()[0]
            #return a probability distribution
            return normalize(counts)
            #maximum likelihood
            #return argmax(counts)
        if self.type == 's':
            c = None
            try:
                c = self.children[v]
            except KeyError:
                #print "Unseen value for feature",self.feature,": ",v
                best = None
                bestDist = float('inf')
                for (val,c) in self.children.iteritems():
                    if abs(val - v) < bestDist:
                        bestDist = abs(val - v)
                        best = c
                c = best
            return c.predict(entry)
        elif self.type == 'i':
            if v <= self.value:
                return self.children[0].predict(entry)
            else:
                return self.children[1].predict(entry)
        raise RuntimeError("Invalid DecisionTreeNode type?")

    def lookup(self,entry):
        """Looks up the leaf node corresponding to the given entry.  Does not
        handle missing values."""
        if self.type == 'v':
            return self
        v = entry[self.feature]
        assert v != None
        if self.type == 's':
            c = None
            try:
                c = self.children[v]
            except KeyError:
                #print "Unseen value for feature",self.feature,": ",v
                best = None
                bestDist = float('inf')
                for (val,c) in self.children.iteritems():
                    if abs(val - v) < bestDist:
                        bestDist = abs(val - v)
                        best = c
                c = best
            return c.lookup(entry)
        elif self.type == 'i':
            if v <= self.value:
                return self.children[0].lookup(entry)
            else:
                return self.children[1].lookup(entry)
        raise RuntimeError("Invalid DecisionTreeNode type?")
    
    def pick_best_label(self,db,labels,ids):
        """Given a indexed database db, a list of labels (one for
        each id), and a list of ids to test, sets this node to
        the best label.
        """
        self.type = 'v'
        if len(labels) > 0:
            self.value = vote([labels[id] for id in ids])
        else:
            self.value = None
        return

    def pick_best_split(self,db,labels,ids,features=None):
        """Given an index database db, a list of labels (one for
        each id), and a list of ids to train on, computes the optimal split
        value.

        It modifies this node to have the optimal split type and value,
        and then returns the quality of the split as computed by the 
        split_cost function.

        If features != None, it is a list of available feature indices
        to use in this split, or a function of 0 arguments that can be
        called to get a list of features.
        """
        idlabels = [labels[id] for id in ids]
        if misclassification_error(idlabels) == 0:
            #base case: no misclassifications
            self.type = 'v'
            self.value = idlabels[0]
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
                #count number of labels of a certain value
                splitter = defaultdict(lambda:defaultdict(int))
                #count of labels for missing values
                nmissing = defaultdict(int)
                for id in ids:
                    val = db[i,id]
                    if val is None:
                        #missing values go down to all splits
                        nmissing[labels[id]] += 1
                        continue
                    splitter[val][labels[id]] += 1
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
                    for l,v in nmissing.iteritems():
                        splitter[k][l] += v
                    cmax = max(cmax,sum(splitter[k].values()))
                #shrink by fraction of (# of ids - largest child)/(# of ids)
                scale = (1.0-float(cmax)/float(len(ids)))*len(splitter)
                #evaluate cost
                cost = split_cost(splitter.values())*scale
                #print "Split on",i,"information gain",-cost,splitter.values()
            else:
                #continuous, need to learn the best split
                vals = []
                presentlabels = []
                nonelabels = []
                for id in ids:
                    val = db[i,id]
                    if val is None:
                        nonelabels.append(labels[id])
                        continue
                    vals.append(val)
                    presentlabels.append(labels[id])
                if len(vals) <= 1:
                    print "No values for feature",i,"?"
                    print vals
                    continue
                #print "Considering continuous split on",i
                s,cost = best_split(vals,presentlabels,nonelabels)
                scale = (1.0-float(len(presentlabels)/2+len(nonelabels))/float(len(ids)))*2
                cost *= scale
                #print "Result",s,"Information gain",-cost
            
            if cost < bestCost:
                best = i
                bestCost = cost
                discrete = idiscrete
                if not idiscrete:
                    splitval = s
        
        if best is None:
            self.type = 'v'
            if len(ids) > 0:
                self.value = vote(idlabels)
                return misclassification_error(idlabels)
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

    def predict_all(self,entry,counts):
        """Computes counts for ALL compatible predictions for the given
        entry, taking missing values into account.  counts is assumed to be
        a defaultdict(int) instance"""
        if self.type == 'v':
            counts[self.value] += 1
            return
        v = entry[self.feature]
        if v is None:
            for val,c in self.children.iteritems():
                c.predict_all(entry,counts)
            return
        if self.type == 's':
            c = None
            try:
                c = self.children[v]
            except KeyError:
                #print "Unseen value for feature",self.feature,": ",v
                best = None
                bestDist = float('inf')
                for (val,c) in self.children.iteritems():
                    if abs(val - v) < bestDist:
                        bestDist = abs(val - v)
                        best = c
                c = best
            c.predict_all(entry,counts)
        elif self.type == 'i':
            if v <= self.value:
                self.children[0].predict_all(entry,counts)
            else:
                self.children[1].predict_all(entry,counts)
        return

def print_indent(indent):
    if indent > 0:
        print " "*(indent-1),
            
class DecisionTree:
    """A decision tree learner that handles None values as 'don't cares'
    that match both sides of each DT split during learning.

    Attributes:
    - root: a DecisionTreeNode at the root of the tree
    - keys: a list of string valued keys for the database featuers
    - maxdepth: a depth limit
    - minexamples: if a training set has fewer than this number of values,
      just vote.  A higher value will improve robustness to noise.
    - maxnodes: a maximum number of nodes for the learned tree.
    """
    def __init__(self):
        self.root = None
        self.keys = []
        self.maxdepth=float('inf')
        self.minexamples=1
        self.maxnodes = None
        self.lowmem = 'auto'
        self.lowmem_threshold = 10000000
    
    def predict(self,entry):
        """Predicts a label for a feature vector"""
        assert self.root is not None,"Decision tree is not initialized"
        return self.root.predict(entry)
    
    def learn(self,db,labels):
        """Learns from a Database instance. Each entry is given a label.

        Arguments:
        - db: a Database instance containing the data
        - labels: either a list of labels or a key.  If it is a list,
          it contains one label for each entry in the database.  If it is a
          string or integer key, it gives the column key or index in the database
          containing label values.
        - issparse: if True, each entry of the database is a dict.

        Returns the number of misclassified training examples
        """
        self.keys = db.keys[:]
        labelindex = -1
        if isinstance(labels,str):
            labelindex = db.keys.index(labels)
            assert labelindex >= 0,"label does not exist in database keys"
            labels = db.get_column(labelindex)
        elif isinstance(labels,int):
            labelindex = labels
            labels = db.get_column(labelindex)
        else:
            assert len(labels) == len(db.entries)
        self.root = DecisionTreeNode()
        if labelindex >= 0:
            raise NotImplementedError("Ooops, taking out indexed label broken")
            entries = np.delete(entries,labelindex,1)
        db = IndexedDatabase(db)
        if self.maxnodes != None:
            return self.greedy_learn_search(db,labels)
        else:
            self.deepest = 0
            return self.greedy_learn(self.root,db,labels,range(len(labels)))
    def depth(self):
        """Returns the maximum depth of the tree"""
        return max(n.depth for n in self.iternodes())
    def numNodes(self):
        """Returns the total number of nodes in the tree"""
        res = 0
        for n in self.iternodes():
            res += 1
        return res
    def nodes(self):
        """Returns all nodes in the tree""" 
        return [n for n in self.iternodes()]
    def iternodes(self,root=None):
        if root is None:
            root = self.root
            if root is None:
                return
        yield root
        if root.children is not None:
            for val,c in root.children.iteritems():
                for n in self.iternodes(c):
                    yield n
        return
    def pprint(self,indent=0,node=None):
        """Pretty prints the tree"""
        if node == None:
            node = self.root
            if node == None:
                print_indent(indent)
                print "[empty tree]"
                return
        if node.type == 'v':
            print_indent(indent)
            print node.value
        elif node.type == 's':
            for (val,c) in node.children.iteritems():
                print_indent(indent)
                print "-",self.keys[node.feature],"=",val,":"
                self.pprint(indent+1,c)
        elif node.type == 'i':
            print_indent(indent)
            print self.keys[node.feature],"<=",node.value,":"
            self.pprint(indent+1,node.children[0])
            print_indent(indent)
            print self.keys[node.feature],">",node.value,":"
            self.pprint(indent+1,node.children[1])

    def pack(self):
        return {'root':self.root.pack(),'keys':self.keys}

    def unpack(self,map):
        self.root = DecisionTreeNode()
        self.root.unpack(map['root'])
        self.keys = map['keys']

    def save(self,fn):
        import pickle
        f = open(fn,"w")
        structure = self.pack()
        structure['maxdepth'] = self.maxdepth
        structure['minexamples'] = self.minexamples
        structure['maxnodes'] = self.maxnodes
        pickle.dump(structure,f)
        f.close()

    def load(self,fn):
        import pickle
        f = open(fn,'r')
        v = pickle.load(f)
        self.unpack(v)
        self.maxdepth = v['maxdepth']
        self.minexamples = v['minexamples']
        self.maxnodes = v['maxnodes']
        f.close()

    def feature_subset(self,node,db,labels,ids):
        """Can overload this to choose different features"""
        return None

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
            err = misclassification_error([labels[id] for id in ids])
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
                c = DecisionTreeNode(node)
                #print "Recursing on value",v
                #print "   ids:",vids
                errors += self.greedy_learn(c,db,labels,vids+noneids)
                node.children[v] = c
                if c.depth > self.deepest:
                    self.deepest = c.depth
                    print "Decision tree learner: Reached node with depth",self.deepest
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
                errors = misclassification_error([labels[id] for id in ids])
                print "useless split on feature",node.feature,"value",node.value,"misclassification error",errors
                print "Left size",len(leftids),"right size",len(rightids)
                raw_input()
                node.pick_best_label(db,labels,ids)
                return errors
            #clear memory associated with ids list
            del ids[:]
            ids = None
            #print "Left size",len(leftids),"right size",len(rightids)
            c1 = DecisionTreeNode(node)
            c2 = DecisionTreeNode(node)
            #left side
            errors = self.greedy_learn(c1,db,labels,leftids)
            #right side
            errors += self.greedy_learn(c2,db,labels,rightids)
            #restore index
            node.children = {0:c1,1:c2}
            if c1.depth > self.deepest:
                self.deepest = c1.depth
                print "Decision tree learner: Reached node with depth",self.deepest
            return errors

    def identify_examples(self,db,labels,node):
        """Identifies the list of example indices that would follow the decision tree to
        node."""
        path = []
        while node.parent != None:
            nkey = None
            for (k,c) in node.parent().children.iteritems():
                if c is node:
                    nkey = k
                    break
            assert nkey != None
            path.append((node.parent(),nkey))
            node = node.parent()
        path = path[::-1]
        nids = len(labels)
        ids = []
        for id in xrange(nids):
            valid = True
            for n,ckey in path:
                f = n.feature
                val = featureMatrix[f,id]
                if val is None:
                    #it's a None value, just continue on
                    continue
                else:
                    key = None
                    if n.type == 'i':
                        key = (0 if val <= n.value else 1)
                    else:
                        key = val
                    if key != ckey:
                        valid = False
                        break
            if valid:
                ids.append(id)
        return ids


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
        deepest = 0
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
                err += misclassification_error([labels[id] for id in trainingset])
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
                    c = DecisionTreeNode(node)
                    node.children[v] = c
                    err = misclassification_error([labels[id] for id in vids+noneids])
                    cids = (None if dolowmem else vids+noneids)
                    queue.push((c,cids),err)
                    if c.depth > deepest:
                        deepest = c.depth
                        print "Decision tree learner: Reached node with depth",deepest
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
                c1 = DecisionTreeNode(node)
                c2 = DecisionTreeNode(node)
                node.children = {0:c1,1:c2}
                #print "->",len(leftids)
                #print "->",len(rightids)
                err1 = misclassification_error([labels[id] for id in leftids])
                err2 = misclassification_error([labels[id] for id in rightids])
                if dolowmem:
                    leftids = None
                    rightids = None
                queue.push((c1,leftids),err1)
                queue.push((c2,rightids),err2)
                if c1.depth > deepest:
                    deepest = c1.depth
                    print "Decision tree learner: Reached node with depth",deepest
        #end of recursion. for the rest of the nodes still in the queue, make them leaf nodes
        if len(queue) > 0:
            print "%d nodes remaining in queue, setting to leaves"%(len(queue),)
            for (node,trainingset) in queue:
                node.pick_best_label(db,labels,trainingset)
                err += misclassification_error([labels[id] for id in trainingset])
        return err


class LazyList:
    """A lazy list compatible with many list operators.  You can set
    an entry [i] to a delayed evaluation of a zero-argument lambda function
    which gets evaluated when [i] is accessed.

    For example, this is compatible with DecisionTree.predict(x).
    """
    def __init__(self,entries=None,memoized=False):
        """Initializes the list.  If entries is given, this
        initializes the entries of the list.  If memoized = True,
        any lazy evaluated entries are saved after their first evaluation.
        """
        if entries is not None:
            self.entries = entries[:]
        else:
            self.entries = []
        self.memoized = memoized
    def __getitem__(self,key):
        v = self.entries[key]
        if callable(v):
            res = v()
            if self.memoized:
                self.entries[key] = v
            return res
        else:
            return v
    def __setitem__(self,key,lambda_or_value):
        self.entries[key] = lambda_or_value

def test_basic():
    db = database.Database()
    db.keys = ['x1','x2','x3','label']
    db.entries = [[1,0,0,  1],
                  [0,1,0.1,  0],
                  [0,0,0.2,  0],
                  [1,1,0.3,  1],
                  [None,0,0.4,  1],
                  [1,0,0.5,  1],
                  [1,1,0.6,  1],
                  [1,0,0.7,  1]]
    tree = DecisionTree()
    errors = tree.learn(db,'label')
    print "Decision tree makes",errors,"errors"
    print "Depth",tree.depth(),"nodes",tree.numNodes()
    if tree.numNodes() < 100:
        tree.pprint()

    print tree.predict([0,0,-1])
    x = LazyList([(lambda : 0), (lambda : 0), (lambda : -1)])
    print tree.predict(x)
    print tree.predict([0,0,None])

def gen_random(N,dims,widthTarget=0.5,fractionDontCare=0.0,noiseProbability=0.0):
    import random
    assert dims >= 2
    db = database.Database()
    db.keys = ['x'+str(i+1) for i in range(dims)]+['label']
    for i in xrange(N):
        e = [random.uniform(0,1) for j in range(dims)]
        label = (abs(e[0]-0.5) <= widthTarget/2) != (abs(e[1]-0.5) <= widthTarget/2)
        e.append(label)
        if random.random() < noiseProbability:
            e[-1] = not e[-1]
        #some fraction of missing values
        order = range(2,dims)
        random.shuffle(order)
        for j in order:
            if random.random() < fractionDontCare:
                e[j] = None
        db.entries.append(e)
    #for e in db.entries:
    #    print e
    return db

def gen_deterministic(N,dims,widthTarget=0.5,fractionDontCare=0.0,noiseProbability=0.0):
    import random
    assert dims >= 5
    db = database.Database()
    db.keys = ['x'+str(i+1) for i in range(dims)]+['label']
    for i in xrange(N):
        e = [random.uniform(0,1) for j in range(dims)]
        relevant =  [1,2] if e[0] >= 0.5 else [3,4]
        irrelevant =  [3,4] if e[0] >= 0.5 else [1,2]
        label = all(abs(e[d]-0.5) <= widthTarget/2 for d in relevant)
        e.append(label)
        if random.random() < noiseProbability:
            e[-1] = not e[-1]
        #some fraction of missing values
        order = range(5,dims)
        random.shuffle(order)
        for j in order:
            if random.random() < fractionDontCare:
                e[j] = None
        for j in irrelevant:
            if random.random() < fractionDontCare:
                e[j] = None
        db.entries.append(e)
    #for e in db.entries:
    #    print e
    return db

def test_decision_tree(train,test,maxnodes=None):
    """Given a training and testing dataset, builds a decision tree and tests it"""
    tree = DecisionTree()
    tree.maxnodes = maxnodes
    errors = tree.learn(train,'label')
    print "Decision tree makes",errors,"errors"
    print "Depth",tree.depth(),"nodes",tree.numNodes()
    if tree.numNodes() < 100:
        tree.pprint()
    if errors > 0:
        print "Training errors:"
        for id,e in enumerate(train.entries):
            res = tree.predict(e[:-1])
            if res != e[-1]:
                if len(e[:-1]) > 10:
                    print "  Error on",id,"prediction",res
                else:
                    print "  Error on",e[:-1],"prediction",res
    print "Testing error:"
    tp,tn,fp,fn = 0,0,0,0
    for e in test.entries:
        res = tree.predict(e[:-1])
        if res and e[-1]:
            tp += 1
        elif res and not e[-1]:
            fp += 1
        elif not res and e[-1]:
            fn += 1
        else:
            tn += 1
    Ntest = len(test.entries)
    print "True +: %g, True -: %g"%(float(tp)/Ntest,float(tn)/Ntest)        
    print "False -: %g, False +: %g"%(float(fn)/Ntest,float(fp)/Ntest)
    print "Overall error: %g"%(float(fn+fp)/Ntest,)


def test_random(N,dims,widthTarget=0.5,fractionDontCare=0.1,noiseProbability=0.0,Ntest=None):
    if Ntest==None:
        Ntest = N
    #train = gen_random(N,dims,widthTarget,fractionDontCare)
    train = gen_random(N,dims,widthTarget,fractionDontCare,noiseProbability)
    test = gen_random(Ntest,dims,widthTarget,0.0,0.0)
    test_decision_tree(train,test)

def test_deterministic(N,dims,widthTarget=0.5,fractionDontCare=0.1,noiseProbability=0.0,Ntest=None):
    if Ntest==None:
        Ntest = N
    #train = gen_random(N,dims,widthTarget,fractionDontCare)
    train = gen_deterministic(N,dims,widthTarget,fractionDontCare,noiseProbability)
    test = gen_deterministic(Ntest,dims,widthTarget,0.0,0.0)
    test_decision_tree(train,test)

if __name__ == '__main__':
    #A small test...
    #test_basic()
    
    #Test how with very few data points (N) and a fraction of "don't care"
    #features, the generalization performance of the classifier can be
    #significantly boosted.
    #
    #This example generates 100 points from the hypercube [0,1]^100, and tries
    #to learn the concept  (|x1-0.5| < widthtarget/2 && |x2-0.5| < widthtarget/2).
    #
    #even with "don't cares" on only 5% of the irrelevant dimensions, accuracy
    #is greatly boosted (around 90%?) and it is much more likely to use x1
    #and x2 as decision variables.
    #At 10%, accuracy is better still (98%) and I have never seen it learn
    #irrelevant features in the decision tree.
    #test_random(N=1000,dims=100,widthTarget=0.5,fractionDontCare=0.0,noiseProbability=0.05,Ntest=10000)
    #test_deterministic(N=300,dims=1000,widthTarget=0.5,fractionDontCare=0,noiseProbability=0.0,Ntest=10000)
    test_deterministic(N=300,dims=100,widthTarget=0.5,fractionDontCare=0.5,noiseProbability=0.0,Ntest=1000)
