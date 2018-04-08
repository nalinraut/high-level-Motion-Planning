from decisiontree_missing import vote
from database import Database
import math
import numpy as np
import random
import time
from collections import defaultdict

def sample(counts):
    """Performs a random sample from a histogram / probability distribution"""
    numvals = sum(counts.itervalues())
    if numvals <= 0:
        return None
    v = random.uniform(0,numvals)
    for (k,cnt) in counts.iteritems():
        if v < cnt: return k
        v -= cnt
    print "Uhhh should not reach here"
    return None

def distance_missing(a,b,weights=None):
    assert len(a)==len(b),"Vectors must have equal length"
    d = 0.0
    if weights is None:
        return np.linalg.norm(a-b)
        for (u,v) in zip(a,b):
            if u is None or v is None:
                continue
            d += (u-v)**2
    else:
        assert len(a)==len(weights),"Weight vector must have equal length"
        return np.linalg.norm(np.multiply(a-b,weights))
        for (u,v,w) in zip(a,b,weights):
            if u is None or v is None:
                continue
            d += w*(u-v)**2
    return d
    #return math.sqrt(d)

def distance_missing_dict(a,b,weights=None):
    assert isinstance(a,(list,np.ndarray))
    assert isinstance(b,dict)
    d = 0.0
    if weights is None:
        for (i,v) in b.iteritems():
            d += (a[i]-v)**2
    else:
        assert len(a)==len(weights),"Weight vector must have equal length"
        for (i,v) in b.iteritems():
            d += (weights[i]*(a[i]-v))**2
    return d
    #return math.sqrt(d)

def sparsify(v,empty=None):
    res = {}
    for i,a in enumerate(v):
        if a!=empty:
            res[i] = a
    return res

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

class NearestNeighbor:
    """A random forest that handles None values as 'don't cares'
    that can take on any value
    """
    def __init__(self,k=1):
        self.k = k
        self.db = None
        self.dbsparse = None
        self.labels = None
        self.weights = None
        self.threshold = None

    def neighbors(self,entry):
        """Returns the k-nearest neighbors of the given entry"""
        assert self.db is not None,"Nearest neighbor classifier is not initialized"
        if self.k == 1:
            best = None
            mind = float('inf')
            if self.dbsparse != None:
                for i,e in enumerate(self.dbsparse.entries):
                    d = distance_missing_dict(entry,e,self.weights)
                    if d < mind:
                        best = i
                        mind = d
            else:
                for i,e in enumerate(self.db.entries):
                    d = distance_missing(entry,e,self.weights)
                    if d < mind:
                        best = i
                        mind = d
            return [best]
        else:
            best = list()
            if self.dbsparse != None:
                for i,e in enumerate(self.dbsparse.entries):
                    d = distance_missing_dict(entry,e,self.weights)
                    if len(best) < self.k or d < best[-1][0]:
                        if len(best) >= self.k:
                            best.pop(-1)
                        best = sorted(best + [(d,i)])
            else:
                for i,e in enumerate(self.db.entries):
                    d = distance_missing(entry,e,self.weights)
                    if len(best) < self.k or d < best[-1][0]:
                        if len(best) >= self.k:
                            best.pop(-1)
                        best = sorted(best + [(d,i)])
            return [i for d,i in best]

    def save(self,fn):
        import pickle
        f = open(fn,'w')
        pickle.dump((self.k,self.db,self.labels,self.weights,self.threshold),f)
        f.close()

    def load(self,fn):
        import pickle
        f = open(fn,'r')
        (self.k,self.db,self.labels,self.weights,self.threshold) = pickle.load(f)
        f.close()
        self.dbsparse = None
        if isinstance(self.db.entries[0],dict):
            self.dbsparse = self.db
        else:
            print "NearestNeighbors: Making sparse database..."
            self.dbsparse = Database()
            self.dbsparse.keys = self.db.keys
            self.dbsparse.entries = [sparsify(e) for e in self.db.entries]
    
    def predict(self,entry):
        """Predicts a label for a feature vector"""
        labels = [self.labels[i] for i in self.neighbors(entry)]
        if self.threshold == None:
            res = vote(labels)
        else:
            res = (1 if sum(labels) >= self.threshold else 0)
        return res
    
    def learn(self,db,labels):
        """Learns from a Database instance. Each entry is given a label.
        NOTE: keeps a pointer to the DB and the labels list.

        Arguments:
        - db: a Database instance containing the data
        - labels: a list of labels, one for each entry in the database.
        """
        self.db = db
        self.labels = labels
        normalize = True
        issparse = False
        for i in range(10):
            e = random.choice(db.entries)
            if isinstance(e,dict) or any(v == None for v in e):
                issparse = True
                break
        ranges = [(float('inf'),-float('inf')) for i in self.db.keys]
        if issparse:
            if isinstance(db.entries[0],dict):
                self.dbsparse = db
            else:
                print "NearestNeighbors: Making sparse database..."
                self.dbsparse = Database()
                self.dbsparse.keys = db.keys
                self.dbsparse.entries = [sparsify(e) for e in db.entries]
            if normalize:
                for e in self.dbsparse.entries:
                    for (k,v) in e.iteritems():
                        ranges[k] = [min(v,ranges[k][0]),max(v,ranges[k][1])]
                self.weights = [(1.0/(b-a) if b != a else 1.0) for (a,b) in ranges]
        else:
            if normalize:
                for e in self.db.entries:
                    for (k,v) in enumerate(e):
                        ranges[k] = [min(v,ranges[k][0]),max(v,ranges[k][1])]
                self.weights = [(1.0/(b-a) if b != a else 1.0) for (a,b) in ranges]

        if self.k > 1:
            #determine optimal classification threshold via L.O.O. cross validation
            nsamples = min(len(self.db.entries),500)
            neg_preds = [0]*(self.k+1)
            pos_preds = [0]*(self.k+1)
            keydistributions = [defaultdict(int) for k in self.db.keys]
            for e in self.db.entries:
                if isinstance(e,list):
                    for i,v in enumerate(e):
                        if v != None:
                            keydistributions[i][v] += 1
                else:
                    for i,v in e.iteritems():
                        keydistributions[i][v] += 1
            self.k += 1
            for index,e in enumerate(self.db.entries[:nsamples]):
                if isinstance(e,list):
                    e = e[:]
                    for i,v in enumerate(e):
                        if v is None:
                            e[i] = sample(keydistributions[i])
                else:
                    eflat = [sample(keydistributions[i]) for i,k in enumerate(self.db.keys)]
                    eflat = [0 for k in self.db.keys]
                    for i,v in e.iteritems():
                        eflat[i] = v
                    e = eflat

                n = self.neighbors(e)
                if index not in n:
                    #there's another entry at 0 distance
                    n.pop(-1)
                else:
                    n.remove(index)
                labels = [self.labels[i] for i in n]
                if self.labels[index]:
                    pos_preds[sum(labels)] += 1
                else:
                    neg_preds[sum(labels)] += 1
            self.k -= 1
            if sum(neg_preds) == 0 or sum(pos_preds) == 0:
                print "Sample is either all positive or negative, not using threshold"
            else:
                wneg = 1.0/sum(neg_preds)
                wpos = 1.0/sum(pos_preds)
                self.threshold = -1
                nneg = 0
                npos = sum(pos_preds)
                bestaccuracy = wneg*nneg + wpos*npos
                for i in range(self.k+1):
                    thresh = (float(i)+0.5)
                    nneg += neg_preds[i]
                    npos -= pos_preds[i]
                    accuracy = wneg*nneg + wpos*npos
                    if accuracy > bestaccuracy:
                        self.threshold = thresh
                        bestaccuracy = accuracy
                print "Negative thresholds",neg_preds
                print "Positive thresholds",pos_preds
                print "Dynamically determined classification threshold:",self.threshold

    def loo_accuracy(self):
        """Returns an estimate of the accuracy of the classifier using
        leave-one-out cross validation."""
        self.k += 1
        errors = 0
        keydistributions = [defaultdict(int) for k in self.db.keys]
        for e in self.db.entries:
            if isinstance(e,list):
                for i,v in enumerate(e):
                    if v != None:
                        keydistributions[i][v] += 1
            else:
                for i,v in e.iteritems():
                    keydistributions[i][v] += 1
        progress = ProgressUpdater(len(self.db.entries))
        for index,(l,e) in enumerate(zip(self.labels,self.db.entries)):
            progress.update()
            #fill in none values with randoms
            e = e[:]
            for i,v in enumerate(e):
                if v is None:
                    e[i] = sample(keydistributions[i])
            n = self.neighbors(e)
            if index not in n:
                #there's another entry at 0 distance
                n.pop(-1)
            else:
                n.remove(index)
            labels = [self.labels[i] for i in n]
            if self.threshold == None:
                pred = vote(labels)
            else:
                pred = (1 if sum(labels) >= self.threshold else 0)
            if pred != l:
                errors += 1
        progress.done()
        self.k -= 1
        return 1.0 - float(errors) / len(self.db.entries)
        