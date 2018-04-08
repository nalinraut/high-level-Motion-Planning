import kdtree
from klampt.math import vectorops,so3
import math
import numpy as np
from knn import *

#set this to true if you wish to double-check the results of the kd tree
check_kdtree = False

infty = float('inf')

def expand_se3(x):
    m = x[3:]
    return x[:3] + vectorops.mul(so3.from_moment(m),0.3)

class NearestNeighbors:
    def __init__(self,metric,method='bruteforce'):
        """Accepts either bruteforce, kdtree, or balltree methods.
        If kdtree, metric must be a weighted euclidean metric."""
        self.metric = metric
        self.method = method
        if self.method == 'kdtree':
            self.kdtree = kdtree.KDTree(self.metric)
            if check_kdtree:
                self.checker = NearestNeighbors(self.metric)
                print "Debugging: Double checking KD-tree with nearest neighbors"
        elif self.method == 'balltree':
            try:
                from sklearn.neighbors import BallTree,DistanceMetric
                self.points = []
                self.datas = []
                self.dirty = True
            except ImportError:
                print "NearestNeighbors: scikit-learn is not installed, falling back to brute force"
                self.method = 'bruteforce'
                self.nodes = []
        elif self.method == 'se3balltree':
            try:
                from sklearn.neighbors import BallTree,DistanceMetric
                self.points = []
                self.datas = []
                self.dirty = True
            except ImportError:
                print "NearestNeighbors: scikit-learn is not installed, falling back to brute force"
                self.method = 'bruteforce'
                self.nodes = []
        else:
            self.nodes = []

    def reset(self):
        if self.method == 'kdtree':
            self.kdtree = kdtree.KDTree(self.metric)
            if check_kdtree: self.checker = NearestNeighbors(self.metric)
        else:
            self.nodes = []

    def add(self,point,data=None):
        """Adds a point with an associated datum."""
        if self.method == 'kdtree':
            self.kdtree.add(point,data)
            self.kdtree.rebalance()
            if check_kdtree: self.checker.add(point,data)
        elif self.method == 'balltree' or self.method == 'se3balltree':
            self.points.append(point)
            self.datas.append(data)
            self.dirty = True
        else:
            self.nodes.append((point,data))

    def remove(self,point,data=None):
        """Removes a point, optionally matching the data too.
        Time is O(nearest).  Returns the number of points removed.
        (TODO: can only be 0 or 1 at the moment)."""
        if self.method == 'kdtree':
            res = self.kdtree.remove(point,data)
            self.dirty = True
            if check_kdtree:
                cres = self.checker.remove(point,data)
                if cres != res:
                    raise ValueError("KD-tree did not remove the correct numer of points")
            if res == 0:
                print "KDTree: Unable to remove",point,"does not exist"
            return res
        elif self.method == 'balltree' or self.method == 'se3balltree':
            raise RuntimeError("remove() cannot be called for balltree method")
        else:
            for i,(p,pd) in enumerate(self.nodes):
                if p == point and (data == None or pd==data):
                    del self.nodes[i]
                    return 1
            for p,pd in self.nodes:
                print p,pd
        return 0
            

    def set(self,points,datas=None):
        """Sets the point set to a list of points."""
        if datas==None:
            datas = [None]*len(points)
        if hasattr(self,'kdtree'):
            print "Resetting KD tree..."
            self.kdtree.set(points,datas)
            if check_kdtree: self.checker.set(points,datas)
        elif self.method == 'balltree' or self.method == 'so3balltree':
            self.points = points[:]
            self.datas = datas[:]
            self.dirty = True
        else:
            self.nodes = zip(points,datas)

    def nearest(self,pt,filter=None):
        """Nearest neighbor lookup, possibly with filter"""
        if filter is not None:
            assert callable(filter)
        self.updateDataStructures()
        if self.method == 'kdtree':
            res = self.kdtree.nearest(pt,filter)
            if check_kdtree: 
                rescheck = self.checker.nearest(pt,filter)
                if res != rescheck:
                    print "KDTree nearest(",pt,") error",res,"should be",rescheck
                    print self.metric(res[0],pt)
                    print self.metric(rescheck[0],pt)
            return res
        elif self.method == 'balltree':
            dist,ind = self.balltree.query([pt],1)
            dist = dist[0,:].tolist()
            ind = ind[0,:].tolist()
            return (self.points[ind[0]],self.datas[ind[0]])
        elif self.method == 'se3balltree':
            dist,ind = self.balltree.query([expand_se3(pt)],1)
            dist = dist[0,:].tolist()
            ind = ind[0,:].tolist()
            return (self.points[ind[0]],self.datas[ind[0]])
        else:
            #brute force
            res = None    
            dbest = infty
            for p,data in self.nodes:
                d = self.metric(p,pt)
                if d < dbest and (filter == None or not filter(p,data)):
                    res = (p,data)
                    dbest = d
            return res

    def knearest(self,pt,k,filter=None):
        """K-nearest neighbor lookup, possibly with filter"""
        self.updateDataStructures()
        if self.method == 'kdtree':
            res = self.kdtree.knearest(pt,k,filter)
            if check_kdtree: 
                rescheck = self.checker.knearest(pt,k,filter)
                if res != rescheck:
                    print "KDTree knearest(",pt,") error",res,"should be",rescheck
                    print self.metric(res[0][0],pt)
                    print self.metric(rescheck[0][0],pt)
            return res
        elif self.method == 'balltree':
            dist,ind = self.balltree.query([pt],1)
            dist = dist[0,:].tolist()
            ind = ind[0,:].tolist()
            return [(self.points[i],self.datas[i]) for i in ind]
        elif self.method == 'se3balltree':
            dist,ind = self.balltree.query([expand_se3(pt)],1)
            dist = dist[0,:].tolist()
            ind = ind[0,:].tolist()
            return [(self.points[i],self.datas[i]) for i in ind]
        else:
            #brute force
            res = KNearestResult(k)
            for p,data in self.nodes:
                if (filter == None or not filter(p,data)):
                    d = self.metric(p,pt)
                    res.tryadd(d,(p,data))
            return res.sorted_items()
    
    def neighbors(self,pt,radius):
        """Range query, all points within pt.  Filtering can be done
        afterward by the user."""
        self.updateDataStructures()
        if self.method == 'kdtree':
            res = self.kdtree.neighbors(pt,radius)
            if check_kdtree: 
                rescheck = self.checker.neighbors(pt,radius)
                if len(res) != len(rescheck):
                    print "KDTree neighbors(",pt,",",radius,") error",res,"should be",rescheck
                else:
                    for r in res:
                        if r not in rescheck:
                            print "KDTree neighbors(",pt,",",radius,") error",res,"should be",rescheck
                            break

            return res
        elif self.method == 'balltree':
            import time
            t0 = time.time()
            ind = self.balltree.query_radius([pt],radius)
            #print "Query time",time.time()-t0,"radius",radius,"results",len(ind[0])
            ind = ind[0].tolist()
            return [(self.points[i],self.datas[i]) for i in ind]
        elif self.method == 'se3balltree':
            import time
            t0 = time.time()
            ind = self.balltree.query_radius([expand_se3(pt)],radius)
            #print "Query time",time.time()-t0,"radius",radius,"results",len(ind[0])
            ind = ind[0].tolist()
            return [(self.points[i],self.datas[i]) for i in ind]
        else:
            #brute force
            import time
            t0 = time.time()
            res = []
            for p,data in self.nodes:
                d = self.metric(p,pt)
                if d < radius:
                    res.append((p,data))
            #print "Query time",time.time()-t0,"radius",radius,"results",len(res)
            return res
      
    def updateDataStructures(self):
        if self.method == 'balltree':
            if self.dirty:
                from sklearn.neighbors import BallTree,DistanceMetric
                parray = np.array(self.points)
                print "Building a BallTree with",len(self.points),"points"
                self.balltree = BallTree(parray,metric=DistanceMetric.get_metric("pyfunc",func=self.metric))
                self.dirty = False
        elif self.method == 'se3balltree':
            if self.dirty:
                from sklearn.neighbors import BallTree,DistanceMetric
                parray = np.array([expand_se3(p) for p in self.points])
                print "Building a SE3 BallTree with",len(self.points),"points"
                self.balltree = BallTree(parray)
                self.dirty = False
