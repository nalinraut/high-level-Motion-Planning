import kdtree

infty = float('inf')
check_kdtree = False

class NearestNeighbors:
    def __init__(self,metric,method='bruteforce'):
        self.metric = metric
        self.method = method
        if self.method == 'kdtree':
            self.kdtree = kdtree.KDTree(self.metric)
            if check_kdtree:
                self.checker = NearestNeighbors(self.metric)
                print "Debugging: Double checking KD-tree with nearest neighbors"
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
        else:
            self.nodes.append((point,data))

    def set(self,points,datas=None):
        """Sets the point set to a list of points."""
        if datas==None:
            datas = [None]*len(points)
        if hasattr(self,'kdtree'):
            print "Resetting KD tree..."
            self.kdtree.set(points,datas)
            if check_kdtree: self.checker.set(points,datas)
        else:
            self.nodes = zip(points,datas)

    def nearest(self,pt,filter=None):
        """Nearest neighbor lookup, possibly with filter"""
        if self.method == 'kdtree':
            res = self.kdtree.nearest(pt,filter)
            if check_kdtree: 
                rescheck = self.checker.nearest(pt,filter)
                if res != rescheck:
                    print "KDTree nearest(",pt,") error",res,"should be",rescheck
                    print self.metric(res[0],pt)
                    print self.metric(rescheck[0],pt)
            return res
        else:
            #brute force
            res = None    
            dbest = infty
            for p,data in self.nodes:
                d = self.metric(p,pt)
                if d < dbest and not filter(p,data):
                    res = (p,data)
                    dbest = d
            return res
        
    def neighbors(self,pt,radius):
        """Range query, all points within pt.  Filtering can be done
        afterward by the user."""
        if self.method == 'kdtree':
            res = self.kdtree.neighbors(pt,radius)
            if check_kdtree: 
                rescheck = self.checker.neighbors(pt,radius)
                if res != rescheck:
                    print "KDTree neighbors(",pt,",",radius,") error",res,"should be",rescheck
            return res
        else:
            #brute force
            res = []
            for p,data in self.nodes:
                d = self.metric(p,pt)
                if d < radius:
                    res.append((p,data))
            return res
      

