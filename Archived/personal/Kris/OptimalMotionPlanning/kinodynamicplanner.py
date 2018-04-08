from sampler import *
from costspace import *
from edgechecker import EdgeChecker
from randomdict import RandomDict
import time
import metric
from nearestneighbors import *
import itertools

rrtNumControlSampleIters = 10
#rrtNumControlSampleIters = 1

estDefaultResolution = 10
estNumExtensionSamples = 10
#estNumExtensionSamples = 50


infty = float('inf')

def popdefault(mapping,item,default,warning=True):
    """Extracts item from the dict mapping.  If it doesnt exist, returns
    default."""
    try:
        res =  mapping[item]
        del mapping[item]
        return res
    except KeyError:
        if warning == True:
            print "Parameter",item,"not specified, using default",default
        elif warning != None and warning != False:
            print warning
        return default

class CountProfiler:
    """Collects an accumulated item."""
    def __init__(self):
        self.count = 0
    def __str__(self):
        return str(self.count)
    def set(self,value):
        self.count = value
    def add(self,value):
        self.count += value
    def __iadd__(self,value):
        self.add(value)
        return self

class ValueProfiler:
    """Collects a number-valued item, reporting the min, max, mean, and
    variance.  Can also be weighted."""
    def __init__(self):
        self.average = 0
        self.variance = 0
        self.minimum = None
        self.maximum = None
        self.count = 0
    def __str__(self):
        if self.count==0: return 'empty'
        return 'min %f, max %f, average %f, count %f'%(self.minimum,self.maximum,self.average,self.count)
    def reset(self):
        self.average = 0
        self.variance = 0
        self.minimum = None
        self.maximum = None
        self.count = 0        
    def add(self,value,weight=1):
        if self.count==0:
            self.minimum = self.maximum = value
        else:
            if value < self.minimum: self.minimum=value
            elif value > self.maximum: self.maximum=value
        oldEsq =  self.variance + self.average*self.average
        self.average = (self.count*self.average + weight*value)/(self.count+weight)
        newEsq = oldEsq + weight*value*value
        self.variance = newEsq - self.average*self.average
        self.count += weight
    def __iadd__(self,value):
        self.add(value)

class TimingProfiler(ValueProfiler):
    def __init__(self):
        ValueProfiler.__init__(self)
        self.tstart = None
    def begin(self):
        assert self.tstart == None, "Called begin() twice"
        self.tstart = time.time()
    def end(self):
        assert self.tstart != None, "Called end() without begin"
        t = time.time()-self.tstart
        self.tstart = None
        self.add(t)

class Profiler:
    def __init__(self):
        self.items = {}
    def stopwatch(self,item):
        try:
            return self.items[item]
        except KeyError:
            self.items[item] = TimingProfiler()
            return self.items[item]
    def count(self,item):
        try:
            return self.items[item]
        except KeyError:
            self.items[item] = CountProfiler()
            return self.items[item]
    def value(self,item):
        try:
            return self.items[item]
        except KeyError:
            self.items[item] = ValueProfiler()
            return self.items[item]
    def descend(self,item):
        try:
            return self.items[item]
        except KeyError:
            self.items[item] = Profiler()
            return self.items[item]
    def pretty_print(self,indent=0):
        for (k,v) in self.items.iteritems():
            print ' '*indent+str(k),":",
            if isinstance(v,Profiler):
                print
                v.pretty_print(indent+1)
            else:
                print str(v)

def cumsum(ls):
    """Returns a list containing the cumulative sums at every element of
    ls.
    
    i.e., cumsum([1,2,3]) = [1,3,6]."""
    
    acc = 0
    r = [0 for v in ls]
    for i,v in enumerate(ls):
        acc += v
	r[i] = acc
    return r

def sample_weighted(weights, vals=None, eps=1.0e-4):
    """Selects a value from vals with probability proportional to the
    corresponding value in weights.

    If vals == None, returns the index that would have been picked
    """
	
    weightSum = sum(weights)
    if weightSum == 0:
        if vals==None:
            return random.randint(0,len(weights)-1)
        return random.choice(vals)
    r = random.uniform(0.0,weightSum)
    if vals==None:
        for i,w in enumerate(weights):
            if r <= w:
                return i
            r -= w
        return len(weights)-1
    else:
        for v,w in zip(vals,weights):
            if r <= w:
                return v
            r -= w
        return vals[-1]


class ControlSelector:
    """A function that produces a control u that tries to steer a
    state x to another state xdesired."""
    def select(self,x,xdesired):
        raise NotImplementedError()

class RandomControlSelector(ControlSelector):
    """A ControlSelector that randomly samples numSamples controls
    and finds the one that is closest to the destination, according
    to a given metric."""
    def __init__(self,controlSpace,metric,numSamples):
        self.controlSpace = controlSpace
        self.metric = metric
        self.numSamples = numSamples
    def select(self,x,xdesired):
        ubest = None
	#do we want to eliminate extensions that do not improve the metric
	#from the starting configuration?  experiments suggest no 5/6/2015
        #dbest = self.metric(x,xdesired)
	dbest = infty
        t0 = time.time()
	U = self.controlSpace.controlSet(x)
        for iters in xrange(self.numSamples):
            u = U.sample()
            if U.contains(u):
                xnext = self.controlSpace.nextState(x,u)
                d = self.metric(xnext,xdesired)
                if d < dbest:
                    dbest = d
                    ubest = u
        return ubest


class KinematicControlSelector(ControlSelector):
    """A steering function for a kinematic control space"""
    def __init__(self,controlSpace,maxDistance=float('inf')):
        assert isinstance(controlSpace,ControlSpaceAdaptor)
        self.controlSpace = controlSpace
        self.cspace = controlSpace.configurationSpace()
        self.maxDistance = maxDistance
    def select(self,x,xdesired):
        d = self.cspace.distance(x,xdesired)
        if d > self.maxDistance:
            e = self.cspace.interpolator(x,xdesired)
            return e.eval(self.maxDistance/d)
        return xdesired

class KinematicCostControlSelector:
    """A steering function for a CostControlSpace on top of a kinematic
    control space"""
    def __init__(self,controlSpace,maxDistance=float('inf')):
        assert isinstance(controlSpace,CostControlSpace)
        assert isinstance(controlSpace.baseSpace,ControlSpaceAdaptor)
        self.controlSpace = controlSpace
        self.cspace = controlSpace.baseSpace.configurationSpace()
        self.maxDistance = maxDistance
    def select(self,x,xdesired):
        xb = x[:-1]
        xdb = xdesired[:-1]
        d = self.cspace.distance(xb,xdb)
        if d > self.maxDistance:
            e = self.cspace.interpolator(xb,xdb)
            return e.eval(self.maxDistance/d)
        return xdb

class Node:
    """A node of a kinodynamic tree"""
    def __init__(self,x,uparent=None,eparent=None):
        self.x = x
        self.uparent = uparent
        self.eparent = eparent
        self.parent = None
        self.children = []
    def destroy(self):
        """Call this to free up the memory stored by this sub tree."""
        if self.parent:
            self.parent.children.remove(self)
            self.parent = None
        for c in self.children:
            c.parent = None
            c.destroy()
        self.children = []
    def unparent(self):
        """Detatches this node from its parent"""
        if self.parent:
            self.parent.children.remove(self)
            self.parent = None
    def addChild(self,c):
        """Adds a child"""
        assert c.parent == None
        c.parent = self
        self.children.append(c)
    def setParent(self,p,uparent=None,eparent=None):
        """Sets the parent of this node to p"""
        if self.parent != None:
            self.parent.children.remove(self)
        self.parent = p
        self.uparent = uparent
        self.eparent = eparent
        p.children.append(self)
    def traverse(self,visitor):
        """Given a callback visitor(n), traverses the tree in DFS fashion.
	The visitor function can return False to prune traversal below a
	given node."""
	if visitor(self)==False:
	    return
        for c in self.children:
	    c.traverse(visitor)
	return

class TreePlanner:
    """A base class for kinodynamic tree planners"""
    def __init__(self):
        self.root = None
        self.nodes = []
    def destroy(self):
        """To be nice to the GC, call this to free up memory after you're
        done planning"""
        if self.root:
            self.root.destroy()
            self.root = None
    def setRoot(self,x):
        """Sets the root of the tree"""
        if self.root != None:
           self.root.destroy()
        self.root = Node(x)
        self.nodes = [self.root]
    def addEdge(self,n,u,edge):
        """Adds an edge to the tree"""
        nnew = Node(edge.end(),u,edge)
        n.addChild(nnew)
        self.nodes.append(nnew)
        return nnew
    def getPath(self,n):
        """Returns a state-control pair ([x0,...,xn],[u1,...,un])"""
        pathToStart = []
        while n != None:
            pathToStart.append(n)
            n = n.parent
        pathToGoal = pathToStart
        pathToGoal.reverse()
        return ([n.x for n in pathToGoal],[n.uparent for n in pathToGoal[1:]])
    def getRoadmap(self):
        """Returns a graph (V,E) where V contains states and E contains
        triples (i,j,u) where control u connnects V[i] to V[j]"""
        V = []
        E = []
        n = self.root
        if n == None:
            return (V,E)
        V.append(n.x)
        q = [(n,0)]
        while len(q) > 0:
            n,i = q.pop()
            for c in n.children:
                j = len(V)
                E.append((i,j,c.uparent))
                V.append(c.x)
                q.append((c,j))
        return (V,E)

class RRT(TreePlanner):
    """The Rapidly-exploring Random Tree kinodynamic planner.

    Stores a tree of Nodes.  Expands the tree at random using the RRT strategy
    with a goal bias of probability pChooseGoal.

    Default controlSelector uses a RandomControlSelector with 10 samples.

    If you turn dynamicDomain = True, then the Adaptive Dynamic-Domain RRT
    planner of Jaillet et al 2005 is used.
    """
    def __init__(self,controlSpace,metric,edgeChecker,
                 **params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
        TreePlanner.__init__(self)
	if not isinstance(controlSpace,ControlSpace):
		print "Warning, controlSpace is not a ControlSpace"
	if not isinstance(edgeChecker,EdgeChecker):
		print "Warning, edgeChecker is not an EdgeChecker"
        self.cspace = controlSpace.configurationSpace()
	if not isinstance(self.cspace,ConfigurationSpace):
		print "Warning, controlSpace.configurationSpace() is not a ConfigurationSpace"
        self.controlSpace = controlSpace
        self.metric = metric
        self.edgeChecker = edgeChecker
        self.goal = None
        self.goalSampler = None
        self.pChooseGoal = popdefault(params,'pChooseGoal',0.1)
        self.goalNodes = []
        self.configurationSampler = Sampler(self.cspace)
        numControlSamples = popdefault(params,'numControlSamples',rrtNumControlSampleIters)   
        self.controlSelector = RandomControlSelector(controlSpace,self.metric,numControlSamples)
	self.successBiasing = False
        self.dynamicDomain = popdefault(params,'dynamicDomain',False)
        if self.dynamicDomain:
            self.dynamicDomainInitialRadius = popdefault(params,'dynamicDomainInitialRadius',0.1)
            self.dynamicDomainInitialRadius = popdefault(params,'dynamicDomainGrowithParameter',0.5)
        nnmethod = popdefault(params,'nearestNeighborMethod','kdtree')
        self.nearestNeighbors = NearestNeighbors(self.metric,nnmethod)

        self.pruner = None
        self.stats = Profiler()
        self.numIters = self.stats.count('numIters')
        self.nextSampleList = []
        if len(params) != 0:
            print "Warning, unused params",params
        
    def destroy(self):
        TreePlanner.destroy(self)
        self.goalNodes = []
    def reset(self):
        """Re-initializes the RRT to the same start / goal, clears the planning
	tree."""
	x0 = self.root.x
	goal = self.goal
	self.destroy()
	self.setBoundaryConditions(x0,goal)
	self.numIters.set(0)
	
    def setBoundaryConditions(self,x0,goal):
        """Initializes the tree from a start state x0 and a goal
        ConfigurationSubset.
        
        goal can be set to None to just explore.
        """
        self.setRoot(x0)
	self.root.numExpansionsAttempted = 0
	self.root.numExpansionsSuccessful = 0
        self.goal = goal
        if goal != None:
            if isinstance(goal,(list,tuple)):
                self.goal = SingletonSubset(self.cspace,goal)
            self.goalSampler = SubsetSampler(self.cspace,self.goal)
        self.nearestNeighbors.reset()
        self.nearestNeighbors.add(x0,self.root)
    def setConfigurationSampler(self,sampler):
        self.configurationSampler = sampler
    def setControlSelector(self,selector):
        self.controlSelector = selector
    def planMore(self,iters):
        for n in xrange(iters):
            self.numIters += 1
            n = self.expand()
            if n != None and self.goal != None:
                if self.goal.contains(n.x):
                    self.goalNodes.append(n)
                    return True
        return False
    def expand(self):
        """Expands the tree via the RRT technique.  Returns the new node
        or None otherwise."""
        if len(self.nextSampleList)==0:
            if self.goalSampler and random.uniform(0.0,1.0) < self.pChooseGoal:
                xrand = self.goalSampler.sample()
            else:
                xrand = self.configurationSampler.sample()
            if not self.cspace.feasible(xrand):
                return None
        else:
            xrand = self.nextSampleList.pop(0)
        self.stats.stopwatch('pickNode').begin()
        nnear = self.pickNode(xrand)
        self.stats.stopwatch('pickNode').end()
        if nnear == None:
            self.stats.count('pickNodeFailure').add(1)
            return None
        self.stats.stopwatch('selectControl').begin()
	nnear.numExpansionsAttempted += 1
        u = self.controlSelector.select(nnear.x,xrand)
        self.stats.stopwatch('selectControl').end()
        #print "Expanding",nnear.x,"toward",xrand,"selected control",u
        if u == None:
            #do we want to adjust the dynamic domain?
            if self.dynamicDomain:
                if hasattr(nnear,'ddRadius'):
                    nnear.ddRadius *= (1.0-self.dynamicDomainGrowthParameter)
                else:
                    nnear.ddRadius = self.dynamicDomainInitialRadius
            self.stats.count('controlSelectionFailure').add(1)
            return None
        self.stats.stopwatch('edgeCheck').begin()
        edge = self.controlSpace.interpolator(nnear.x,u)
        if not self.edgeChecker.feasible(edge):
            self.stats.stopwatch('edgeCheck').end()
            if self.dynamicDomain:
                if hasattr(nnear,'ddRadius'):
                    nnear.ddRadius *= (1.0-self.dynamicDomainGrowthParameter)
                else:
                    nnear.ddRadius = self.dynamicDomainInitialRadius
            self.stats.count('infeasibleEdges').add(1)
            return None
        self.stats.stopwatch('edgeCheck').end()
        #feasible edge, add it
        if self.dynamicDomain:
            if hasattr(nnear,'ddRadius'):
                nnear.ddRadius *= (1.0+self.dynamicDomainGrowthParameter)
        nnew = self.addEdge(nnear,u,edge)
        if self.prune(nnew):
            nnew.destroy()
            self.nodes.pop()
            return None
        self.nearestNeighbors.add(nnew.x,nnew)
        nnear.numExpansionsSuccessful += 1
        nnew.numExpansionsAttempted = 0
	nnew.numExpansionsSuccessful = 0
        return nnew
    def prune(self,node):
        """Overload this to add tree pruning.  Return True to prune a node"""
        if self.pruner:
            return self.pruner(node)
        return False
    def pruneTree(self):
        """Prunes all branches of the tree that should be pruned according to
	the prune() function, updates all internal data structures."""
        self.stats.stopwatch('pruneTree').begin()
	def pruneIt(n):
            newchildren = []
            delchildren = []
            for c in n.children:
                if self.prune(c) or not self.cspace.feasible(c.x):
                    delchildren.append(c)
                else:
                    newchildren.append(c)
	    for c in delchildren:
                c.parent = None
                c.destroy()
            n.children = newchildren
            return True
	newNodes = []
	def addNodes(n):
	    newNodes.append(n)
	assert not self.prune(self.root),"Root node is asked to be pruned... can't handle this case"
	self.root.traverse(pruneIt)
	self.root.traverse(addNodes)
	self.nodes = newNodes
        self.nearestNeighbors.set([n.x for n in self.nodes],self.nodes)
        self.stats.stopwatch('pruneTree').end()
        
    def pickNode(self,xrand):
        """Picks a node closest to xrand.  If dynamicDomain is True,
        uses the radius associated with the node"""
        #setup nearest neighbor filters
        filters = [lambda pt,n: self.prune(n)]
        if self.dynamicDomain:
            filters.append(lambda pt,n:hasattr(n,'ddRadius') and self.metric(n.x,xrand) >= n.ddRadius)
        if self.successBiasing:
            filters.append(lambda pt,n: (random.random() > float(n.numExpansionsSuccessful+1) / float(n.numExpansionsAttempted+1)))
        #do the lookup
        res = self.nearestNeighbors.nearest(xrand,lambda pt,n:any(f(pt,n) for f in filters))
        if res == None: return None
        n = res[1]
        return n
    
    def getPath(self,n=None):
        if n == None:
            if len(self.goalNodes)==0:
                return None
            return TreePlanner.getPath(self,self.goalNodes[0])
        return TreePlanner.getPath(self,n)

class EST(TreePlanner):
    """The Expansive Space Tree kinodynamic planner.

    Stores a tree of Nodes.  Expands the tree at random using the inverse
    density weighting strategy.
    """
    def __init__(self,controlSpace,edgeChecker,
                 **params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
        TreePlanner.__init__(self)
        self.cspace = controlSpace.configurationSpace()
        self.controlSpace = controlSpace
        self.edgeChecker = edgeChecker
        self.goal = None
        self.goalNodes = []
        self.stats = Profiler()
        self.numIters = self.stats.count('numIters')
        self.pruner = None
	self.radius = popdefault(params,'densityEstimationRadius',0.1)
        if len(params) != 0:
            print "Warning, unused params",params
    def destroy(self):
        TreePlanner.destroy(self)
        self.goalNodes = []
    def setBoundaryConditions(self,x0,goal):
        """Initializes the tree from a start state x0 and a goal
        ConfigurationSubset.
        
        goal can be set to None to just explore.
        """
        if self.root != None:
            self.destroy()
        self.setRoot(x0)
        self.onAddNode(self.root)
        self.goal = goal
        if goal != None:
            if isinstance(goal,(list,tuple)):
                self.goal = SingletonSubset(self.cspace,goal)
    def reset(self):
        self.setBoundaryConditions(self.root.x,self.goal)
        self.numIters.set(0)
        self.goalNodes = []
        if hasattr(self,'extensionCache'):
            del self.extensionCache
    def planMore(self,iters):
        for n in xrange(iters):
            self.numIters += 1
            n = self.expand()
            if n != None and self.goal != None:
                if self.goal.contains(n.x):
                    #print "new goal node",n.x
                    self.goalNodes.append(n)
                    return True
        return False
    def expand(self):
        """Expands the tree via the EST technique.  Returns the new node
        or None otherwise."""
        #control sampling method
        global estNumExtensionSamples
        numNodeSamples = estNumExtensionSamples
        numControlSamplesPerNode = 1
        #numNodeSamples = 1
        #Temp: test some probability of rejection?
        #extensions = [None]
        #weights = [1.0]
        #Tests seem to indicate that cached extensions aren't very useful
        #at least on simple problems
        cachedExtensions = True
        if not cachedExtensions:
            weights = []
            extensions = []
        else:
            assert numControlSamplesPerNode == 1,"Cached extensions require only 1 control samples per node, at the moment"
            if not hasattr(self,'extensionCache'):
                weights = []
                extensions = []
                self.extensionCache = (weights,extensions)
            else:
                (weights,extensions) = self.extensionCache
            #clear cache occasionally
            if self.numIters.count % 100000 == 0:
                weights = []
                extensions = []
                self.extensionCache = (weights,extensions)
            #re-estimate density of existing extensions
            for i in xrange(len(weights)):
                edge = extensions[i][2]
                de = self.density(edge.end())
                weights[i] = 1.0/(1.0+de**2)
        for n in xrange(numNodeSamples - len(weights)):
            nnear = self.pickNode()
            if nnear == None:
                return None
	    #d0 = self.density(nnear.x)
            U = self.controlSpace.controlSet(nnear.x)
            for i in xrange(numControlSamplesPerNode):
                u = U.sample()
                if U.contains(u):
                    edge = self.controlSpace.interpolator(nnear.x,u)
                    #if self.prune(edge.end()):
                    #    continue
                    if not self.cspace.feasible(edge.end()):
                        continue
                    if cachedExtensions:
                        if not self.edgeChecker.feasible(edge):
                            continue
                    de = self.density(edge.end())
                    extensions.append((nnear,u,edge))
                    # pick with probability inversely proportional to density
		    #weights.append(1.0/(1.0+de))
		    # pick with probability inversely proportional to density squared?
                    weights.append(1.0/(1.0+de**2))
        if len(extensions)==0:
            #failed extension
	    #TODO: penalize this node
	    return None
        i = sample_weighted(weights)
        if extensions[i]==None:
            #this gives some probability to rejecting extensions
	    #in highly dense regions
	    return None
        n,u,edge = extensions[i]
        
        if cachedExtensions:
            #delete from cache
            weights[i] = weights[-1]
            extensions[i] = extensions[-1]
            weights.pop(-1)
            extensions.pop(-1)
        if not cachedExtensions:
            #cached extensions are pre-checked for feasibility
            if not self.edgeChecker.feasible(edge):
	        #TODO: penalize this node
                return None

        #feasible edge, add it
        nnew = self.addEdge(n,u,edge)
        if self.prune(nnew):
            nnew.destroy()
            self.nodes.pop()
            return None
        self.onAddNode(nnew)
        return nnew
    def onAddNode(self,n):
        """Adds nodes' density contribution to nearby nodes"""
	d = 1.0
	for nother in self.nodes:
	    if nother is n: continue
            dist = self.cspace.distance(n.x,nother.x)
            if dist > 4.0*self.radius: continue
	    d += math.exp(-(dist/self.radius)**2)
	    nother.density += d
	n.density = d
    def density(self,x):
        d = 0.0
	for n in self.nodes:
	    d += math.exp(-(self.cspace.distance(n.x,x)/self.radius)**2)
	return d
    def pickNode(self):
        """Picks a node according to inverse density weighting strategy"""
	return sample_weighted([1.0/n.density for n in self.nodes],self.nodes)
    def prune(self,node):
        """Overload this to add tree pruning.  Return True to prune a node"""
        if self.pruner:
            return self.pruner(node)
        return False
    def pruneTree(self):
        """Prunes all branches of the tree that should be pruned according to
	the prune() function, updates all internal data structures."""
	def pruneIt(n):
            newchildren = []
            delchildren = []
            for c in n.children:
                if self.prune(c) or not self.cspace.feasible(c.x):
                    delchildren.append(c)
                else:
                    newchildren.append(c)
	    for c in delchildren:
                c.parent = None
                c.destroy()
            n.children = newchildren
            return True
	newNodes = []
	def addNodes(n):
	    self.onAddNode(n)
	    newNodes.append(n)
	assert not self.prune(self.root),"Root node is asked to be pruned... can't handle this case"
	self.root.traverse(pruneIt)
	self.root.traverse(addNodes) 
	self.nodes = newNodes
    def getPath(self,n=None):
        """Returns a path to the node n if specified, or to the first goal
	node by default."""
        if n == None:
            if len(self.goalNodes)==0:
                return None
            return TreePlanner.getPath(self,self.goalNodes[0])
        return TreePlanner.getPath(self,n)

class ESTWithProjections(EST):
    """The EST but with a faster density estimator and data structure
    update.
    """
    def __init__(self,controlSpace,edgeChecker,
                 **params):
        self.projectionBases = []
        self.projectionHashes = []
        self.projectionResolution = popdefault(params,'projectionResolution',estDefaultResolution)
        EST.__init__(self,controlSpace,edgeChecker,
                     **params)
    def destroy(self):
        EST.destroy(self)
        self.projectionBases = []
        self.projectionHashes = []
    def reset(self):
        self.projectionBases = []
        self.projectionHashes = []
        EST.reset(self)
    def generateDefaultBases(self,indices):
        #generate exhaustive bases for the indicated space dimensions
        #first, determine a scale factor
        try:
            bmin,bmax = self.controlSpace.configurationSpace().bounds()
            scale = [1.0/(b-a) for (a,b) in zip(bmin,bmax)]
            #if isinstance(self.controlSpace,CostControlSpace):
            #    if scale[-1] != 0:
            #        scale[-1] = 1
            print "EST projection hash scale",scale
        except Exception:
            scale = [1]*self.controlSpace.configurationSpace().dimension()
            print "EST projection hash scale",scale
        #now enumerate all size-3 subsets of the indices
        d = min(len(indices),3)
        self.projectionBases = []
        self.projectionHashes = []
        for element in itertools.combinations(indices,d):
            self.projectionBases.append([])
            for i in element:
                basis = {i:self.projectionResolution*scale[i]}
                self.projectionBases[-1].append(basis)
            self.projectionHashes.append(RandomDict())
        print "EST using",len(self.projectionBases),"projection bases"
        if self.root != None:
            #need to re-add the elements of the tree
            def recursive_add(node):
                self.onAddNode(node)
                for c in node.children:
                    recursive_add(c)
            recursive_add(self.root)
    def onAddNode(self,n):
        """Adds nodes to hash functions"""
        if len(self.projectionBases)==0:
            self.generateDefaultBases(range(len(n.x)))
        for basis,bhash in zip(self.projectionBases,self.projectionHashes):
            #sparse dot products
            dp = [0.0]*len(basis)
            for i,basisvector in enumerate(basis):
                for (k,v) in basisvector.iteritems():
                    #experimental
                    #if isinstance(self.controlSpace,CostControlSpace) and k==len(x)-1:
                    #    #cost log transform
                    #    dp[i] += math.log(n.x[k]*v+1.0)*3.0
                    #else:
                    dp[i] += n.x[k]*v
            index = tuple([int(v) for v in dp])
            bhash.setdefault(index,[]).append(n)
        return
    def density(self,x):
        c = 0
        for basis,bhash in zip(self.projectionBases,self.projectionHashes):
            #sparse dot products
            dp = [0.0]*len(basis)
            for i,basisvector in enumerate(basis):
                for (k,v) in basisvector.iteritems():
                    #experimental
                    #if isinstance(self.controlSpace,CostControlSpace) and k==len(x)-1:
                    #    #cost log transform
                    #    dp[i] += math.log(x[k]*v+1.0)*3.0
                    #else:
                    dp[i] += x[k]*v
            index = tuple([int(v) for v in dp])
            c += len(bhash.get(index,[]))
        return c
    def pruneTree(self):
        self.projectionBases = []
	self.projectionHashes = []
	EST.pruneTree(self)
    def pickNode(self):
        """Picks a node according to inverse density weighting strategy"""
        index = random.randint(0,len(self.projectionBases)-1)
        basis = self.projectionBases[index]
        bhash = self.projectionHashes[index]
        key = bhash.random_key()
	#key = bhash.random_key(weight=lambda k,x:1.0/len(x))
        #print "Basis",basis,"select",key,"random from points",len(bhash[key])
        res = random.choice(bhash[key])
        if self.prune(res): return None
        #print "Selected",res.x
        return res


class CostEdgeChecker(EdgeChecker):
    def __init__(self,edgeChecker):
        self.edgeChecker = edgeChecker
        self.costMax = None
    def feasible(self,interpolator):
        return self.edgeChecker.feasible(interpolator.components[0]) and (self.costMax == None or interpolator.components[1].end()[0] <= self.costMax)

class CostMetric:
    def __init__(self,metric,costWeight=1.0):
        self.metric = metric
        self.costWeight = costWeight
        self.costMax = None
    def __call__(self,a,b):
        #a is the existing node in the tree, b is the new sample
        d = self.metric(a[:-1],b[:-1])
        if self.costWeight != None:
            if (self.costMax != None) and (a[-1] > self.costMax or b[-1] > self.costMax):
                return infty
            d += max(a[-1]-b[-1],0.0)*self.costWeight
            #d += abs(a[-1]-b[-1])*self.costWeight
        return d

class CostSpaceSampler(Sampler):
    def __init__(self,baseSampler,costMax):
        self.baseSampler = baseSampler
        self.costMax = costMax
    def sample(self):
        xb = self.baseSampler.sample()
        cmax = 0.0 if self.costMax == None else self.costMax
        return xb+[random.uniform(0.0,cmax)]


class CostGoal(ConfigurationSubset):
    def __init__(self,baseGoal,objective,costMax):
        self.baseGoal = baseGoal
        self.objective = objective
        self.costMax = costMax
    def contains(self,x):
        if not self.baseGoal.contains(x[:-1]):
            return False
        if self.costMax==None:
            return True
        return x[-1]+self.objective.terminal(x[:-1])<=self.costMax
    def sample(self):
        xb = self.baseGoal.sample()
        if xb == None: return None
        cmax = 0.0 if self.costMax == None else self.costMax-self.objective.terminal(xb)
        if cmax < 0.0: cmax = 0.0
        return xb+[random.uniform(0,cmax)]
    def project(self,x):
        return self.baseGoal.project(x[:-1])+[min(self.costMax,x[-1])]

class HeuristicCostSpaceSampler(Sampler):
    """Given optionally a cost-to-come heuristic and a cost-to-goal heuristic,
    samples only from the range of cost values that could yield a feasible
    path.
    """
    def __init__(self,baseSampler,heuristicCostToCome,heuristicCostToGo,costMax):
        self.baseSampler = baseSampler
        self.heuristicCostToCome = (heuristicCostToCome if heuristicCostToCome!=None else lambda x:0.0)
        self.heuristicCostToGo = (heuristicCostToGo if heuristicCostToGo!=None else lambda x:0.0)
        self.costMax = costMax
    def sample(self):
        count = 0
        while True:
            xb = self.baseSampler.sample()
            if self.costMax==None:
                return xb + [0.0]
            #putting a lower bound seems to be problematic for poorly scaled
            #metrics
            #cmin = self.heuristicCostToCome(xb)
            cmin = 0.0
            cmax = self.costMax - self.heuristicCostToGo(xb)
            #cmax = self.costMax
            if cmin <= cmax:
                return xb+[random.uniform(cmin,cmax)]
            count += 1
            if count % 1000 == 0:
                print "Heuristic sampler: spinning over 1000 iterations?"

class HeuristicCostSpacePruner:
    """Prunes a node if cost-from-start + heuristicCostToGo > costMax"""
    def __init__(self,heuristicCostToGo,costMax):
        self.heuristicCostToGo = (heuristicCostToGo if heuristicCostToGo!=None else lambda x:0.0)
        self.costMax = costMax
    def __call__(self,x):
        if isinstance(x,Node):
	    return self(x.x)
        if self.costMax == None:
            return False
        cost = x[-1]
        xbase = x[:-1]
        return cost+self.heuristicCostToGo(xbase) > self.costMax

class CostSpaceRRT:
    """The cost-space Rapidly-exploring Random Tree planner.
    """
    def __init__(self,controlSpace,objective,metric,edgeChecker,
                 **params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
        self.objective = objective
        self.baseSpace = controlSpace.configurationSpace()
        self.baseControlSpace = controlSpace
        self.costSpace = CostControlSpace(controlSpace,objective)
        self.baseMetric = metric
        self.metric = CostMetric(self.baseMetric,0)
        self.edgeChecker = CostEdgeChecker(edgeChecker)
        #self.costWeight = popdefault(params,'costWeight','adaptive')
        self.costWeight = popdefault(params,'costWeight',1)
        self.rrt = RRT(self.costSpace,self.metric,self.edgeChecker,**params)
        self.costSpaceSampler = CostSpaceSampler(Sampler(self.baseSpace),None)
        self.rrt.setConfigurationSampler(self.costSpaceSampler)
        self.bestPath = None
        self.bestPathCost = None
        self.lastPruneCost = None
        self.stats = Profiler()
        self.stats.items['rrt'] = self.rrt.stats
        self.numIters = self.stats.count('numIters')
                
    def destroy(self):
        """To be nice to the GC, call this to free up memory after you're
        done planning"""
        self.rrt.destroy()
    def setBoundaryConditions(self,x0,goal):
        """Initializes the tree from a start state x0 and a goal
        ConfigurationSubset.
        
        goal can be set to None to just explore.
        """
        if isinstance(goal,(list,tuple)):
            goal = SingletonSubset(self.baseSpace,goal)
        self.baseStart = x0
        self.baseGoal = goal
        self.costGoal = CostGoal(goal,self.objective,self.bestPathCost)
        self.rrt.setBoundaryConditions(self.costSpace.makeState(x0,0.0),self.costGoal)
    def setHeuristic(self,heuristicCostToCome=None,heuristicCostToGo=None):
        self.costSpaceSampler = HeuristicCostSpaceSampler(Sampler(self.baseSpace),heuristicCostToCome,heuristicCostToGo,self.bestPathCost)
        self.rrt.setConfigurationSampler(self.costSpaceSampler)
        self.rrt.pruner = HeuristicCostSpacePruner(heuristicCostToGo,self.bestPathCost)
    def setConfigurationSampler(self,sampler):
        self.rrt.setConfigurationSampler(CostSpaceSampler(sampler,self.bestPathCost))
    def setControlSelector(self,selector):
        self.rrt.setControlSelector(selector)
    def reset(self):
        """Resets all planning effort"""
        self.rrt.reset()
	self.bestPath = None
	self.bestPathCost = None
        self.lastPruneCost = None
	self.updateBestCost()
    def updateBestCost(self):
        self.costGoal.costMax = self.bestPathCost
        self.costSpaceSampler.costMax = self.bestPathCost
        self.metric.costMax = self.bestPathCost
        self.edgeChecker.costMax = self.bestPathCost
        if self.rrt.pruner:
            self.rrt.pruner.costMax = self.bestPathCost
        self.costSpace.setCostMax(self.bestPathCost)
        if self.bestPathCost != None:
            if self.costWeight == 'adaptive':
                n = self.rrt.goalNodes[-1]
                dgoal = self.baseMetric(n.x[:-1],self.baseStart)
                self.metric.costWeight = self.bestPathCost / (dgoal)
                print "Setting RRT distance weight on cost to",self.metric.costWeight
            else:
                self.metric.costWeight = self.costWeight
	else:
	    self.metric.costWeight = 0.0
    def planMore(self,iters):
        didreset = False
        foundNewPath = False
        for n in xrange(iters):
            self.numIters.add(1)
            if self.rrt.planMore(1):
                #check to see if RRT has a better path
                n = self.rrt.goalNodes[-1]
                c = n.x[-1] + self.objective.terminal(n.x[:-1])
                if self.bestPathCost == None or c < self.bestPathCost:
                    print "Improved best path cost from",self.bestPathCost,"to",c
                    self.bestPathCost = c
                    self.bestPath = self.rrt.getPath(n)
                    self.updateBestCost()
                    foundNewPath = True
		    #resets may help performance... but experiments suggest
                    #that there's little effect (5/6/2015)
                    #self.rrt.reset()
                    #didreset = True
        if foundNewPath and not didreset:
            #print "Trying pruning..."
            self.lastPruneCost = self.bestPathCost
            prunecount = 0
            for n in self.rrt.nodes:
                if n.x[-1] > self.bestPathCost or self.rrt.prune(n):
                    prunecount += 1
            print "Can prune",prunecount,"of",len(self.rrt.nodes),"nodes"
	    if prunecount > len(self.rrt.nodes)/5:
            #if prunecount > 0:
	        oldpruner = self.rrt.pruner
		if self.rrt.pruner == None:
		    self.rrt.pruner = lambda n:n.x[-1] >= self.bestPathCost
	        self.rrt.pruneTree()
		self.rrt.pruner = oldpruner
                print "   pruned down to",len(self.rrt.nodes),"nodes"
                
        return self.bestPathCost
    def getPathCost(self):
        return self.bestPathCost
    def getPath(self):
        """Returns ([x0,...,xn],[u1,...,un]) for the base space."""
        if self.bestPath == None:
            return None
        x,u = self.bestPath
        return ([xi[:-1] for xi in x],u)
    def getRoadmap(self):
        """Returns a roadmap for the base space"""
        (V,E) = self.rrt.getRoadmap()
        return ([x[:-1] for x in V],E)


class CostSpaceEST:
    """The cost-space Expansive Space Tree planner.
    """
    def __init__(self,controlSpace,objective,edgeChecker,**params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
        self.objective = objective
        self.baseSpace = controlSpace.configurationSpace()
        self.baseControlSpace = controlSpace
        self.costSpace = CostControlSpace(controlSpace,objective)
        self.edgeChecker = CostEdgeChecker(edgeChecker)
        self.est = ESTWithProjections(self.costSpace,self.edgeChecker,**params)
        self.bestPath = None
        self.bestPathCost = None
        self.stats = Profiler()
        self.stats.items['est'] = self.est.stats
        self.numIters = self.stats.count('numIters')
        self.lastPruneCost = None
    def destroy(self):
        """To be nice to the GC, call this to free up memory after you're
        done planning"""
        self.est.destroy()
    def setBoundaryConditions(self,x0,goal):
        """Initializes the tree from a start state x0 and a goal
        ConfigurationSubset.
        
        goal can be set to None to just explore.
        """
        if isinstance(goal,(list,tuple)):
            goal = SingletonSubset(self.baseSpace,goal)
        self.baseStart = x0
        self.baseGoal = goal
        self.costGoal = CostGoal(goal,self.objective,self.bestPathCost)
        self.est.generateDefaultBases(range(len(x0)))
        self.est.setBoundaryConditions(self.costSpace.makeState(x0,0.0),self.costGoal)

    def setHeuristic(self,heuristicCostToCome=None,heuristicCostToGo=None):
        self.est.pruner = HeuristicCostSpacePruner(heuristicCostToGo,self.bestPathCost)
    def reset(self):
        print "CostSpaceEST reset"
        self.est.reset()
        print "Regenerating projection bases without cost dimension."
        self.est.generateDefaultBases(range(len(self.baseStart)))
        self.numIters.set(0)
        self.bestPath = None
        self.bestPathCost = None
        self.lastPruneCost = None
        self.updateBestCost()
    def updateBestCost(self):
        self.costSpace.setCostMax(self.bestPathCost)
        if self.edgeChecker.costMax == None and self.bestPathCost != None:
            print "Regenerating projection bases to include cost dimension."
            self.est.generateDefaultBases(range(len(self.baseStart)+1))
        self.costGoal.costMax = self.bestPathCost
        self.edgeChecker.costMax = self.bestPathCost
        if self.est.pruner:
            self.est.pruner.costMax = self.bestPathCost
    def planMore(self,iters):
        foundNewPath = False
        for n in xrange(iters):
            self.numIters.add(1)
            if self.est.planMore(1):
                #check to see if RRT has a better path
                n = self.est.goalNodes[-1]
                c = n.x[-1] + self.objective.terminal(n.x[:-1])
                if self.bestPathCost == None or c < self.bestPathCost:
                    print "Improved best path cost from",self.bestPathCost,"to",c
                    foundNewPath = True
                    self.bestPathCost = c
                    self.bestPath = self.est.getPath(n)
                    self.updateBestCost()
		    #Resets seem to really hurt performance
                    #self.est.reset()

        if foundNewPath:
            #print "Trying pruning..."
            self.lastPruneCost = self.bestPathCost
            prunecount = 0
            for n in self.est.nodes:
                if n.x[-1] > self.bestPathCost or self.est.prune(n):
                    prunecount += 1
            #print len(self.est.nodes),"nodes, can prune",prunecount
	    #redo tree when can prune 20% of nodes
	    if prunecount >  len(self.est.nodes)/5:
	        oldpruner = self.est.pruner
		if self.est.pruner == None:
		    self.est.pruner = lambda n:n.x[-1] >= self.bestPathCost
	        self.est.pruneTree()
		self.est.pruner = oldpruner
		#print "   pruned down to",len(self.est.nodes),"nodes"
	#else:
	    #print len(self.est.nodes),"nodes"
        return self.bestPathCost
    def getPathCost(self):
        return self.bestPathCost
    def getPath(self):
        """Returns ([x0,...,xn],[u1,...,un]) for the base space."""
        if self.bestPath == None:
            return None
        x,u = self.bestPath
        return ([xi[:-1] for xi in x],u)
    def getRoadmap(self):
        """Returns a roadmap for the base space"""
        (V,E) = self.est.getRoadmap()
	pruner = lambda x:False
	if self.est.pruner != None: pruner = self.est.pruner
        return ([x[:-1] for x in V],[e for e in E if (not pruner(V[e[0]]) and not pruner(V[e[1]]))])


class RepeatedRRT(RRT):
    """The repeated Rapidly-exploring Random Tree planner.
    """
    def __init__(self,controlSpace,objective,metric,edgeChecker,
                 **params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
	RRT.__init__(self,controlSpace,metric,edgeChecker,**params)
        self.objective = objective
	self.bestPath = None
	self.bestPathCost = None
	self.doprune = False
    def reset(self):
        """Resets all planning effort"""
        RRT.reset(self)
	self.bestPath = None
	self.bestPathCost = None
    def setBoundaryConditions(self,x0,goal):
        """Sets the start and goal"""
	RRT.setBoundaryConditions(self,x0,goal)
	#add cost term to root node
	self.root.c = 0
    def planMore(self,iters):
        foundNewPath = False
        for n in xrange(iters):
            self.numIters += 1
            n = RRT.expand(self)
            if n != None:
	        if not hasattr(n,'c'):
		    n.c = n.parent.c + self.objective.incremental(n.parent.x,n.uparent)
	        if self.goal != None:
		    if self.goal.contains(n.x):
                        totalcost = n.c + self.objective.terminal(n.x)
			if self.bestPath == None or totalcost < self.bestPathCost:
                            print "RRT found path with new cost",totalcost
                            foundNewPath = True
			    self.bestPathCost = totalcost
			    self.bestPath = RRT.getPath(self,n)
			    RRT.reset(self)
			    return True
		        if not self.doprune:
			    RRT.reset(self)
			return False
        if self.doprune and foundNewPath:
            prunecount = 0
            for n in self.nodes:
                if n.c > self.bestPathCost or self.prune(n):
                    prunecount += 1
	    if prunecount > len(self.nodes)/5 and prunecount > 100:
	        #print "Can prune",prunecount,"of",len(self.nodes),"nodes"
	        oldpruner = self.pruner
		if self.pruner == None:
		    self.pruner = lambda n:n.c >= self.bestPathCost
	        self.pruneTree()
		self.pruner = oldpruner
		#print "   pruned down to",len(self.nodes),"nodes"
	return False
    def prune(self,n):
        if not self.doprune or self.bestPathCost == None: return False
        if not hasattr(n,'c'):
            n.c = n.parent.c + self.objective.incremental(n.parent.x,n.uparent)
        if n.c > self.bestPathCost:
            return True
        if self.pruner:
            return self.pruner(node)
        return False
    def getPathCost(self):
        return self.bestPathCost
    def getPath(self):
        """Returns ([x0,...,xn],[u1,...,un]) for the base space."""
        if self.bestPath == None:
            return None
        return self.bestPath

class AnytimeRRT(RRT):
    """The Anytime Rapidly-exploring Random Tree planner (Ferguson and Stentz,
    06).
    """
    def __init__(self,controlSpace,objective,metric,edgeChecker,
                 **params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
        #amount that the cost/nearness weight gets shifted each time a path
        #is found
        self.weightIncrement = popdefault(params,'weightIncrement',0.1)
        #amount required for path cost shrinkage
        self.epsilon = popdefault(params,'epsilon',0.01)
        #must use brute force picking
        #commented out because pickNode does this already
        #params['nearestNeighborMethod'] = 'bruteforce'  
	RRT.__init__(self,controlSpace,metric,edgeChecker,**params)
        self.objective = objective
	self.bestPath = None
	self.bestPathCost = None
        self.distanceWeight = 1
        self.costWeight = 0

    def pickNode(self,xrand):
        """Picks a node closest to xrand."""
        nnear = None    
        dbest = infty
        for n in self.nodes:
            d = self.distanceWeight*self.metric(n.x,xrand) + self.costWeight*n.c
            if d < dbest and not self.prune(n):
                nnear = n
                dbest = d
        return nnear
    def reset(self):
        """Resets all planning effort"""
        RRT.reset(self)
	self.bestPath = None
	self.bestPathCost = None
        self.distanceWeight = 1
        self.costWeight = 0
    def setBoundaryConditions(self,x0,goal):
        """Sets the start and goal"""
	RRT.setBoundaryConditions(self,x0,goal)
	#add cost term to root node
	self.root.c = 0
    def planMore(self,iters):
        foundNewPath = False
        for n in xrange(iters):
            self.numIters += 1
            n = RRT.expand(self)
            if n != None:
	        if not hasattr(n,'c'):
		    n.c = n.parent.c + self.objective.incremental(n.parent.x,n.uparent)
	        if self.goal != None:
		    if self.goal.contains(n.x):
                        totalcost = n.c + self.objective.terminal(n.x)
			if self.bestPath == None or totalcost < self.bestPathCost*(1.0-self.epsilon):
                            print "Anytime-RRT found path with new cost",totalcost
                            self.distanceWeight -= self.weightIncrement
                            self.distanceWeight = max(self.distanceWeight,0)
                            self.costWeight += self.weightIncrement
                            self.costWeight = min(self.costWeight,1)
                            foundNewPath = True
			    self.bestPathCost = totalcost
			    self.bestPath = RRT.getPath(self,n)
			    RRT.reset(self)
			    return True
			return False
	return False
    def prune(self,n):
        if self.bestPathCost == None: return False
        if not hasattr(n,'c'):
            n.c = n.parent.c + self.objective.incremental(n.parent.x,n.uparent)
        if n.c > self.bestPathCost*(1.0-self.epsilon):
            return True
        if self.pruner:
            return self.pruner(node)
        return False
    def getPathCost(self):
        return self.bestPathCost
    def getPath(self):
        """Returns ([x0,...,xn],[u1,...,un]) for the base space."""
        if self.bestPath == None:
            return None
        return self.bestPath


class RepeatedEST(ESTWithProjections):
    """The repeated Expansive Space Tree planner.
    """
    def __init__(self,controlSpace,objective,edgeChecker,
                 **params):
        """Given a ControlSpace controlSpace, a metric, and an edge checker"""
	ESTWithProjections.__init__(self,controlSpace,edgeChecker,**params)
        self.objective = objective
	self.bestPath = None
	self.bestPathCost = None
	self.doprune = False
    def reset(self):
        """Resets all planning effort"""
        ESTWithProjections.reset(self)
	self.bestPath = None
	self.bestPathCost = None
    def setBoundaryConditions(self,x0,goal):
        """Sets the start and goal"""
	ESTWithProjections.setBoundaryConditions(self,x0,goal)
	#add cost term to root node
	self.root.c = 0
    def planMore(self,iters):
        foundNewPath = False
        for n in xrange(iters):
            self.numIters += 1
            n = ESTWithProjections.expand(self)
            if n != None:
	        if not hasattr(n,'c'):
		    n.c = n.parent.c + self.objective.incremental(n.parent.x,n.uparent)
	        if self.goal != None:
		    if self.goal.contains(n.x):
                        totalcost = n.c + self.objective.terminal(n.x)
			if self.bestPath == None or totalcost < self.bestPathCost:
                            print "EST found path with new cost",totalcost
                            foundNewPath = True
			    self.bestPathCost = totalcost
			    self.bestPath = ESTWithProjections.getPath(self,n)
			    ESTWithProjections.reset(self)
			    return True
		        if not self.doprune:
			    ESTWithProjections.reset(self)
			return False
        if self.doprune and foundNewPath:
            prunecount = 0
            for n in self.nodes:
                if n.c > self.bestPathCost or self.prune(n):
                    prunecount += 1
	    if prunecount > len(self.nodes)/5 and prunecount > 100:
	        #print "Can prune",prunecount,"of",len(self.nodes),"nodes"
	        oldpruner = self.pruner
		if self.pruner == None:
		    self.pruner = lambda n:n.c >= self.bestPathCost
	        self.pruneTree()
		self.pruner = oldpruner
		#print "   pruned down to",len(self.nodes),"nodes"
	return False
    def prune(self,n):
        if not self.doprune or self.bestPathCost == None: return False
        if not hasattr(n,'c'):
            n.c = n.parent.c + self.objective.incremental(n.parent.x,n.uparent)
        if n.c > self.bestPathCost:
            return True
        if self.pruner:
            return self.pruner(node)
        return False
    def getPathCost(self):
        return self.bestPathCost
    def getPath(self):
        """Returns ([x0,...,xn],[u1,...,un]) for the base space."""
        if self.bestPath == None:
            return None
        return self.bestPath
