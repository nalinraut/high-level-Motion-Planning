import math
from optimize import TrajectoryOptimizationProblem
from collections import deque
from ..condition import *
import random
from geometry import vectorops

class TreeNode:
    """Important note: since there are cycles, the garbage collector
    will not destroy this tree in the first-level pass.  To ensure that a
    tree is destroyed call the root's destroy() method before orphaning the
    instance."""
    def __init__(self,data=None,parent=None):
        self.data = data
        self.parent = parent
        self.children = []
        if self.parent:
            self.parent.children.append(self)

    def dfsiter(self):
        """Returns an iterator that walks the entire tree using depth-first 
        search"""
        yield self
        for c in self.children:
            for n in c.dfsiter():
                yield n
        return 

    def bfsiter(self):
        """Returns an iterator that walks the entire tree using breadth-first 
        search"""
        q = deque([self])
        while len(q) > 0:
            n = q.popleft()
            yield n
            for c in n.children:
                q.append(c)
        return 

    def destroy(self):
        for c in self.children:
            c.parent = None
            c.destroy()
        self.children = []

    def isAncestor(self,n):
        return n.isDescendant(self)

    def isDescendant(self,n):
        return n == self or (self.parent != None and self.parent.isDescendant(n))

    def path(self,root=None):
        res = []
        n = self
        while n != root:
            res.append(n)
            n = n.parent
        if root != None:
            res.append(root)
        res.reverse()
        return res

    def dataPath(self,root=None):
        return [p.data for p in self.path(root)]

class SearchState:
    def __init__(self,x,uparent=None):
        self.x = x
        self.uparent = None
        self.depth = 0
        self.pathCost = 0.0
        self.terminalCost = 0.0
        self.totalCost = 0.0
        self.isGoal = False

class TreeSearch:
    """Important note: to destroy the tree call the destroy() method."""
    def __init__(self,problem):
        self.problem = problem
        self.root = self.makeNode(problem.x0)

    def solutionPath(self,n):
        p = n.dataPath()
        us = [s.uparent for s in p[1:]]
        xs = [s.x for s in p]
        return (xs,us)

    def makeNode(self,x,parent=None,uparent=None):
        d = SearchState(x)
        if parent != None:
            d.depth = parent.data.depth + 1
            d.pathCost = parent.data.pathCost + self.problem.evalIncrementalCost(parent.data.x,uparent,parent.data.depth)
            d.uparent = uparent
        d.terminalCost = self.problem.evalTerminalCost(x,d.depth)
        d.totalCost = d.pathCost + d.terminalCost
        d.isGoal = self.problem.goalSet(x) if self.problem.goalSet!=None else True
        n = TreeNode(d,parent)
        self.onAdd(n)
        return n
        
    def destroy(self):
        self.root.destroy()
        self.root = None

    def onAdd(self,n):
        """Subclasses can override this to implement some callback"""
        pass


class RRTSearch(TreeSearch):
    """Given an optimal control problem and a cspace, builds a search tree
    via an RRT-like algorithm adapted to optimal control problems.
    
    cspace needs to have sample() and distance() methods.
    """
    def __init__(self,problem,cspace):
        TreeSearch.__init__(self,problem)
        self.cspace = cspace
        self.numControlSamples = 10
        self.descendProbability = 0.2
        self.expansionLimit = 1e300

        self.numExpands = 0
        self.uInfeasible = 0
        self.xInfeasible = 0

    def planFeasible(self,termCond):
        if isinstance(termCond,int):
            termCond = CountCondition(termCond)
        while termCond():
            n = self.step()
            if n!=None and n.data.isGoal:
                return n
        return None

    def planMinimum(self,termCond):
        if isinstance(termCond,int):
            termCond = CountCondition(termCond)
        while termCond():
            n = self.step()
        return self.optimalNode(requireGoal=True)
    
    def step(self):
        self.numExpands += 1
        if random.random() < 0.01:
            n = self.optimalNode(requireGoal=False)
            if n == None: return None
            ne = self.expandDescend(n,self.numControlSamples*10)
            if ne:
                self.expansionLimit = min(self.expansionLimit,ne.data.totalCost)
            return ne
        elif random.random() < self.descendProbability:
            #n = self.optimalNode(requireGoal=False)
            n = self.optimalBiasNode(3,requireGoal=False)
            if n == None: return None
            ne = self.expandDescend(n)
            #ne = self.expandToward(n,self.xsample())
            if ne:
                self.expansionLimit = min(self.expansionLimit,ne.data.totalCost)
            return ne
        else:
            xd = self.xsample()
            n = self.pickNode(xd)
            if n == None: return None
            ne = self.expandToward(n,xd)
            if ne:
                self.expansionLimit = min(self.expansionLimit,ne.data.totalCost)
            return ne
    
    def xsample(self):
        return self.cspace.sample()
    
    def pickNode(self,x):
        closest = None
        dmin = 1e300
        for n in self.root.dfsiter():
            if n.data.pathCost > self.expansionLimit:
                continue
            d = self.cspace.distance(n.data.x,x)
            if d < dmin:
                dmin, closest = d,n
        assert closest != None
        return closest
   
    def optimalNode(self,requireGoal=True):
        best = None
        bestCost = 1e300
        for n in self.root.dfsiter():
            if requireGoal and n.data.isGoal==False:
                continue
            if n.data.totalCost < bestCost:
                bestCost, best = (n.data.totalCost,n)
        return best

    def optimalBiasNode(self,power=2,requireGoal=True):
        nodes = [(n.data.totalCost,n) for n in self.root.dfsiter() if (not requireGoal or n.data.isGoal)]
        if power < len(nodes):
            return min([random.choice(nodes) for i in xrange(power)])[1]
        else:
            #approximate with continuous
            v = pow(random.uniform(0.0,1.0),1.0/power)
            nodes = sorted(nodes)
            return nodes[int(math.floor(v*len(nodes)))][1]

    def walkerExpandToward(self,n,x,delta=0.5,dt=0.1):
        #hack for walker
        d = self.cspace.distance(n.data.x,x)
        if d > delta:
            x = self.cspace.interpolate(n.data.x,x,delta/d)
        u = [dt]+vectorops.sub(x[6:12],n.data.x.q[6:])+vectorops.sub(x[18:],n.data.x.v[6:])
        if not self.problem.isUFeasible(n.data.x,u):
            self.uInfeasible += 1
            return None
        xu = self.problem.f(n.data.x,u)
        if not self.problem.isXFeasible(xu):
            self.xInfeasible += 1
            return None
        return self.makeNode(xu,n,u)

    
    def expandToward(self,n,x,numSamples=None):
        if self.problem.params['name']=='walker':
            return self.walkerExpandToward(n,x)
        """
        next = None
        for i in xrange(10):
            n = self.walkerExpandToward(n,x)
            if not n:
                return next
            next = n
        return next
        """

        if numSamples==None:
            numSamples = self.numControlSamples
        closest = None
        dmin = 1e300
        for i in xrange(numSamples):
            u = self.problem.usampler()
            if not self.problem.isUFeasible(n.data.x,u):
                self.uInfeasible += 1
                continue
            xu = self.problem.f(n.data.x,u)
            if not self.problem.isXFeasible(xu):
                self.xInfeasible += 1
                continue
            d = self.cspace.distance(xu,x)
            if d < dmin:
                dmin, closest = d, (u,xu)
        if closest==None: return None
        return self.makeNode(closest[1],n,closest[0])

    def expandDescend(self,n,numSamples=None):
        if numSamples==None:
            numSamples = self.numControlSamples
        closest = None
        dmin = 1e300
        for i in xrange(numSamples):
            u = self.problem.usampler()
            if not self.problem.isUFeasible(n.data.x,u):
                self.uInfeasible += 1
                continue
            xu = self.problem.f(n.data.x,u)
            if not self.problem.isXFeasible(xu):
                self.xInfeasible += 1
                continue
            c = n.data.pathCost + self.problem.evalIncrementalCost(n.data.x,u,n.data.depth) + self.problem.evalTerminalCost(xu,n.data.depth+1)
            if c < dmin:
                dmin, closest = c, (u,xu)
        if closest==None: return None
        return self.makeNode(closest[1],n,closest[0])


class DDRRTSearch(RRTSearch):
    """Given an optimal control problem and a cspace, builds a search tree
    via the dynamic-domain RRT algorithm.
    """
    def __init__(self,problem,cspace,R0=2.0,alpha=0.2):
        self.R0 = R0
        self.alpha = alpha
        self.bounds = None
        RRTSearch.__init__(self,problem,cspace)

    def xsample(self):
        if self.bounds == None:
            return self.cspace.sample()
        else:
            return [random.uniform(a-0.1,b+0.1) for (a,b) in self.bounds]

    def makeNode(self,x,parent=None,uparent=None):
        n = TreeSearch.makeNode(self,x,parent,uparent)
        #hack for walker problem
        if self.bounds == None:
            self.bounds = [(v,v) for v in x.q+x.v]
        else:
            self.bounds = [(min(a,v),max(b,v)) for v,(a,b) in zip(x.q+x.v,self.bounds)]
        #essentially infinite radius
        n.data.radius = 1e300
        return n
    
    def pickNode(self,x):
        closest = None
        dmin = 1e300
        for n in self.root.dfsiter():
            if n.data.pathCost > self.expansionLimit:
                continue
            d = self.cspace.distance(n.data.x,x)
            if d < dmin and d < n.data.radius:
                dmin, closest = d,n
        return closest
       
    def expandToward(self,n,x):
        res = RRTSearch.expandToward(self,n,x)
        if not res:
            if n.data.radius >= 1e300:
                n.data.radius = self.R0
            else:
                n.data.radius *= (1.0-self.alpha)
        else:
            if n.data.radius >= 1e300:
                pass
            else:
                n.data.radius *= (1.0+self.alpha)
        return res
