import math
import random
import numpy as np
import copy
import time
import Tkinter
import sys
from sample import weightedSample
from sample import stratifiedSample
from gaussian import GaussianDistribution

WIDTH = 800
HEIGHT = 600

def angle(a,b):
    """Returns the angle of the heading toward b with origin a"""
    return math.atan2(b[1]-a[1],b[0]-a[0])

def anglesub(a,b):
    """Return the smallest angular difference betwen a and b, (a-b)mod 2pi"""
    r = math.fmod(a-b,math.pi*2)
    if r < -math.pi: return r+2.0*math.pi
    if r > math.pi:  return r-2.0*math.pi
    return r

def distance(a,b):
    return math.sqrt(np.dot(b-a,b-a))

class AngularGaussianDistribution(GaussianDistribution):
    """A univariate Gaussian distribution on angles"""
    def __init__(self,mean,std):
        GaussianDistribution.__init__(self,mean,std)

    def log_probability(self,x):
        assert not isinstance(self.mean,(list,tuple))
        return -0.5*anglesub(x,self.mean)**2/self.std**2 - math.log(math.pi*2.0)*0.5 - math.log(self.std)    
     
    def probability(self,x):
        assert not isinstance(self.mean,(list,tuple))
        return math.exp(-0.5*anglesub(x,self.mean)**2/self.std**2)/(math.sqrt(math.pi*2.0)*self.std)      
    
    def sample(self):
        while True:
            v = random.gauss(self.mean,self.std)
            if abs(v-self.mean) <= math.pi:
                return v
        return None
    
    def toString(self):
        s = str( self.mean) + "\t" + str( self.std)
        return s
    
class JointConstraint( AngularGaussianDistribution):
    def __init__(self, mean, std, v_min, v_max):
        AngularGaussianDistribution.__init__( self, mean, std)
        self.v_min = v_min
        self.v_max = v_max
        assert self.v_min <= self.v_max
        
    def log_probability(self, x):
        if not self.isFeasible(x):
            return float("INF")
        return AngularGaussianDistribution.log_probability(self, x)
    
    def probability(self, x):
        if not self.isFeasible(x):
            return 0
        return AngularGaussianDistribution.probability(self, x)
        
    def isFeasible(self, x):
        if x <= self.v_max and x >= self.v_min:
            return True
        return False

    def sample(self):
        while True:
            t = AngularGaussianDistribution.sample(self)
            if self.isFeasible(t):
                return t
        return None
    
    def toString(self):
        s = AngularGaussianDistribution.toString(self) + "\t" + str( self.v_min) + "\t" + str( self.v_max)
        return s

class CenterOfMassConstraint(GaussianDistribution):
    def __init__(self, mean, std, x_min, x_max):
        GaussianDistribution.__init__(self, mean, std)
        self.x_min = x_min + mean[0]
        self.x_max = x_max + mean[0]
        assert self.x_min < self.x_max
        
    def isFeasible(self, x):
        if x[0] <= self.x_max and x[0] >= self.x_min:
            return True
        return False
    
    def log_probability(self, x):
        if not self.isFeasible(x):
            return float("INF")
        return GaussianDistribution.log_probability(self, x[0:1])
    
    def probability(self, x):
        if not self.isFeasible(x):
            return 0
        return GaussianDistribution.probability(self, x[0:1])
        
def calcIK( xa, xb, La, Lb):
    """Returns the two IK solutions for a point La and Lb distance away
    from given points xa and xb, or None if there are no solutions.
    The result is usually a pair of results (x1,x2), or can be None, or can
    be several results if there are an infinite number of solutions."""
    dx = xb-xa
    d2 = np.dot(dx,dx)
    if d2 > (La+Lb)**2:
        return None
    d = math.sqrt(d2)
    if d < 1e-10:
        if abs(La-Lb) < 1e-10:
            #infinite number of solutions! sample a few
            res = []
            for i in xrange(10):
                theta1 = random.uniform(0,math.pi*2)
                theta2 = -theta1
                dx = np.array([math.cos(theta1),math.sin(theta1)])*La
                x1 = xa+dx
                res.append(x1)
            return res
        else:
            raise ValueError("no solutions")
    a = 0.5*(La**2-Lb**2)/d**2 + 0.5
    c2 = (La/d)**2 - a**2
    if c2 < 0:
        raise ValueError("numerical error in ikSolutions?")
    c = math.sqrt(c2)
    x0 = xa+a*dx
    perp = np.array([-dx[1],dx[0]])
    x1 = x0+c*perp
    x2 = x0-c*perp
    assert(abs(distance(x1,xa)-La)<1e-6)
    assert(abs(distance(x2,xa)-La)<1e-6)
    assert(abs(distance(x1,xb)-Lb)<1e-6)
    assert(abs(distance(x2,xb)-Lb)<1e-6)
    return (x1,x2)

class Branched2DChain:
    def __init__(self):
        self.L =[]
        self.x = []
        self.parents = []
        self.children = []
        self.thetaPriors = []
        self.xPriors = []
        #a list of (index,pos) pairs
        self.fixedPositions = []

    def getState(self):
        return copy.deepcopy(self.x)

    def setState(self,state):
        self.x = copy.deepcopy(state)

    def copy(self,chain):
        self.L = chain.L[:]
        self.x = copy.deepcopy(chain.x)
        self.thetaPriors = copy.deepcopy(chain.thetaPriors)
        self.xPriors = copy.deepcopy(chain.xPriors)
        self.fixedPositions = copy.copy(chain.fixedPositions)

    def startBuild(self,x):
        self.L=[None]
        self.x=[x];
        self.parents = [-1]
        self.children = [[]]
        self.thetaPriors=[None]
        self.xPriors=[None]
        self.fixedPositions = [(0,x)]
    
    def add( self, L, dx, thetaPrior = None, xPrior = None, parent = 'last'):
        """Adds a new link"""
        if abs( math.sqrt( np.dot( dx, dx)) - L) > 1e-5:
            raise ValueError("Link length not appropriately set")
        self.L.append( L)
        if parent == 'last':
            parent = len(self.x)-1
        self.parents.append( parent)
        if parent >= 0:
            self.children[parent].append(len(self.x))
        self.x.append( self.x[parent] + dx)
        self.children.append([])
        self.thetaPriors.append( thetaPrior)
        self.xPriors.append( xPrior)

    def isFixed(self,index):
        return any([i==index for (i,x) in self.fixedPositions])

    def descendants(self,index):
        """Iterate over the descendants of index (inclusive of index)"""
        yield index
        for c in self.children[index]:
            for d in self.descendants(c):
                yield d
        raise StopIteration()
    
    def ancestors(self,index):
        """Iterate over the ancestors of index (inclusive of index)"""
        while index >= 0:
            yield index
            index = self.parents[index]
        raise StopIteration()

    def reroot(self,root):
        """Sets root to be the new root of the chain"""
        p = self.parents[root]
        if p < 0: return
        cind = self.children[p].index(root)
        del self.children[p][cind]
        self.parents[root] = -1
        
        self.reroot(p)

        self.children[root].append(p)
        self.parents[p] = root

        #copy edge information
        self.L[p] = self.L[root]
        self.L[root] = None

    def getAbsTheta(self,index):
        """Returns the absolute angle of the segment from x[p[index]] to
        x[index]"""
        if self.parents[index] < 0: return 0.0
        return angle(self.x[self.parents[index]],self.x[index])

    def getAbsThetas(self):
        """Returns a list of absolute angles of each segment"""
        return [self.getAbsTheta(i) for i in range(len(self.x))]

    def setAbsTheta(self,index,abstheta):
        """Sets x[index] from x[p[index]] and the absolute angle abstheta"""
        assert self.parents[index] >= 0
        dx = np.array([math.cos(abstheta),math.sin(abstheta)])*self.L[index]
        self.x[index]=self.x[self.parents[index]]+dx

    def setAbsThetaBwd(self,index,abstheta):
        """Sets x[p[index]] from x[index] and the absolute angle abstheta
        (so that the forward cumulative angle is abstheta)"""
        assert self.parents[index] >= 0
        dx = np.array([math.cos(abstheta),math.sin(abstheta)])*self.L[index]
        self.x[self.parents[index]] = self.x[index]-dx

    def getTheta(self,index):
        """Returns the angle of the segment from x[p[index]] to
        x[index] relative to the previous segment"""
        assert self.parents[index] >= 0
        return anglesub(self.getAbsTheta(index),self.getAbsTheta(self.parents[index]))

    def getThetas(self):
        """Returns a list of relative angles of each segment"""      
        abstheta = self.getAbsThetas()
        return [(None if self.parents[i]<0 else anglesub(theta,abstheta[self.parents[i]])) for (i,theta) in enumerate(abstheta)]

    def setTheta(self,index,theta):
        """Sets x[index] from x[p[index]] and the relative angle abstheta"""
        assert self.parents[index] >= 0
        abstheta = self.getAbsTheta(self.parents[index]) + theta
        self.setAbsTheta(index,abstheta)

    def translateChain(self,delta):
        """Translates the chain uniformly by delta"""
        self.x = [x+delta for x in self.x]

    def rotateChain(self,index,delta):
        """Rotates the chain starting from the parent of x[index], applying
        a rotation of magnitude delta to all descendants of index"""
        assert self.parents[index] >= 0
        origin = self.x[self.parents[index]]
        s = math.sin(delta)
        c = math.cos(delta)
        R = np.array([[c,-s],[s,c]])
        for d in self.descenants(index):
            self.x[d] = np.dot(R,self.x[d] - origin) + origin

    def rotateChainBwd(self,index,delta):
        """Rotates the chain starting from the parent of x[index], applying
        a rotation of magnitude delta to all ancestors of p[index]"""
        origin = self.x[index]
        s = math.sin(delta)
        c = math.cos(delta)
        R = np.array([[c,-s],[s,c]])
        for d in self.ancestors(index):
            self.x[d] = np.dot(R,self.x[d] - origin) + origin

    def setFixed(self,index):
        """Sets a fixed position"""
        self.fixedPositions.append((index,self.x[index]))

    def openEndpoints(self):
        self.fixedPositions = []

    def closeEndpoints(self):
        self.fixedPositions = []
        for i in xrange(len(self.x)):
            if self.parents[i] < 0 or len(self.children[i])==0:
                self.setFixed(i)

    def is_closed(self,tol=1e-5):
        """Determines whether the chain is closed properly"""
        for i in xrange(len(self.x)):
            if self.parents[i] < 0: continue
            d = distance(self.x[i],self.x[self.parents[i]])
            if abs(d - self.L[i]) > tol:
            #if abs(d-L[i]) > tol:
                return False
        for (i,x) in self.fixedPositions:
            if distance(self.x[i],x) > tol:
                return False
        return True

    def jointAngleDerivative(self,pt,joint):
        """Returns the jacobian of the point indexed by pt, with respect
        to joint angle theta[joint].  Assumes joint is an ancestor of point"""
        assert self.parents[joint] >= 0
        dx = self.x[pt]-self.x[self.parents[joint]]
        return np.array([-dx[1],dx[0]])

    def jacobian(self,pt):
        """Fills out the jacobian of the point indexed by pt, with respect
        to joint angles theta"""
        J = np.zeros((2,len(self.x)))
        i = pt
        while i >= 0:
            if self.parents[i] >= 0:
                dx = self.x[pt]-self.x[self.parents[i]]
                J[0,i] = -dx[1]
                J[1,i] = dx[0]
            i = self.parents[i]
        return J

    def metricTensor(self,pt):
        """Returns the value JJ^T for J = the Jacobian of the given
        point w.r.t. joint angles theta"""
        if pt < 0: pt = len(self.x)+pt
        JJt = np.zeros((2,2))
        i = pt
        while i >= 0:
            if self.parents[i] >= 0:
                dx = self.x[pt]-self.x[i]
                JJt[0,0] += dx[1]**2
                JJt[1,1] += dx[0]**2
                JJt[0,1] -= dx[0]*dx[1]
                JJt[1,0] -= dx[1]*dx[0]
        return JJt

    def ikSolutions(self,index):
        """Returns the two IK solutions for x[index] given x[parent] and
        x[child], or None if there are no solutions.  The result is a
        pair of results (x1,x2).
        
        If there are an infinite number of solutions, this samples k
        sample solutions (k=10 right now)"""
        parent = self.parents[index]
        assert parent >= 0
        assert len(self.children[index])==1
        child = self.children[index][0]
        return calcIK(self.x[parent],self.x[child],self.L[index],self.L[child])

    def closeChainSample(self,i,checkFeasible=True):
        """Finds the IK solutions for x[i] given its parent and child, and
        sets the chain to one of those two solutions at random.
        """
        solns = self.ikSolutions(i)
        if solns == None:
            raise ValueError("Chain can't be closed?")
        if checkFeasible:
            feasibleSolns = []
            for xi in solns:
                if self.xPriors[i]!=None and not self.xPriors[i].isFeasible(xi):
                    continue
                self.x[i] = xi
                if self.thetaPriors[i]!=None and not self.thetaPriors[i].isFeasible(self.getTheta(i)):
                    continue
                c = self.children[i][0]
                if self.thetaPriors[c]!=None and not self.thetaPriors[c].isFeasible(self.getTheta(c)):
                    continue
                feasibleSolns.append(xi)
            if len(feasibleSolns) > 0:
                self.x[i]=random.choice(feasibleSolns)
                return
        self.x[i] = random.choice(solns)

    def xLogProbability(self,i):
        return self.xPriors[i].log_probability(self.x[i]) if self.xPriors[i] != None else 0.0 

    def thetaLogProbability(self,i):
        return self.thetaPriors[i].log_probability(self.getTheta(i)) if self.thetaPriors[i] != None else 0.0 
    
    def xFeasible(self, i):
        return self.xPriors[i].isFeasible( self.x[i]) if self.xPriors[i] != None else True
    
    def thetaFeasible(self, i):
        parent = self.parents[i]
        assert( parent != -1)
        grandparent = self.parents[parent]
        assert( grandparent != -1)
        
        return self.thetaPriors[i].isFeasible(self.getTheta(i)) if self.thetaPriors[i] != None else True
    
    def blockFeasible(self, block):
        if block.type == Block.STRAIGHT:
            index_theta = []
            if self.parents[ block.a] != -1:
                index_theta.append( block.i)
            index_theta.append( block.n)
            index_theta.append( block.b)
              
            for c in self.children[ block.b]:
                index_theta.append( c)             
            for index in index_theta:
                if not self.thetaFeasible( index):
#                    print "Error Part1:\t" + str(index)
                    return False
                
            index_x = [ block.i, block.n]           
            for index in index_x:
                if not self.xFeasible( index):
#                    print "Error Part2:\t" + str(index)
                    return False
            return True
        
        elif block.type == Block.OPENEND:
            return self.thetaFeasible( block.i) and self.xFeasible( block.i)
        
        elif block.type == Block.BRANCH2:
            index_theta = []
            if self.parents[ block.a] != -1:
                index_theta.append( block.i)
            index_theta.append( block.n)
            index_theta.append( block.b)
            
            for c in self.children[ block.b]:
                index_theta.append(c)             
            for index in index_theta:
                if not self.thetaFeasible( index):
#                    print "Error Part1:\t" + str(index)
                    return False
            index_x = [ block.i, block.n]           
            for index in index_x:
                if not self.xFeasible( index):
#                    print "Error Part1:\t" + str(index)
                    return False
                
            for (pi,qi) in zip( block.p,block.q):
#                print "pi:" + str(pi) + "qi:" + str(qi)
                if self.thetaFeasible( pi) == False or self.thetaFeasible( qi) == False or self.xFeasible( pi) == False:
#                    print "Error Part1:\t" + str( pi) + "\t" + str( qi)                    
                    return False
                for c in self.children[qi]:
                    #print c
                    if self.thetaFeasible(c) == False:
#                        print "Error Part1:\t" + str(c)
                        return False
                
            return True
        elif block.type == Block.BRANCH1:
            size = len( block.b)
            if self.xFeasible( block.i) == False:
#                print "Error Part2:\t" + str( block.i)
                return False
            if self.parents[ block.a] != -1:
                if self.thetaFeasible( block.i) == False:
                    return False
            for index in range( size):
                if self.thetaFeasible( block.n[index]) == False:
#                    print "Error Part1:\t" + str( block.n[index])
                    return False
                if self.thetaFeasible( block.b[ index]) == False:
#                    print "Error Part1:\t" + str( block.b[ index])
                    return False
                if self.xFeasible( block.n[index]) == False:
#                    print "Error Part2:\t" + str( block.n[index])
                    return False
                for c in self.children[block.b[index]]:
                    if self.thetaFeasible( c) == False:
#                        print "Error Part1:\t" + str( c)
                        return False
            return True
        else:
            print "Error! Block type not supported!"

    def forwardKinematics(self,root,theta):
        """Sets joint angles theta starting from root and descending"""
        for d in self.descendants(root):
            if d == root: continue
            self.setAbsTheta(d,theta[d]+self.getAbsTheta(self.parents[d]))

    def backwardKinematics(self,root,theta):
        """Sets joint angles theta starting from root and ascending"""
        for a in self.ancestors(root):
            if self.parents[a] < 0: continue
            self.setAbsThetaBwd(a,self.getAbsTheta(a)-theta[self.parents[a]])

    def perturbGaussian(self,xradius,thetaradius,root=0):
        """Perturbs the conformation according to a gaussian magnitude,
        beginning at the root position"""
        self.x[root][0] += random.gauss(0.0,xradius)
        self.x[root][1] += random.gauss(0.0,xradius)
        theta = self.getAbsThetas()
        for i in xrange(1,len(theta)):
            theta[i] += random.gauss(0.0,thetaradius)
        self.forwardKinematics(root,theta)
        self.backwardKinematics(root,theta)
        
    def sampleRandom(self,root=0):
        """Samples a random conformation, beginning at the root position"""
        if self.xPriors[root] != None:
            self.x[root] = self.xPriors[root].sample()
        else:
            if not self.isFixed(root):
                raise ValueError("sampleRandom either needs to have a prior or start at a fixed position")
            #set fixed position
            for (i,pos) in self.fixedPositions:
                if root == i:
                    self.x[root] = pos
                    break
        theta = [0.0]*len(self.x)
        for i in xrange(0,len(self.x)):
            theta[i] = self.thetaPriors[i].sample() if self.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        self.forwardKinematics(root,theta)
        self.backwardKinematics(root,theta)

class Metropolis2DChainSampler:
    def __init__(self,chain):
        self.chain = chain
        self.xRadius = 0.2
        self.thetaRadius = 0.25
        #self.xRadius = 0.02
        #self.thetaRadius = 0.025
        self.maxSamples = 10

    def setSampleRandom(self):
        self.xRadius = None
        self.thetaRadius = None

    def setSamplePerturb(self,xradius,thetaradius):
        self.xRadius = xradius
        self.thetaRadius = thetaradius

    def sample(self):
        """Performs metropolis sampling.  Returns True if the proposed move
        was accepted"""
        assert (self.chain.endPosition == None),"Can only do standard metropolis on free chains"
        prfunc = None
        samplefunc = None
        if self.xRadius == None:
            prfunc = self.logProbabilitySample
            samplefunc = self.chain.sampleRandom
        else:
            prfunc = self.logProbability
            samplefunc = lambda : self.chain.perturbGaussian(self.xRadius,self.thetaRadius,0)
        oldx = self.chain.getState()
        lpold = prfunc()
        for iters in xrange(self.maxSamples):
            sampleres = samplefunc()
            #enforce start constraint
            if len(self.chain.fixedPositions) != 0:
                if len(self.chain.fixedPositions) != 1:
                    raise ValueError("Metropolis2DChainSampler cannot sample multiply closed chains")
                (index,pos) = self.chain.fixedPositions[0]
                self.chain.translateChain(pos-self.chain.x[index])
            lpnew = prfunc()
            if lpnew < lpold:
                alpha = math.exp(lpnew - lpold)
                if random.random() > alpha:
                    #reject move
                    self.chain.setState(oldx)
                    continue
            #print "Accepted move"
            return True
        #print "Rejected",self.maxSamples," candidate moves"
        return False

    def logProbabilitySample(self):
        """Returns the log probability of the conformation divided by the
        the sampling distribution x[0],theta[1],...,theta[n]"""            
        lp = 0.0
        for i in xrange(1,len(self.chain.x)):
            lp += self.chain.xLogProbability(i)
        Js = [np.hstack([np.eye(2),self.chain.jacobian(i)[:,1:]]) for i in xrange(len(self.chain.x)) if self.chain.xPriors[i] != None]
        Jthetas = []
        for i in xrange(1,len(self.chain.thetaPriors)):
            if self.chain.thetaPriors != None:
                Jthetas.append(np.zeros((1,1+len(self.chain.x))))
                Jthetas[-1][0,i] = 1.0
        J = np.vstack(Js+Jthetas)
        G = np.dot(J.T,J)
        (sign,logdetG) = np.linalg.slogdet(G)
        assert sign > 0
        lp += 0.5*logdetG
        return lp

    def logProbability(self):
        """Returns the log probability of the conformation"""
        lp = 0.0
        for i in xrange(1,len(self.chain.x)):
            lp += self.chain.thetaLogProbability(i)
        for i in xrange(0,len(self.chain.x)):
            lp += self.chain.xLogProbability(i)
        #TEMP: disregard metric tensor
        #return lp
        Js = [np.hstack([np.eye(2),self.chain.jacobian(i)[:,1:]]) for i in xrange(len(self.chain.x)) if self.chain.xPriors[i] != None]
        #print np.vstack(Js)
        angleWeight = 1.0
        Jthetas = []
        for i in xrange(1,len(self.chain.thetaPriors)):
            if self.chain.thetaPriors[i] != None:
                Jthetas.append(np.zeros((1,2+len(self.chain.theta)-1)))
                Jthetas[-1][0,1+i] = 1.0*angleWeight
        J = np.vstack(Js+Jthetas)
        #print J
        G = np.dot(J.T,J)
        #print G
        (sign,logdetG) = np.linalg.slogdet(G)
        assert sign > 0
        #print logdetG
        lp += 0.5*logdetG
        return lp
#    bodyPrior = AngularGaussianDistribution( math.pi / 4.0, math.pi * 5.0 / 180.0)

def angleRange(joint,ptr,target,Lremaining):
    """For a joint centered at the point joint, returns the range of angles
    theta in [theta0,theta0+delta] such that the point (ptr,1)
    rotated by theta is within Lremaining of the
    point target. Returns the pair (theta0,delta) or None if no such angles
    exist."""
    r0 = ptr
    D = distance(joint,target)
    if D+r0 <= Lremaining:
        #inside ball centered around target
        return (0,2.0*math.pi)
    elif D > Lremaining+r0:
        return None

    px = (D**2-Lremaining**2+r0**2)/(2*D)
    py = math.sqrt(r0**2-px**2)
    theta1 = math.atan2(py,px)
    theta2 = math.atan2(-py,px)
    thetaofs = angle(joint,target)
    #print "Polar coords of target, relative to joint",thetaofs,D
    #print "px,py",(px,py)
    #print "Theta1",theta1,"theta2",theta2
    #start of range
    ta = theta2+thetaofs
    #size of range
    trange = 2*theta1
    #print "Theta start",ta,"range",trange
    assert trange >= 0
    return (ta,trange)

def angle_range_intersect(a,b):
    """For two angle ranges given in the form (low,width)
    return an angle range that is the intersection of a and b."""
    assert(a[1]>=0)
    assert(b[1]>=0)
    if a[1]>=2.0*math.pi:
        return b
    elif b[1]>=2.0*math.pi:
        return a
    (alo,aw) = (a[0],a[1])
    (blo,bw) = (b[0],b[1])
    if anglesub(blo,alo) <= aw and anglesub(blo,alo) >= 0:
        #a contains blo
        iw = min(bw,aw-anglesub(blo,alo))
        assert iw >= 0
        return (blo,iw)
    elif anglesub(alo,blo) <= bw and anglesub(alo,blo) >= 0:
        iw = min(aw,bw-anglesub(alo,blo))
        assert iw >= 0
        return (alo,iw)
    else:
        return None


def angle_ranges_intersect(angleRanges):
    """For a list of angle ranges given in the form (middle,width)
    return an angle range that is the intersection of all ranges."""
    if len(angleRanges)==1:
        return angleRanges[0]
    res = angleRanges[0]
    for arange in angleRanges[1:]:
        res = angle_range_intersect(res,arange)
        if res==None: return None
    return res

def angleRanges(joint,ptr,targets,Lremaining):
    """For a joint centered at the point joint, returns the range of angles
    theta in [theta0-delta,theta0+delta] such that the point (ptr,1)
    rotated by theta is within Lremaining[i] of the
    point targets[i]. Returns the pair (theta0,delta) or None if no such angles
    exist."""
    res = [angleRange(joint,ptr,tgti,Li) for (tgti,Li) in zip(targets,Lremaining)]
    if any(r == None for r in res): return None
    return angle_ranges_intersect(res)


def angleSampleRLG(joint,pttheta,ptr,targets,Lremainings, thetaPrior=None,maxSamples=100,discretizeThreshold=0.1):
    """For a joint centered at the point joint, samples an angle theta
    according to the prior such that the point pt (given in polar
    coordinates pttheta,ptr) rotated by theta is within Lremaining of the
    point target. If no prior is given, samples
    randomly"""
    trange = angleRanges(joint,ptr,targets,Lremainings)
    if trange == None:
        raise ValueError("Chain out of reach")
    if thetaPrior and hasattr(thetaPrior,'v_min'):
        trange = angle_range_intersect(trange,(thetaPrior.v_min+pttheta,thetaPrior.v_max-thetaPrior.v_min))
    if trange == None:
        raise ValueError("Chain out of reach w/joint limits")
    (ta,trange) = trange
    if trange >= math.pi:
        #inside ball centered around target
        return thetaPrior.sample() if thetaPrior else random.uniform(0,2*math.pi)
    if thetaPrior==None:
        #print "Random sampling range",ta,trange
        return anglesub(ta + random.uniform(0,trange),pttheta)

    #we've got a prior
    assert(trange>=0)
    if trange == 0:
        print "Single point?",ta,trange
        return anglesub(ta,pttheta)
    if trange < discretizeThreshold:
        #discretize prior in range
        stepsize = max(trange/maxSamples,0.001)
        n = math.ceil(trange/stepsize)
        #print "Discretizing angular range",trange,"into",n,"bins"
        stepsize = trange / n
        thetas = [anglesub(ta + (i+0.5)*stepsize,pttheta) for i in xrange(int(n))]
        p = [thetaPrior.probability(theta) for theta in thetas]
        i = weightedSample(p)
        return thetas[i] + (random.random()+0.5)*stepsize
    else:
        #rejection sample
        for iters in xrange(maxSamples):
            theta = thetaPrior.sample()
            dtheta = anglesub(pttheta+theta,ta)
            if dtheta >= 0 and dtheta <= trange:
                #print "Rejection sampling succeeded in",iters+1,"iterations"
                return theta
    #failed to rejection sample
    return anglesub(ta + random.uniform(0,trange),pttheta)

class Block:
    """The "type" member tells you which type of block this is.
    - STRAIGHT blocks have structure a->i->n->b
    - BRANCH1 blocks have structure a->i->n[k]->b[k]
      with n and b arrays
    - BRANCH2 blocks have structure a->i->n->b
                                          v
                                          p[k]->q[k]
      with p and q arrays
    - OPENSTART blocks have structure i->n->b
    - OPENEND blocks have structure a->i
    - OPENEND2 blocks have structure a->i->n
    - Other structures are not supported and type is set to INVALID.
    Optional: open side-chains are specified in the 'side' member
    """
    INVALID = -1
    STRAIGHT = 0
    BRANCH1 = 1
    BRANCH2 = 2
    OPENSTART = 3
    OPENEND = 4
    OPENEND2 = 5

    def __init__(self,index,chain):
        """Index is the first changed index of the block"""
        self.i = index
        self.type = Block.INVALID
        self.activeIndices = [index]
        self.fixedIndices = []
        self.a = chain.parents[index]
        self.side = []
        
        if self.a < 0:
            #may be an OPENSTART
            if len(chain.children[index]) != 1:
                raise ValueError("Invalid open-start block")
            self.n = chain.children[index][0]
            self.activeIndices.append(self.n)
            if len(chain.children[self.n]) != 1:
                raise ValueError("Invalid open-start block")
            self.b = chain.children[self.n][0]
            self.fixedIndices.append(self.b)
            self.type = Block.OPENSTART
            return

        self.fixedIndices.append(self.a)
        if len(chain.children[index]) == 0:
            self.type = Block.OPENEND
            return
        elif len(chain.children[index]) == 1:
            self.n = chain.children[index][0]
            self.activeIndices.append(self.n)
            if len(chain.children[self.n]) == 0:
                #open end
                self.type = Block.OPENEND2
                return
            elif len(chain.children[self.n]) == 1:
                #straight
                self.b = chain.children[self.n][0]
                self.fixedIndices.append(self.b)
                self.type = Block.STRAIGHT
                return
            else:
                #may be a BRANCH2, or a STRAIGHT with side chains
                self.b = chain.children[self.n][0]
                self.p = []
                self.q = []
                for p in chain.children[self.n][1:]:
                    if len(chain.children[p]) > 1:
                        raise ValueError("Invalid multiple-branching structure")
                    elif len(chain.children[p])==0:
                        #side chain
                        self.side.append(p)
                    else:
                        self.p.append(p)
                        self.q.append(chain.children[p][0])
                self.fixedIndices.append(self.b)
                self.activeIndices += self.p
                self.fixedIndices += self.q
                if len(self.p) != 0:
                    self.type = Block.BRANCH2
                else:
                    self.type = Block.STRAIGHT
                return
        else:
            #may be a BRANCH1 or a STRAIGHT with side chains
            self.n = []
            for n in chain.children[index]:
                if len(chain.children[n]) > 1:
                    raise ValueError("Invalid multiple-branching structure")
                elif len(chain.children[n]) == 0:
                    self.side.append(n)
                else:
                    self.n.append(n)
            self.activeIndices += self.n
            self.b = []
            for n in self.n:
                self.b.append(chain.children[n][0])
                self.fixedIndices.append(self.b[-1])
            if len(self.n)==0:
                raise ValueError("Structure only has side chains (star)?")
            if len(self.n)==1:
                self.n = self.n[0]
                self.b = self.b[0]
                self.type = Block.STRAIGHT
            else:
                self.type = Block.BRANCH1
            return
    
class Gibbs2DChainSampler:
    def __init__(self,chain):
        self.chain = chain
        self.maxSamplesPerBlock = 10

    def straightBlockMetricSqrt(self,theta1,theta2,theta3,L1,L2,L3):
        """Returns the sqrt of the metric tensor for sampling a 3-link chain
        with absolute angles theta1,theta2,theta3 with link lengths L1,L2,L3"""
        #expression obtained through mathematica
        L1s,L2s,L3s=L1**2,L2**2,L3**2
        den = abs(math.sin(theta2-theta3)*math.sqrt(2)*L2*L3)
        if den  < 1e-3*L2*L3: den = 1e-3*L2*L3
        return math.sqrt(L1s*L2s + L1s*L3s + L2s*L3s + 2*L1s*L2s*L3s -
                         L1s*L2s*(1 + L3s)*math.cos(2*(theta1 - theta2)) -
                         L1s*L3s*math.cos(2*(theta1 - theta3)) - 
                         L2s*L3s*math.cos(2*(theta2 - theta3)) - 
                         L1s*L2s*L3s*math.cos(2*(theta2 - theta3))) /den;

    def sideChainSample(self,block):
        if len(block.side)==0: return
        chain = self.chain
        for s in block.side:
            theta = chain.thetaPriors[s].sample() if chain.thetaPriors[s] else random.uniform(0,2*math.pi)
            chain.setTheta(s,theta)

    def sideChainLogImportance(self,block):
        lp = 0.0
        for s in block.side:
            lp += self.chain.xLogProbability(s)
        return lp

    def straightBlockSample(self,block):
        """
        Given a block a->i->n->b, samples x[i] and x[n] given x[a] and x[b].
        Returns true if successful.  Otherwise, the state of the variables
        x[i] and x[n] are undefined"""
        chain = self.chain
        assert( block.type == Block.STRAIGHT)
        a,i,n,b = block.a,block.i,block.n,block.b       
        
#        print "Current Block Index:\t" + str(i)
#        if chain.thetaPriors[i] != None:
#            print "Current Prior:\t" + chain.thetaPriors[i].toString() 
#        else:
#            print "No Prior Found!"
#        theta1 = chain.thetaPriors[i].sample() if chain.thetaPriors[i] != None else random.uniform(0,2 * math.pi)
#        print "The theta I got is:" + str( theta1)
        theta1 = angleSampleRLG(chain.x[a],chain.getAbsTheta(a),chain.L[i],
                                [chain.x[b]],
                                [chain.L[n]+chain.L[b]],
                                chain.thetaPriors[i])
        chain.setAbsTheta( i, chain.getAbsTheta(a) + theta1)
        if abs( distance( chain.x[i], chain.x[b])) <= chain.L[n] + chain.L[b]:
            #reachable
            try:
                #solve for IK constraint
                chain.closeChainSample(n)
                self.sideChainSample(block)
                return True
            except ValueError:
                return False
        else:
            print "angleSampleRLG failed?"
        return False

    def straightBlockLogImportance(self,block):
        """Returns the log-importance weight w'
          log P(x[i,n],theta[i,n,b,c]|x[a],x[b]) -
          log Q(x[i,n],theta[i,n,b,c]|x[a],x[b])
        to be used in importance sampling. """
        chain = self.chain
        assert(block.type == Block.STRAIGHT)
        a,i,n,b = block.a,block.i,block.n,block.b
        pt1 = chain.thetaLogProbability(n)
        pt2 = chain.thetaLogProbability(b)
        pt3 = sum(chain.thetaLogProbability(c) for c in chain.children[b])
        px1 = chain.xLogProbability(i)
        px2 = chain.xLogProbability(n)
        try:
            return pt1+pt2+pt3+px1+px2+math.log(self.straightBlockMetricSqrt(chain.getAbsTheta(i),chain.getAbsTheta(n),chain.getAbsTheta(b),chain.L[i],chain.L[n],chain.L[b]))+self.sideChainLogImportance(block)
        except ValueError:
            #domain error at singularity
            return 1e308

    def straightBlockMetropolis(self,block):
        """Given a straight block a->i->n->b, samples x[i] and x[n] given
        x[a], x[b] up to maxSamplesPerBlock times in order to get a
        metropolis sample.
        """
        chain = self.chain
        assert(block.type == Block.STRAIGHT)
        a,i,n,b = block.a,block.i,block.n,block.b

        R = distance(chain.x[a],chain.x[b])
        L = chain.L[i]+chain.L[n]+chain.L[b]
        if abs(R-L)<1e-5:
            #straightened out, only 1 solution
            print "Gibbs step: straightened out"
            return True
        if R > L:
            print "Gibbs step: points too far away",R,">",L
            raise False

        #in case the sampling fails, revert to old
        xold = [chain.x[i],chain.x[n]]
        
        #        print "Calculating old block:"
        
        oldlogw = self.straightBlockLogImportance(block)
        bestnewlogw = -1e100
        for iters in xrange(self.maxSamplesPerBlock):
            if self.straightBlockSample(block):
                if not chain.blockFeasible( block):
                    continue
                newlogw = self.straightBlockLogImportance(block)
                if newlogw > bestnewlogw:
                    bestnewlogw = bestnewlogw
                alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
                if random.random() <= alpha:
                    #accept move
                    return True

        #restore to old pristine copy
        [chain.x[i],chain.x[n]] = xold
        
        return False
    
    def open_endBlockSample(self, block):
        assert block.type == Block.OPENEND
        chain = self.chain
        a, i = block.a, block.i
        theta1 = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        chain.setTheta( i, theta1)
        return True

    def open_endBlockLogImportance(self, block):
        chain = self.chain
        i = block.i
        Pt = chain.thetaLogProbability(i)
        Px = chain.xLogProbability(i)
        Q = 0
        return Pt + Px - Q
    
    def open_endBlockMetropolis(self, block):    
        chain = self.chain
        #in case the sampling fails, revert to old
        xold = [chain.x[index] for index in block.activeIndices]

        oldlogw = self.open_endBlockLogImportance(block)
        for iters in xrange(self.maxSamplesPerBlock):
            if self.open_endBlockSample(block):
                if not chain.blockFeasible( block):
                    continue
                newlogw = self.open_endBlockLogImportance(block)
                alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
                if random.random() <= alpha:
                    #accept move
                    return True

        #restore to old pristine copy
        for index,val in zip(block.activeIndices,xold):
            chain.x[index] = val
        return False
    
    def branch1BlockSample(self,block):
        """
        
        Given a block a->i->n[0]->b[0],
                         v  ..........
                         v  n[k]->b[k]
        samples x[i],x[n], and x[p[k]] for all k given x[a], x[b], and x[q[k]].
        
        Returns true if successful.  Otherwise, the state of the variables
        x[i],x[n],x[p[k]] for all k are undefined.
        """
        chain = self.chain
        assert(block.type == Block.BRANCH1)
        a,i,n,b = block.a,block.i,block.n,block.b
        #theta1 = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2 * math.pi)
        theta1 = angleSampleRLG(chain.x[a],chain.getAbsTheta(a),chain.L[i],
                                [chain.x[bi] for bi in b],
                                [chain.L[ni]+chain.L[bi] for (ni,bi) in zip(n,b)],
                                chain.thetaPriors[i])
        chain.setTheta(i,theta1)
                
        for (ni,bi) in zip(n,b) :
            if abs(distance(chain.x[i],chain.x[bi]))<=chain.L[ni]+chain.L[bi]:
                #reachable
                try:
                    chain.closeChainSample(ni)
                    self.sideChainSample(block)
                except ValueError:
                    return False
            else:
                print "angleSampleRLG-multiple failed"
                return False
        return True
    
    def branch1BlockLogImportance(self,block):
        """Returns the log-importance weight w'
          log P(x[i,n],theta[i,n,b,c]|x[a],x[b]) -
          log Q(x[i,n],theta[i,n,b,c]|x[a],x[b])
        to be used in importance sampling. """
        
        chain = self.chain
        assert(block.type == Block.BRANCH1)
        a,i,n,b = block.a,block.i,block.n,block.b

        px1 = chain.xLogProbability(i)           
        psum = px1
        
        for index in range( len(n)):
            pt1 = chain.thetaLogProbability(n[index])
            pt2 = chain.thetaLogProbability(b[index])
            pt3 = sum(chain.thetaLogProbability(c) for c in chain.children[b[index]])
            px2 = chain.xLogProbability(n[index])
            psum += (pt1 + pt2 + pt3 + px2)
         
        importance = psum
        for index in range( len(n)):            
            try:
                importance += math.log(self.straightBlockMetricSqrt(chain.getAbsTheta(i),chain.getAbsTheta(n[index]),chain.getAbsTheta(b[index]),chain.L[i],chain.L[n[index]],chain.L[b[index]]))
            except ValueError:
                #domain error at singularity
                return 1e308

        importance += self.sideChainLogImportance(block)
        return importance
#        return pt1+pt2+pt3+px1+px2+math.log(self.straightBlockMetricSqrt(chain.getAbsTheta(i),chain.getAbsTheta(n),chain.getAbsTheta(b),chain.L[i],chain.L[n],chain.L[b]))
        
    def branch1BlockMetropolis(self, block):
        chain = self.chain
                #in case the sampling fails, revert to old
        xold = [chain.x[index] for index in block.activeIndices+block.side]
            
        oldlogw = self.branch1BlockLogImportance(block)
        bestnewlogw = -1e100
        for iters in xrange(self.maxSamplesPerBlock):
            if self.branch1BlockSample(block):
                if not chain.blockFeasible(block):
                    continue
                newlogw = self.branch1BlockLogImportance(block)
                if newlogw > bestnewlogw:
                    bestnewlogw = bestnewlogw
                    
                alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
                if random.random() <= alpha:
                    #accept move 
                    print "  Old Importance:\t" + str(oldlogw)
                    print "  New Importance:\t" + str(newlogw) 
                    return True

        print "  Old Importance:\t" + str(oldlogw)
        print "  Best new Importance (reject):\t" + str(bestnewlogw) 
        #restore to old pristine copy
        for index,val in zip(block.activeIndices+block.side,xold):
            chain.x[index] = val
        return False
        
    def branch2BlockSample(self,block):
        """
        
        Given a block a->i->n->b,
                            v
                            p[k]->q[k]
        samples x[i],x[n], and x[p[k]] for all k given x[a], x[b], and x[q[k]].
        
        Returns true if successful.  Otherwise, the state of the variables
        x[i],x[n],x[p[k]] for all k are undefined.
        """
        chain = self.chain
        assert(block.type == Block.BRANCH2)
        a,i,n,b = block.a,block.i,block.n,block.b
        p,q = block.p,block.q
        #theta1 = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        theta1 = angleSampleRLG(chain.x[a],chain.getAbsTheta(a),chain.L[i],
                                [chain.x[b]]+[chain.x[qi] for qi in q],
                                [chain.L[n]+chain.L[b]]+[chain.L[n]+chain.L[pi]+chain.L[qi] for (pi,qi) in zip(p,q)],
                                chain.thetaPriors[i])
        chain.setAbsTheta(i,chain.getAbsTheta(a)+theta1)
        if abs(distance(chain.x[i],chain.x[b]))<=chain.L[n]+chain.L[b]:
            #reachable
            try:
                #solve for all IK constraints
                solns = calcIK(chain.x[i],chain.x[b],chain.L[n],chain.L[b])
                if len(solns)==0:
                    raise ValueError("Chain cannot be closed")
                chain.x[n] = random.choice(solns)
                for (pi,qi) in zip(p,q):
                    if abs(distance(chain.x[n],chain.x[qi]))>chain.L[pi]+chain.L[qi]:
                        #print "out of reach of child",pi,qi
                        return False
                for (pi,qi) in zip(p,q):
                    chain.closeChainSample(pi)
                self.sideChainSample(block)
                return True
            except ValueError as e:
                print "exception thrown:",e
                return False
        print "out of reach of initial child",n,b
        return False

    def branch2BlockMetricSqrt(self,block):
        chain = self.chain
        assert(block.type == Block.BRANCH2)
        a,i,n,b = block.a,block.i,block.n,block.b
        p,q = block.p,block.q
        AC = np.vstack([chain.jointAngleDerivative(b,n),chain.jointAngleDerivative(b,b)]).T
        dnb = np.dot(np.linalg.pinv(AC),chain.jointAngleDerivative(b,i))
        sumsq = np.dot(dnb,dnb)
        for (pi,qi) in zip(p,q):
            E = np.vstack([chain.jointAngleDerivative(qi,pi),chain.jointAngleDerivative(qi,qi)]).T
            D = chain.jointAngleDerivative(qi,n)
            dnq = np.dot(np.linalg.pinv(E),chain.jointAngleDerivative(b,i) - D)
            sumsq += np.dot(dnq,dnq)
        if sumsq < 1e-6: return 1e-3
        return math.sqrt(sumsq)

    def branch2BlockLogImportance(self,block):
        """Returns the log-importance weight w'
          log P(x[i,n],theta[i,n,b,c]|x[a],x[b]) -
          log Q(x[i,n],theta[i,n,b,c]|x[a],x[b])
        to be used in importance sampling. """
        
        chain = self.chain
        assert(block.type == Block.BRANCH2)
        a,i,n,b = block.a,block.i,block.n,block.b
        p,q = block.p,block.q
        pt0 = 0
        pt1 = chain.thetaLogProbability(n)
        pt2 = chain.thetaLogProbability(b)
        pt3 = sum(chain.thetaLogProbability(c) for c in chain.children[b])
        px1 = chain.xLogProbability(i)
        px2 = chain.xLogProbability(n)
        psum = pt0 + pt1 + pt2 + pt3 + px1 + px2
        for (pi,qi) in zip(p,q):
            psum += chain.thetaLogProbability(pi) +chain.thetaLogProbability(qi)
            psum += chain.xLogProbability(pi)
            for c in chain.children[qi]:
                psum += chain.thetaLogProbability(c)
        try:
            return psum + math.log(self.branch2BlockMetricSqrt(block))+self.sideChainLogImportance(block)
        except ValueError:
            #domain error at singularity
            return 1e308

    def branch2BlockMetropolis(self,block):
        """Given a straight block a->i->n->b, samples x[i] and x[n] given
        x[a], x[b] up to maxSamplesPerBlock times in order to get a
        metropolis sample.
        """
        chain = self.chain
        assert(block.type == Block.BRANCH2)
        a,i,n,b = block.a,block.i,block.n,block.b
        p,q = block.p,block.q

        R = distance(chain.x[a],chain.x[b])
        L = chain.L[i]+chain.L[n]+chain.L[b]
        if abs(R-L)<1e-5:
            #straightened out, only 1 solution
            print "Gibbs step: straightened out"
            return True
        if R > L:
            print "Gibbs step: points too far away",R,">",L
            return False
        for pi,qi in zip(p,q):
            R = distance(chain.x[a],chain.x[qi])
            L = chain.L[i]+chain.L[n]+chain.L[pi]+chain.L[qi]
            if abs(R-L)<1e-5:
                #straightened out, only 1 solution
                print "Gibbs step: straightened out"
                return True
            if R > L:
                print "Gibbs step: points too far away",R,">",L
                return False
            
        #in case the sampling fails, revert to old
        xold = [chain.x[index] for index in block.activeIndices+block.side]
            
        oldlogw = self.branch2BlockLogImportance(block)
        bestnewlogw = -1e100
        for iters in xrange(self.maxSamplesPerBlock):
            if self.branch2BlockSample(block):
                if not chain.blockFeasible(block):
                    print "  Not a feasible Branch2"
                    continue
                newlogw = self.branch2BlockLogImportance(block)
                if newlogw > bestnewlogw:
                    bestnewlogw=newlogw
                alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
                if random.random() <= alpha:
                    #accept move 
                    print "  Old Importance:\t" + str(oldlogw)
                    print "  New Importance (accept):\t" + str(newlogw) 
                    return True

        print "  Old Importance:\t" + str(oldlogw)
        print "  Best rejected importance:\t" + str(bestnewlogw) 


        #restore to old pristine copy
        for index,val in zip(block.activeIndices+block.side,xold):
            chain.x[index] = val
        return False

    def printBlocks(self,blocks=None):
        names = ['straight','branch1','branch2','openstart','openend','openend2','invalid']
        if blocks==None:
            blocks = []
            for i in range(len(self.chain.x)):
                try:
                    block = Block(i,self.chain)
                    print i,':',names[block.type]
                    blocks.append(block)
                except Exception as e:
                    print i,': failed',e
        else:
            for block in blocks:
                print block.i,':',names[block.type]

        #debugging
        coverage = [0]*len(self.chain.x)
        for block in blocks:
            if block.type == Block.STRAIGHT or block.type == Block.BRANCH2 or block.type == Block.OPENEND:
                for k in block.activeIndices:
                    coverage[k] += 1
        print "Joint coverage",coverage
        free = [True]*len(self.chain.x)
        for (i,x) in self.chain.fixedPositions:
            free[i] = False
        for (i,f) in enumerate(free):
            if not f and coverage[i] != 0:
                print "WARNING: fixed position %d is covered by a block"%(i,)
            if f and coverage[i] == 0:
                print "WARNING: free position %d is covered by a block"%(i,)
        return

    def gibbsPassMetropolisBlock(self, i):
        """Passes a Gibbs step through all links in the given order"""
        print "-----gibbsPassMetropolis----"
        try:
            block = Block(i,self.chain)
            if block.type == Block.STRAIGHT:
                print "STARIGHT:\t" + str(block.a) + " " + str(block.i) + " " + str(block.n) + " " + str( block.b)
                self.straightBlockMetropolis(block)
            elif block.type == Block.BRANCH2:
                print "BRANCH2:\t" + str(block.a) + " " + str(block.i) + " " + str(block.n) + " " + str( block.b)
                self.branch2BlockMetropolis(block)
            elif block.type == Block.OPENEND:
                print "OPENEND:\t" + str(block.a) + " " + str(block.i)
                self.open_endBlockMetropolis(block)                 
            elif block.type == Block.OPENEND2:
                print "OPENEND2"
            elif block.type == Block.OPENSTART:
                print "OPENSTART"
            elif block.type == Block.BRANCH1:
                print "BRANCH1:\t" + str(block.a) + " " + str(block.i) + " " + str( block.n[0]) + " " + str( block.b[0])
                self.branch1BlockMetropolis(block)
            else:
                #TODO: different types of blocks
                print "ERROR BLOCK"
        except ValueError as e:
            print e
            pass

    def gibbsPassMetropolis(self,order=None):
        """Passes a Gibbs step through all links in the given order"""
        print "-----gibbsPassMetropolis----"
        if order == None:
            order = range(len(self.chain.x))
        for i in order:
            try:
                block = Block(i,self.chain)
                if block.type == Block.STRAIGHT:
                    #print "STARIGHT:\t" + str(block.a) + " " + str(block.i) + " " + str(block.n) + " " + str( block.b)
                    self.straightBlockMetropolis(block)
                elif block.type == Block.BRANCH2:
                    print "BRANCH2:\t" + str(block.a) + " " + str(block.i) + " " + str(block.n) + " " + str( block.b)
                    self.branch2BlockMetropolis(block)
                elif block.type == Block.OPENEND:
                    #print "OPENEND:\t" + str(block.a) + " " + str(block.i)
                    self.open_endBlockMetropolis(block)                 
                elif block.type == Block.OPENEND2:
                    #print "OPENEND2"
                    pass
                elif block.type == Block.OPENSTART:
                    #print "OPENSTART"
                    pass
                elif block.type == Block.BRANCH1:
                    print "BRANCH1:\t" + str(block.a) + " " + str(block.i) + " " + str( block.n[0]) + " " + str( block.b[0])
                    self.branch1BlockMetropolis(block)
                    pass
                else:
                    #TODO: different types of blocks
                    print "ERROR BLOCK"
                    continue
            except ValueError as e:
                print e
                pass
#            print block.type
        return

def autocorrelation(sequence,h,total=False):
    if h*10 > len(sequence):
        raise ValueError("TODO: correct autocorrelation when h is relatively large compared to sample size")
    mean = sum(sequence)/len(sequence)
    var = sum((s-mean)**2 for s in sequence)/len(sequence)
    cov = 0
    for x0,xh in zip(sequence[:-h],sequence[h:]):
        cov += (x0-mean)*(xh-mean)
        
    cov /= len(sequence)
    if total:
        return sum(cov)/sum(var) if sum(var) != 0.0 else 0.0
    return [ci/vi if vi!=0.0 else 0.0 for ci,vi in zip(cov,var)]


def build_default_chain(n=4,theta=math.pi/3,thetapriorstd=0.0):
    thetaprior = None
    if thetapriorstd>0.0:
        thetaprior = AngularGaussianDistribution(0,thetapriorstd)
    chain = Branched2DChain()
    chain.startBuild(np.array([0.0,0.0]))
    dx = np.array([math.cos(theta),math.sin(theta)])
    dx2 = np.array([math.cos(theta),-math.sin(theta)])
    chain.add(1.0,dx,None,None)
    for i in xrange(1,n/2):
        chain.add(1.0,dx,thetaprior,None)
    if n%2 == 1:
        chain.add(1.0,np.array([1.0,0.0]),thetaprior,None)
        chain.add(1.0,dx2,thetaprior,None)
    else:
        chain.add(1.0,dx2,thetaprior,None)
    for i in xrange(len(chain.x)-1,n):
        chain.add(1.0,dx2,thetaprior,None)

    """
    size = 0.25
    #place middle joint in center
    chain.xPriors[(n+1)/2] = GaussianDistribution([chain.x[(n+1)/2][0],-chain.x[(n+1)/2][1]],[size,size])
    #place all joints in place
    for i in xrange(len(chain.x)):
        chain.xPriors[i] = GaussianDistribution([chain.x[i][0],chain.x[i][1]],[size,size])
    """
    #straight chain
    """
    for i in xrange(n+1):
        chain.xPriors[i] = GaussianDistribution([i,0.0],[size,size])
    """
    #squished chain
    """
    for i in xrange(n+1):
        chain.xPriors[i] = GaussianDistribution([i/2,0.0],[size,size])
    """
    #circle chain
    """    
    size = 0.1
    rad = n*0.5/math.pi/math.cos(2.0*math.pi/float(2*(n+1)))
    for i in xrange(n+1):
        u = float(i)/float(n+1)*2.0*math.pi
        chain.xPriors[i] = GaussianDistribution([rad*(1.0-math.cos(u)),rad*math.sin(u)],[size/(i+1),size/(i+1)])
    """
    #sawtooth chain
    """
    for i in xrange(n+1):
        if i % 2 == 0:
            chain.xPriors[i] = GaussianDistribution([math.cos(theta)*i,0],[size,size])
        else:
            chain.xPriors[i] = GaussianDistribution([math.cos(theta)*i,math.sin(theta)],[size,size])
    """

    chain.closeEndpoints()
    assert len(chain.x) == n+1
    #chain.closeChainSample(len(chain.x)-2)
    return chain

def build_branched_chain(n=3,armlength=2,thetapriorstd=0.0):
    thetaprior = None
    if thetapriorstd>0.0:
        thetaprior = AngularGaussianDistribution(0,thetapriorstd)
    chain = Branched2DChain()
    chain.startBuild(np.array([0.0,0.0]))
    for i in xrange(n):
        theta = (math.pi*2.0*i)/n
        dx = np.array([math.cos(theta),math.sin(theta)])
        chain.add(1.0,dx,None,None,parent=0)
        dx2 = np.array([-math.sin(theta),math.cos(theta)])
        for j in xrange(armlength-1):
            if j%2 == 1:
                chain.add(1.0,dx,thetaprior,None,parent='last')
            else:
                chain.add(1.0,dx2,thetaprior,None,parent='last')
    print chain.parents
    print chain.children
    chain.reroot(armlength)
    print chain.parents
    print chain.children
    chain.closeEndpoints()
    return chain

def build_humanoid_chain(thetapriorstd=0.0):    
    armlength = 1
    leglength = 1.5

    chain = Branched2DChain()
    
    #base: 0
    chain.startBuild( np.array([0.0,-2.91421356]))      
    #trunk: 1
#    bodyPrior = JointConstraint( math.pi / 4.0, math.pi * 5.0 / 180.0, -math.pi / 18, math.pi / 18)
    theta = math.pi / 2.0;
    dx = np.array( [armlength*math.cos(theta),armlength*math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 'last')    
    #trunk: 2
#    bodyPrior = JointConstraint( 0.0, math.pi * 5.0 / 180.0, -math.pi / 18, math.pi / 18)
    theta = math.pi / 2.0;
    dx = np.array( [armlength*math.cos(theta),armlength*math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 'last')
    #head: 3
#    bodyPrior = JointConstraint( 0.0, math.pi * 5.0 / 180.0, -math.pi / 6, math.pi / 6)
    theta = math.pi / 2.0;
    dx = np.array( [armlength*math.cos(theta),armlength*math.sin(theta)])
    chain.add( armlength, dx, None, None, parent='last')
        
    #left hip: 4
#    hipPrior_left  = JointConstraint( 0, math.pi * 20.0 / 180.0, -math.pi, math.pi)
    theta4 = 2.35619448851
    dx = np.array( [armlength*math.cos(theta4),armlength*math.sin(theta4)])
    chain.add( armlength, dx, None, None, parent = 0)
    
    #left legs: 5
#    leg_prior = JointConstraint(0, math.pi * 20.0 / 180.0, -math.pi, math.pi)
    theta = math.pi * 210/180.0;
    dx = np.array([leglength*math.cos(theta),leglength*math.sin(theta)])
#    chain.add( armlength, dx, None, None, parent = 'last')
    chain.add( leglength, dx, None, None, parent = 'last')
    
    #left legs: 6
    theta = math.pi * 330/180.0;
    dx = np.array([leglength*math.cos(theta),leglength*math.sin(theta)])
    chain.add( leglength, dx, None, None, parent = 'last')
          
    #right hip: 7
#    hipPrior_right  = JointConstraint( -math.pi / 2.0, math.pi * 2.0 / 180.0, -math.pi, math.pi)
    theta = math.pi - theta4
    dx = np.array( [armlength*math.cos(theta),armlength*math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 0)
    
    #right legs: 8
    theta = -math.pi * 30/180.0
    dx = np.array([leglength*math.cos(theta),leglength*math.sin(theta)])
    chain.add( leglength, dx, None, None, parent = 'last') 
    #right legs: 9
    theta = math.pi * 210/180.0
    dx = np.array([leglength*math.cos(theta),leglength*math.sin(theta)])
    chain.add( leglength, dx, None, None, parent = 'last') 
           
    #left shoulder: 10
    theta = math.pi*5.0/6.0
    dx = np.array( [armlength * math.cos(theta), armlength * math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 2)
    
    #left elbow: 11
    theta = math.pi * 1.25;
    dx = np.array( [armlength * math.cos(theta), armlength * math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 'last')
    
    #left hand: 12
    theta = math.pi * 1.5;
    dx = np.array( [armlength * math.cos(theta), armlength * math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 'last')
    
    #right shoulder: 13
    theta = math.pi*1.0/6.0
    dx = np.array([armlength * math.cos(theta),armlength * math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 2)
    
    #right elbow: 14
#    theta = math.pi * 1.75;
    theta = -math.pi * 1.75
    dx = np.array([armlength * math.cos(theta), armlength * math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 'last')
    
    #right hand: 15
#    theta = math.pi * 0.25;
    theta = -math.pi * 1.5
    dx = np.array([armlength * math.cos(theta), armlength * math.sin(theta)])
    chain.add( armlength, dx, None, None, parent = 'last')        
     
######################## Priors
    #1
    bodyPrior = JointConstraint( math.pi / 4.0, math.pi * 30.0 / 180.0, -math.pi/4.0, math.pi*3.0/4.0)
    chain.thetaPriors[1] = bodyPrior
    #2
    bodyPrior = JointConstraint( 0.0, math.pi * 10.0 / 180.0, -1.0 / 3.0 * math.pi, 1.0 / 3.0 * math.pi)    
    chain.thetaPriors[2] = bodyPrior
    #3
    bodyPrior = JointConstraint( 0.0, math.pi * 10.0 / 180.0, -1.0 / 3.0 * math.pi, 1.0 / 3.0 * math.pi)
    chain.thetaPriors[3] = bodyPrior
    #4
    hipPrior_left = JointConstraint( 0, math.pi * 5.0 / 180.0, -math.pi, 1.0 / 4.0 * math.pi)    
    chain.thetaPriors[4] = hipPrior_left
    #5: TODO:
    leg_prior = JointConstraint( math.pi / 2, math.pi * 5.0 / 180.0, -math.pi, math.pi)
    chain.thetaPriors[5] = leg_prior
    #6
    chain.thetaPriors[6] = None
    #7
    hipPrior_right  = JointConstraint( -math.pi / 2.0, math.pi * 10.0 / 180.0, -2.0 / 3.0 * math.pi, 2.0 / 3.0 * math.pi)
#    hipPrior_right  = JointConstraint( 0, math.pi * 5.0 / 180.0, -2 * math.pi, 2 * math.pi)
    chain.thetaPriors[7] = hipPrior_right    
    #8
    leg_prior = JointConstraint( -math.pi / 4.0, math.pi * 5.0 / 180.0, -2.0 / 3.0 * math.pi, 1.0 / 2.0 * math.pi)
    chain.thetaPriors[8] = leg_prior
    #9
    leg_prior = JointConstraint( 0.0, math.pi * 5.0 / 180.0, -math.pi, math.pi/4.0)
    chain.thetaPriors[9] = leg_prior    
    #10
    shoulderPrior_left = JointConstraint( math.pi / 2.0, math.pi * 5.0 / 180.0, 1.0 / 6.0 * math.pi, 3.0 / 4.0 * math.pi)
    chain.thetaPriors[10]= shoulderPrior_left
    #11
#    elbowPrior = JointConstraint( math.pi / 2.0, math.pi * 10.0 / 180.0, -math.pi, math.pi)   
    elbowPrior = JointConstraint( 0, math.pi * 45.0 / 180.0, -1.5 / 2.0*math.pi, 1.5 / 2.0 * math.pi)   
    chain.thetaPriors[11] = elbowPrior
    #12
    handPrior = JointConstraint( 0.0, math.pi * 45.0 / 180.0, -math.pi, math.pi)
    chain.thetaPriors[12] = handPrior
    #13
    shoulderPrior_right = JointConstraint( -math.pi / 2, math.pi * 5.0 / 180.0, -3.0 / 4.0 * math.pi, -1.0 / 6.0 * math.pi)
    chain.thetaPriors[13] = shoulderPrior_right
    #14
    elbowPrior_right = JointConstraint( 0, math.pi * 45.0 / 180.0, -1.5 / 2.0 * math.pi, 1.5 / 2.0*math.pi)
#   elbowPrior_right = JointConstraint( -math.pi / 2.0, math.pi * 10.0 / 180.0, -math.pi, math.pi)
    chain.thetaPriors[14] = elbowPrior_right
    #15
    handPrior = JointConstraint( 0.0, math.pi * 45.0 / 180.0, -math.pi, math.pi)
    chain.thetaPriors[15] = handPrior

######################## Reroot         
    chain.reroot(6)
    bodyPrior = JointConstraint( -math.pi / 4.0, math.pi * 10.0 / 180.0, -2.0 / 3.0 * math.pi, 1.0/4.0 * math.pi)
    chain.thetaPriors[0] = bodyPrior
    thetamean = [prior.mean if prior!=None else None for prior in chain.thetaPriors]
    #chain.forwardKinematics(6,thetamean)
    
    print "Theta Priors:"
    for i in range(len(chain.thetaPriors)):
        has = "YES" if chain.thetaPriors[i] != None else "NO"
        print str(i) + "\t" + has
        if chain.thetaPriors[i] != None and not chain.thetaPriors[i].isFeasible(chain.getTheta(i)):
            print "Warning: initial theta",i,"=",chain.getTheta(i)," infeasible"

    #Here, we set the positional prior to point 0
    mean = [ 0 ]
    std = [ 1.0]
    comPrior = CenterOfMassConstraint( mean, std, -0.8, 0.8)
    chain.xPriors[0] = comPrior
#    #1
#    mean = [ 0, 1]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -0.8, 0.8)
#    chain.xPriors[1] = comPrior
#    #2
#    mean = [ 0, 2]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -0.8, 0.8)
#    chain.xPriors[2] = comPrior
#    #3
#    mean = [ 0, 3]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -0.8, 0.8)
#    chain.xPriors[3] = comPrior
#    for i in range( len( chain.x)):
#        chain.thetaPriors[i] = None
#    #10
#    mean = [ -1, 2.5]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -0.8, 0.8)
#    chain.xPriors[10] = comPrior
#    #13
#    mean = [ 1, 2.5]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -0.8, 0.8)
#    chain.xPriors[13] = comPrior
    #12
#    mean = [ -2.866, 2.5]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -3.8, 3.8)
#    chain.xPriors[12] = comPrior
#    #15
#    mean = [ 2.866, 2.5]
#    std = [ 0.1, 0.1]
#    comPrior = CenterOfMassConstraint( mean, std, -3.8, 3.8)
#    chain.xPriors[15] = comPrior
    
    
    print chain.parents
    print chain.children
    fixed = [6,9,15]
    chain.fixedPositions = [(f,chain.x[f]) for f in fixed]
    
#    print "Priors:"
#    #print len(chain.thetaPriors)
#    for i in range( len(chain.thetaPriors)):
#        if chain.thetaPriors[i] != None:
#            print str(i) + ":" + str(chain.thetaPriors[i].mean) + "\t" + str(chain.thetaPriors[i].std)
#        else:
#            print str(i) + ": NONE"
            
    return chain

class ChainApp(Tkinter.Frame):
	def __init__(self, master=None):
		Tkinter.Frame.__init__(self, master)
		self.pack_propagate(0)
		self.canvas =  None
		self.num_angles = 3
                self.theta_stddev = 0.0
                self.selected_index = 1
                self.samples = []
                self.make_chain()
                self.i = 0
		self.pack()
		# always make_buttons(), make_canvas() in order!
                self.make_buttons()
		self.make_canvas()
		self.redraw()
	
	def update(self):
            self.redraw()

        def toscreen(self,pt):
            w = HEIGHT/len(self.chain.x)/0.9
            h = HEIGHT/len(self.chain.x)/0.9
            ofsx = len(self.chain.x)*0.25*w*WIDTH/HEIGHT
            ofsy = 0.5*HEIGHT
            return [int(pt[0]*w+ofsx),int(-pt[1]*h+ofsy)]

        def draw_chain(self):
            if self.drawlinks.get()==1:
                for i,pt in enumerate(self.chain.x):
                    if self.chain.parents[i] >= 0:
                        line = self.toscreen(self.chain.x[self.chain.parents[i]])+self.toscreen(pt)
                        self.canvas.create_line(line)
            colors = ['red','orange','yellow','green','blue','purple','brown','gray']
            r = 3
            for i,pt in enumerate(self.chain.x):
                p = self.toscreen(pt)
		self.canvas.create_oval(p[0]-r,p[1]-r,p[0]+r,p[1]+r, fill=colors[i%len(colors)], outline='black')

        def score(self,x):
            self.chain.setState(x)
            lp = 0.0
            for i in xrange(1,len(self.chain.x)):
                if self.chain.thetaPriors[i]!=None:
                    lp += self.chain.thetaPriors[i].log_probability(self.chain.getTheta(i))
                if self.chain.xPriors[i]!=None:
                    lp += self.chain.xPriors[i].log_probability(self.chain.x[i])
            return -lp

        def draw_priors(self):
            colors = ['red','orange','yellow','green','blue','purple','brown','gray']
            sigma = 3.0
            for i,prior in enumerate(self.chain.xPriors):
                if prior != None and len(prior.mean)==2:
                    ll = self.toscreen([m-s*sigma for m,s in zip(prior.mean,prior.std)])
                    ur = self.toscreen([m+s*sigma for m,s in zip(prior.mean,prior.std)])
                    self.canvas.create_oval(ll[0],ll[1],ur[0],ur[1], fill=None, outline='black')
            for i,prior in enumerate(self.chain.thetaPriors):
                if prior != None:
                    assert self.chain.parents[i] >= 0
                    theta0 = self.chain.getAbsTheta(self.chain.parents[i])
                    angle = theta0+prior.mean
                    bounds = [theta0+prior.v_min,theta0+prior.v_max]
                    center=self.toscreen(self.chain.x[self.chain.parents[i]])
                    end = self.toscreen(self.chain.x[self.chain.parents[i]]+np.array([math.cos(angle),math.sin(angle)])*0.5)
                    bndmin = self.toscreen(self.chain.x[self.chain.parents[i]]+np.array([math.cos(bounds[0]),math.sin(bounds[0])])*0.25)
                    res = 0.15
                    middle = []
                    theta = bounds[0]+res
                    while theta < bounds[1]:
                        middle +=  self.toscreen(self.chain.x[self.chain.parents[i]]+np.array([math.cos(theta),math.sin(theta)])*0.25)
                        theta += res
                    bndmax = self.toscreen(self.chain.x[self.chain.parents[i]]+np.array([math.cos(bounds[1]),math.sin(bounds[1])])*0.25)
                    self.canvas.create_polygon(center+bndmin+middle+bndmax,fill=colors[i%len(colors)],outline=None)
                    self.canvas.create_line(center+end)


	def redraw(self):
            self.canvas.delete(Tkinter.ALL)
            self.canvas.create_rectangle(2,2,WIDTH,HEIGHT, fill='white', outline='black')
            if self.drawpriors.get()==1:
                self.draw_priors()
            if self.drawall.get()==1:
                oldx = self.chain.getState()
                if self.drawbest.get()==1:
                    scores = [(self.score(x),x) for x in self.samples]
                    num = min(len(self.samples),20)
                    for s,x in sorted(scores)[:num]:
                        self.chain.setState(x)
                        self.draw_chain()
                else:
                    for x in self.samples:
                        self.chain.setState(x)
                        self.draw_chain()
                self.chain.setState(oldx)
            else:
                self.draw_chain()

        def toggle_draw_all(self):
            self.redraw()

        def toggle_draw_links(self):
            self.redraw()

        def toggle_draw_best(self):
            self.redraw()

        def toggle_draw_priors(self):
            self.redraw()

        def toggle_fix_endpoints(self):
            if self.fix_endpoints.get()==0:
                self.chain.openEndpoints()
            else:
                self.chain.closeEndpoints()
			
	def make_chain(self):
            #self.chain = build_default_chain(self.num_angles,math.pi/3,self.theta_stddev)
#            self.chain = build_branched_chain(self.num_angles,5,self.theta_stddev)
            self.chain = build_humanoid_chain(self.theta_stddev)
            gibbs = Gibbs2DChainSampler(self.chain)
            self.samples = [self.chain.getState()]

        def reset(self):
            try:
                self.num_angles = int(self.anglesentry.get())
                if self.num_angles < 3:
                    self.num_angles = 3
            except ValueError:
                pass
            try:
                self.theta_stddev = float(self.stdentry.get())
            except ValueError:
                pass
            self.make_chain()
            self.redraw()

        def sample_loopclose(self):
            #print "Sampling with the RLG"
            #rlg = RLG2DChainSampler(self.chain)
            #rlg.sample()
            #self.samples.append(self.chain.getState())
            #return

            print "Sampling with Newton-Raphson loop closure"
            nr = NewtonRaphson2DChainSampler(self.chain)
            for step in xrange(100):
                if nr.sample():
                    self.samples.append(self.chain.getState())
                    break
            return

        def sample_atom(self):
            #HACK: use loop closure method
            numsamples = 1
            try: numsamples = int(self.numsamplesentry.get())
            except: pass
            t0 = time.time()
            for sample in xrange(numsamples):
                print sample
                self.sample_loopclose()
            t1 = time.time()
            print "Sampling time",t1-t0
            self.redraw()
            return

            #sample just the given atom
            numskip = 0
            try: numskip = int(self.skipentry.get())
            except: pass

            t0 = time.time()
            gibbs = Gibbs2DChainSampler(self.chain)
            for sample in xrange(numsamples):
                for i in xrange(numskip):
                    gibbs.block2Metropolis(self.selected_index)
                gibbs.block2Metropolis(self.selected_index)
                self.samples.append(self.chain.getState())
            t1 = time.time()
            print "Sampling time",t1-t0
            self.redraw()

        def sample_metropolis_basic(self):
            numskip = 0
            try: numskip = int(self.skipentry.get())
            except: pass
            numsamples = 1
            try: numsamples = int(self.numsamplesentry.get())
            except: pass

            #makes sure endpoints are open
            oldendpoints = self.chain.fixedPositions
            self.chain.openEndpoints()
            
            t0 = time.time()
            sampler = Metropolis2DChainSampler(self.chain)
            #sampler.setSampleRandom()
            for sample in xrange(numsamples):
                for i in xrange(numskip):
                    sampler.sample()
                sampler.sample()
                self.samples.append(self.chain.getState())
            t1 = time.time()
            print "Sampling time",t1-t0

            #restore endpoint status
            self.chain.fixedPositions = oldendpoints
            #self.chain.closeEndpoints()
            
            self.redraw()
            
        def sample(self):
            #normal order
#            order = None
            order = [5, 4, 0, 1, 2, 3, 10, 13, 7, 12, 15]
            #reverse order
            #order = [i for i in reversed(range(1,len(self.chain.x)-2))]
            #skipping 2 order
            #order = range(1,len(self.chain.x)-2,2)
            #if order[-1] != len(self.chain.x)-3:
            #    order.append(len(self.chain.x)-3)
            numskip = 0
            try: numskip = int(self.skipentry.get())
            except: pass
            numsamples = 1
            try: numsamples = int(self.numsamplesentry.get())
            except: pass

            t0 = time.time()
            gibbs = Gibbs2DChainSampler(self.chain)
            gibbs.printBlocks([Block(i,self.chain) for i in order])
            for sample in xrange(numsamples):
                for i in xrange(numskip):
                    gibbs.gibbsPassMetropolis(order)
                gibbs.gibbsPassMetropolis(order)
                self.samples.append(self.chain.getState())
            t1 = time.time()
            print "Sampling time",t1-t0
            self.redraw()

        def sampleBlock(self):
            #normal order
#            order = None
            order = [5, 4, 0, 1, 3, 10, 13, 7, 12, 15]
            #reverse order
            #order = [i for i in reversed(range(1,len(self.chain.x)-2))]
            #skipping 2 order
            #order = range(1,len(self.chain.x)-2,2)
            #if order[-1] != len(self.chain.x)-3:
            #    order.append(len(self.chain.x)-3)
            numskip = 0
            try: numskip = int(self.skipentry.get())
            except: pass
            numsamples = 1
            try: numsamples = int(self.numsamplesentry.get())
            except: pass

            t0 = time.time()
            gibbs = Gibbs2DChainSampler(self.chain)
            
            for sample in xrange(numsamples):
                for i in xrange(numskip):
                    gibbs.gibbsPassMetropolisBlock( order[self.i])
                gibbs.gibbsPassMetropolisBlock( order[self.i])
                self.samples.append(self.chain.getState())
            t1 = time.time()
            print "Sampling time",t1-t0
            self.redraw()
            self.i += 1
            self.i = self.i % len( order)
            
        def clear_samples(self):
            self.samples = []
            self.redraw()
	
	def make_buttons(self):
            # Control buttons
            self.angleslabel = Tkinter.Label(self, text="Number of links:")
            self.anglesentry = Tkinter.Entry(self)
            self.anglesentry.insert(Tkinter.END,str(self.num_angles))

            self.stdlabel = Tkinter.Label(self, text="Theta stddev:")
            self.stdentry = Tkinter.Entry(self)
            self.stdentry.insert(Tkinter.END,str(self.theta_stddev))
            
            self.resetbutton = Tkinter.Button(self, text='Reset', command=self.reset)
            self.numsampleslabel = Tkinter.Label(self, text="Number to sample:")
            self.numsamplesentry = Tkinter.Entry(self)
            self.numsamplesentry.insert(Tkinter.END,'1')
            self.skiplabel = Tkinter.Label(self, text="Number to skip:")
            self.skipentry = Tkinter.Entry(self)
            self.skipentry.insert(Tkinter.END,'0')
            self.samplebutton = Tkinter.Button(self, text='Sample', command=self.sample)
            self.sample_selected_button = Tkinter.Button(self, text='Sample selected', command=self.sample_atom)
            self.sample_metropolis_button = Tkinter.Button(self, text='Sample basic metropolis', command=self.sample_metropolis_basic)
            self.fix_endpoints = Tkinter.IntVar(0)
            self.fix_endpoints.set(1)
            self.fixendpointcheck = Tkinter.Checkbutton(self, variable=self.fix_endpoints, text='Fix endpoints', command=self.toggle_fix_endpoints)
            self.drawall = Tkinter.IntVar(0)
            self.drawallcheck = Tkinter.Checkbutton(self, variable=self.drawall, text='Draw all', command=self.toggle_draw_all)
            self.drawlinks = Tkinter.IntVar(0)
            self.drawlinks.set(1)
            self.drawlinkscheck = Tkinter.Checkbutton(self, variable=self.drawlinks, text='Draw links', command=self.toggle_draw_links)

            self.drawbest = Tkinter.IntVar(0)
            self.drawbestcheck = Tkinter.Checkbutton(self, variable=self.drawbest, text='Draw only best', command=self.toggle_draw_best)
            self.clear_samples_button = Tkinter.Button(self, text='Clear samples', command=self.clear_samples)

            self.drawpriors = Tkinter.IntVar(0)
            self.drawpriors.set(1)
            self.drawpriorscheck = Tkinter.Checkbutton(self, variable=self.drawpriors, text='Draw priors', command=self.toggle_draw_priors)

            # Pack it all into subframes and then into the main frame's grid
            self.guiitems = [self.angleslabel, self.anglesentry, self.stdlabel, self.stdentry, self.resetbutton, self.fixendpointcheck, self.numsampleslabel, self.numsamplesentry, self.skiplabel,self.skipentry, self.samplebutton, self.sample_selected_button, self.sample_metropolis_button, self.drawallcheck,self.drawlinkscheck,self.drawbestcheck,self.drawpriorscheck,self.clear_samples_button]
            for i,item in enumerate(self.guiitems):
                item.pack(side=Tkinter.LEFT)
                item.grid(column=0,row=i)
                
        def make_canvas(self):
            if not self.canvas:
                self.canvas = Tkinter.Canvas(self,bg='white',width=WIDTH,height=HEIGHT)
                self.canvas.grid(column=1,row=0,rowspan=len(self.guiitems))

def main_gui():
    root = Tkinter.Tk()
    app = ChainApp(root)
    root.mainloop()

main_gui()
