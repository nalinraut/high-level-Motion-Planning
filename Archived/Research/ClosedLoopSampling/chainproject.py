import math
import random
import numpy as np
import copy
import time
import Tkinter
import canvasvg
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

class Kinematic2DChain:
    def __init__(self):
        self.L =[]
        self.x = []
        self.theta = []
        self.cumtheta = []
        self.thetaPriors = []
        self.xPriors = []
        self.startPosition = None
        self.endPosition = None

    def getState(self):
        return copy.deepcopy((self.x,self.theta,self.cumtheta))

    def setState(self,state):
        (self.x,self.theta,self.cumtheta) = copy.deepcopy(state)

    def copy(self,chain):
        self.L = chain.L[:]
        self.x = copy.deepcopy(chain.x)
        self.theta = chain.theta[:]
        self.cumtheta = chain.cumtheta[:]
        self.thetaPriors = copy.deepcopy(chain.thetaPriors)
        self.xPriors = copy.deepcopy(chain.xPriors)
        self.startPosition = copy.copy(chain.startPosition)
        self.endPosition = copy.copy(chain.endPosition)

    def startBuild(self,x):
        self.L=[None]
        self.x=[x];
        self.theta=[None]
        self.cumtheta = [0.0]
        self.thetaPriors=[None]
        self.xPriors=[None]
        self.startPosition = x
    
    def add(self,L,theta,thetaPrior,xPrior):
        """Adds a new link"""
        self.L.append(L)
        self.theta.append(theta)
        self.cumtheta.append(self.cumtheta[-1]+theta)
        dx = np.array([math.cos(self.cumtheta[-1]),math.sin(self.cumtheta[-1])])*L
        self.x.append(self.x[-1]+dx)
        self.thetaPriors.append(thetaPrior)
        self.xPriors.append(xPrior)

    def endBuild(self,x=None):
        """Sets the terminal position"""
        self.endPosition = x

    def openEndpoints(self):
        self.startPosition = self.endPosition = None

    def closeEndpoints(self):
        self.startPosition = self.x[0]
        self.endPosition = self.x[-1]

    def forwardKinematics(self,root=0):
        """Fills out x's and cumtheta from x[root] and theta's in increasing
        index order"""
        for i in xrange(root+1,len(self.theta)):
            self.cumtheta[i] = self.cumtheta[i-1]+self.theta[i]
            dx = np.array([math.cos(self.cumtheta[i]),math.sin(self.cumtheta[i])])*self.L[i]
            self.x[i] = self.x[i-1]+dx
        return

    def backwardKinematics(self,root=-1):
        """Fills out x's and cumtheta from x[root] and theta's in decreasing
        index order.  NOT TESTED YET"""
        if root < 0: root = root+len(self.theta)
        if root == 0: return
        for i in xrange(root-1,-1,-1):
            self.cumtheta[i] = self.cumtheta[i+1]-self.theta[i+1]
            dx = np.array([math.cos(self.cumtheta[i+1]),math.sin(self.cumtheta[i+1])])*self.L[i+1]
            self.x[i] = self.x[i+1]-dx
        self.theta[1] = self.cumtheta[1]
        self.cumtheta[0] = 0.0
        return

    def jacobian(self,pt=-1):
        """Fills out the jacobian of the chain endpoint (default)
        or the point indexed by pt, with respect to theta"""
        if pt < 0: pt = len(self.x)+pt
        J = np.zeros((2,len(self.theta)))
        for i in xrange(1,pt+1):
            dx = self.x[pt]-self.x[i-1]
            J[0,i] = -dx[1]
            J[1,i] = dx[0]
        return J

    def metricTensor(self,pt=-1):
        """Returns the value JJ^T for J = the Jacobian of the given
        point"""
        if pt < 0: pt = len(self.x)+pt
        JJt = np.zeros((2,2))
        for i in xrange(0,pt):
            dx = self.x[pt]-self.x[i]
            JJt[0,0] += dx[1]**2
            JJt[1,1] += dx[0]**2
            JJt[0,1] -= dx[0]*dx[1]
            JJt[1,0] -= dx[1]*dx[0]
        return JJt

    def ikSolutions(self,index):
        """Returns the two IK solutions for x[index] given x[index-1] and
        x[index+1], or None if there are no solutions.  The result is a
        pair of results [x[index],cumtheta[index],cumtheta[index+1]].
        
        If there are an infinite number of solutions, this samples k
        sample solutions (k=10 right now)"""
        dx = self.x[index+1]-self.x[index-1]
        L1 = self.L[index]
        L2 = self.L[index+1]
        d2 = np.dot(dx,dx)
        if d2 > (L1+L2)**2:
            return None
        d = math.sqrt(d2)
        if d < 1e-10:
            if abs(L1-L2) < 1e-10:
                #infinite number of solutions! sample a few
                res = []
                for i in xrange(10):
                    theta1 = random.uniform(0,math.pi*2)
                    theta2 = -theta1
                    dx = np.array([math.cos(theta1),math.sin(theta1)])*L1
                    x1 = self.x[index-1]+dx
                    res.append([x1,theta1,theta2])
                return res
            else:
                raise ValueError("no solutions")
        a = 0.5*(L1**2-L2**2)/d**2 + 0.5
        c2 = (L1/d)**2 - a**2
        if c2 < 0:
            raise ValueError("numerical error in ikSolutions?")
        c = math.sqrt(c2)
        x0 = self.x[index-1]+a*dx
        perp = np.array([-dx[1],dx[0]])
        x1 = x0+c*perp
        x2 = x0-c*perp
        assert(abs(distance(x1,self.x[index-1])-L1)<1e-6)
        assert(abs(distance(x2,self.x[index-1])-L1)<1e-6)
        assert(abs(distance(x1,self.x[index+1])-L2)<1e-6)
        assert(abs(distance(x2,self.x[index+1])-L2)<1e-6)
        #solve for angles
        theta1 = [angle(self.x[index-1],x1),angle(x1,self.x[index+1])]
        theta2 = [angle(self.x[index-1],x2),angle(x2,self.x[index+1])]
        return ([x1,theta1[0],theta1[1]],[x2,theta2[0],theta2[1]])

    def closeChainSample(self,i):
        """Finds the IK solutions for x[i] given x[i-1] and x[i+1], and
        sets the chain to one of those two solutions at random.

        If weight is true, then the position probability and of the adjacent
        theta variables are taken into account by doing a weighted sampling
        """
        solns = self.ikSolutions(i)
        if solns == None:
            raise ValueError("Chain can't be closed?")
        ind = random.randint(0,len(solns)-1)
        sol = solns[ind]
        self.x[i] = sol[0]
        self.theta[i] = anglesub(sol[1],self.cumtheta[i-1])
        self.theta[i+1] = anglesub(sol[2],sol[1])
        if i+2 < len(self.x):
            self.theta[i+2] = anglesub(self.cumtheta[i+2],sol[2])
        for j in xrange(i,i+2):
            self.cumtheta[j] = self.cumtheta[j-1]+self.theta[j]
        #sanity check: make sure cumtheta is still preserved
        if i+2 < len(self.x):
            assert(abs(anglesub(self.cumtheta[i+2],self.cumtheta[i+1]+self.theta[i+2])) < 1e-3)

    def xLogProbability(self,i):
        return self.xPriors[i].log_probability(self.x[i]) if self.xPriors[i] != None else 0.0 

    def thetaLogProbability(self,i):
        return self.thetaPriors[i].log_probability(self.theta[i]) if self.thetaPriors[i] != None else 0.0 

    def perturbGaussian(self,xradius,thetaradius,root=0):
        """Perturbs the conformation according to a gaussian magnitude,
        beginning at the root position"""
        self.x[root][0] += random.gauss(0.0,xradius)
        self.x[root][1] += random.gauss(0.0,xradius)
        for i in xrange(1,len(self.theta)):
            self.theta[i] += random.gauss(0.0,thetaradius)
        self.forwardKinematics(root)
        self.backwardKinematics(root)

    def sampleRandom(self,root=0):
        """Samples a random conformation, beginning at the root position"""
        assert(self.xPriors[root] != None)
        self.x[root] = self.xPriors[root].sample()
        for i in xrange(1,len(self.theta)):
            self.theta[i] = self.thetaPriors[i].sample() if self.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        self.forwardKinematics(root)
        self.backwardKinematics(root)

class Metropolis2DChainSampler:
    def __init__(self,chain):
        self.chain = chain
        #self.xRadius = 0.2
        #self.thetaRadius = 0.25
        self.xRadius = 0.02
        self.thetaRadius = 0.025
        self.maxSamples = 1

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
            if self.chain.startPosition != None:
                self.chain.x[0] = self.chain.startPosition
                self.chain.forwardKinematics()
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
                Jthetas.append(np.zeros((1,2+len(self.chain.theta)-1)))
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
        for i in xrange(1,len(self.chain.theta)):
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

def angleSampleRLG(joint,pttheta,ptr,target,Lremaining,
                   thetaPrior=None,maxSamples=100,discretizeThreshold=0.1):
    """For a joint centered at the point joint, samples an angle theta
    according to the prior such that the point pt (given in polar
    coordinates pttheta,ptr) rotated by theta is within Lremaining of the
    point target. If no prior is given, samples
    randomly"""
    #print "Sampling angle from",joint
    #print "Rotation point polar",pttheta,ptr
    #print "Target",target
    #print "Lremaining",Lremaining
    r0 = ptr
    D = distance(joint,target)
    if D+r0 <= Lremaining:
        #inside ball centered around target
        return thetaPrior.sample() if thetaPrior else random.uniform(0,2*math.pi)
    elif D > Lremaining+r0:
        raise ValueError("Chain out of reach")

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
    
    if thetaPrior==None:
        #print "Random sampling range",ta,trange
        return anglesub(ta + random.uniform(0,trange),pttheta)

    #we've got a prior
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


class RLG2DChainSampler:
    def __init__(self,chain):
        self.chain = chain
        self.usePrior = False
        self.maxPriorSamples = 1000

    def sample(self,usePrior=False):
        chain = self.chain
        assert (chain.startPosition != None and chain.endPosition != None)
        Li = [0]
        for l in chain.L[1:]:
            Li.append(Li[-1]+l)
        for i in xrange(1,len(chain.theta)-2):
            #intersect sphere centered at x[i-1] with radius chain.L[i]
            #and sphere centered at endPosition with radius Li[-1] - Li[i]
            rleft = Li[-1]-Li[i]
            chain.theta[i] = angleSampleRLG(chain.x[i-1],chain.cumtheta[i-1],chain.L[i],
                                            chain.endPosition,rleft,chain.thetaPriors[i] if self.usePrior else None)
            chain.cumtheta[i] = chain.cumtheta[i-1]+chain.theta[i]
            """
            r0 = chain.L[i]
            rleft = Li[-1]-Li[i]
            D = distance(chain.x[i-1],chain.endPosition)
            if D+r0 <= rleft:
                #inside ball centered around endPosition
                chain.theta[i] = chain.thetaPriors[i].sample() if self.usePrior and chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
                chain.cumtheta[i] = chain.cumtheta[i-1]+chain.theta[i]
            elif D > rleft+r0:
                raise ValueError("Chain out of reach at angle "+str(i))
            else:
                px = (D**2-rleft**2+r0**2)/(2*D)
                py = math.sqrt(r0**2-px**2)
                theta1 = math.atan2(py,px)
                theta2 = math.atan2(-py,px)
                thetaofs = angle(chain.x[i-1],chain.endPosition)
                
                if self.usePrior and chain.thetaPriors[i]!=None:
                    #rejection sample
                    ta = theta2+thetaofs
                    trange = 2*theta1
                    solved = False
                    for steps in xrange(self.maxPriorSamples):
                        theta = chain.thetaPriors[i].sample()
                        dtheta = anglesub(pttheta+theta,ta)
                        if dtheta >= 0 and dtheta <= trange:
                            chain.cumtheta[i] = theta
                            solved = True
                            break
                    if not solved:
                        #just sample uniformly
                        chain.cumtheta[i] = theta2+thetaofs + random.uniform(0,2*theta1)
                else:
                    chain.cumtheta[i] = theta2+thetaofs + random.uniform(0,2*theta1)
                chain.theta[i] = anglesub(chain.cumtheta[i],chain.cumtheta[i-1])
                """
            chain.x[i] = np.array([chain.x[i-1][0]+math.cos(chain.cumtheta[i])*chain.L[i],chain.x[i-1][1]+math.sin(chain.cumtheta[i])*chain.L[i]])
        #close the loop for the remaining 2 joints using analytical IK
        chain.closeChainSample(len(chain.theta)-2)

class NewtonRaphson2DChainSampler:
    def __init__(self,chain):
        self.chain = chain
        self.maxsteps = 50

    def sample(self):
        for i in xrange(1,len(self.chain.theta)):
            self.chain.theta[i] = self.chain.thetaPriors[i].sample() if self.chain.thetaPriors[i] != None else random.uniform(0,2*math.pi)
        return self.closeLoop(self.maxsteps)
    
    def closeLoop(self,maxsteps,ftol=1e-8,xtol=1e-8):
        """Returns 'f' if converged on f tolerance, 'x' if converged on x
        tolerance, False if failed to converge within maxsteps"""
        chain = self.chain
        assert (chain.startPosition != None and chain.endPosition != None)
        chain.forwardKinematics()
        err = chain.x[-1]-chain.endPosition
        f = np.dot(err,err)
        for step in xrange(maxsteps):
            #jacobian inverse search direction
            jinv = np.linalg.pinv(chain.jacobian())
            dtheta = -np.dot(jinv,err)

            #do a line search
            theta0 = chain.theta[:]
            alpha = 1.0
            steplen = abs(dtheta).max()
            reduced = False
            while alpha*steplen > xtol:
                for i in xrange(1,len(chain.theta)):
                    chain.theta[i] = theta0[i]+alpha*dtheta[i]
                chain.forwardKinematics()
                err = chain.x[-1]-chain.endPosition
                falpha = np.dot(err,err)
                if falpha < f:
                    f = falpha
                    reduced = True
                    break
                alpha *= 0.5
            if not reduced:
                chain.theta = theta0
                chain.forwardKinematics()
                return 'f'
            if f < ftol:
                #converged
                return 'x'
        return False

    
class Gibbs2DChainSampler:
    def __init__(self,chain):
        self.chain = chain
        self.maxSamplesPerBlock = 1

    def block2MetricSqrt(self,theta1,theta2,theta3,L1,L2,L3):
        """Returns the sqrt of the metric tensor for sampling a 3-link chain
        with absolute angles theta1,theta2,theta3 with link lengths L1,L2,L3"""
        #expression obtained through mathematica
        L1s,L2s,L3s=L1**2,L2**2,L3**2
        den = abs(math.sin(theta2-theta3)*math.sqrt(2)*L2*L3)
        if den  < 1e-5: den = 1e-5
        return math.sqrt(L1s*L2s + L1s*L3s + L2s*L3s + 2*L1s*L2s*L3s -
                         L1s*L2s*(1 + L3s)*math.cos(2*(theta1 - theta2)) -
                         L1s*L3s*math.cos(2*(theta1 - theta3)) - 
                         L2s*L3s*math.cos(2*(theta2 - theta3)) - 
                         L1s*L2s*L3s*math.cos(2*(theta2 - theta3))) /den;

    def startBlockSample(self):
        """Samples x[0], theta[1] according to the prior distribution on
        theta[1] while leaving the rest of the chain unchanged.
        """
        chain = self.chain
        assert chain.startPosition == None
        """
        theta1 = chain.thetaPriors[1].sample() if chain.thetaPriors[1]!=None else random.uniform(0,2*math.pi)
        chain.theta[1] = theta1
        chain.x[0] = chain.x[1]-np.array([math.cos(theta1),math.sin(theta1)])*chain.L[1]
        chain.cumtheta[1] = theta1
        if len(chain.theta) > 2:
            chain.theta[2] = anglesub(chain.cumtheta[2],chain.cumtheta[1])
        """
        theta1 = chain.thetaPriors[1].sample() if chain.thetaPriors[1]!=None else random.uniform(0,2*math.pi)
        theta2 = chain.thetaPriors[2].sample() if chain.thetaPriors[2]!=None else random.uniform(0,2*math.pi)
        chain.theta[1] = theta1
        chain.theta[2] = theta2
        chain.cumtheta[1] = theta1
        chain.cumtheta[2] = theta1+theta2
        chain.x[1] = chain.x[2]-np.array([math.cos(chain.cumtheta[2]),math.sin(chain.cumtheta[2])])*chain.L[2]
        chain.x[0] = chain.x[1]-np.array([math.cos(theta1),math.sin(theta1)])*chain.L[1]
        if len(chain.theta) > 3:
            chain.theta[3] = anglesub(chain.cumtheta[3],chain.cumtheta[2])
        return

    def startBlockLogImportance(self):
        """Returns the log-importance weight w'
          log P(x[0],theta[1],theta[2]|x[1],x[2]) - log Q(x[0],theta[1],theta[2]|x[1],x[2])
        to be used in importance sampling. """
        chain = self.chain
        """
        #pt1 = chain.thetaLogProbability(1)
        pt2 = chain.thetaLogProbability(2) if len(chain.theta)>2 else 0
        px0 = chain.xLogProbability(0)
        return pt2+px0
        """
        #pt1 = chain.thetaLogProbability(1)
        #pt2 = chain.thetaLogProbability(2)
        pt3 = chain.thetaLogProbability(3) if len(chain.theta)>2 else 0
        px0 = chain.xLogProbability(0)
        px1 = chain.xLogProbability(1)
        return pt3+px0+px1

    def startBlockMetropolis(self):
        """Samples x[0], theta[1] given x[i] up to
        maxSamplesPerBlock times in order to get a metropolis sample
        """
        chain = self.chain

        #in case the sampling fails, revert to old
        """
        xold = chain.x[0]
        thetaold = chain.theta[1:3]
        """
        xold = chain.x[0:2]
        thetaold = chain.theta[1:4]

        oldlogw = self.startBlockLogImportance()
        for iters in xrange(self.maxSamplesPerBlock):
            self.startBlockSample()
            newlogw = self.startBlockLogImportance()
            alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
            if random.random() <= alpha:
                #accept move
                return True

        #restore to old pristine copy
        """
        chain.x[0] = xold
        chain.theta[1:3] = thetaold
        """
        chain.x[0:2] = xold
        chain.theta[1:4] = thetaold
        for j in xrange(1,len(chain.x)):
            chain.cumtheta[j] = chain.cumtheta[j-1]+chain.theta[j]
        return False

    def endBlockSample(self):
        """Samples x[-1], theta[-1] according to the prior distribution on
        theta[-1] while leaving the rest of the chain unchanged.
        """
        chain = self.chain
        assert chain.endPosition == None
        """
        theta = chain.thetaPriors[-1].sample() if chain.thetaPriors[-1]!=None else random.uniform(0,2*math.pi)
        cumtheta = chain.cumtheta[-2]+theta
        chain.theta[-1] = theta
        chain.cumtheta[-1] = cumtheta
        chain.x[-1] = chain.x[-2]+np.array([math.cos(cumtheta),math.sin(cumtheta)])*chain.L[-1]
        """
        theta1 = chain.thetaPriors[-2].sample() if chain.thetaPriors[-2]!=None else random.uniform(0,2*math.pi)
        theta2 = chain.thetaPriors[-1].sample() if chain.thetaPriors[-1]!=None else random.uniform(0,2*math.pi)
        cumtheta1 = chain.cumtheta[-3]+theta1
        cumtheta2 = cumtheta1+theta2
        chain.theta[-2] = theta1
        chain.theta[-1] = theta2
        chain.cumtheta[-2] = cumtheta1
        chain.cumtheta[-1] = cumtheta2
        chain.x[-2] = chain.x[-3]+np.array([math.cos(cumtheta1),math.sin(cumtheta1)])*chain.L[-2]
        chain.x[-1] = chain.x[-2]+np.array([math.cos(cumtheta2),math.sin(cumtheta2)])*chain.L[-1]
        return

    def endBlockLogImportance(self):
        """Returns the log-importance weight w'
          log P(x[-1],theta[-1]|x[-2]) - log Q(x[-1],theta[-1]|x[-2])
        to be used in importance sampling. """
        chain = self.chain
        """
        #pt1 = chain.thetaLogProbability(-1)
        px1 = chain.xLogProbability(-1)
        """
        #pt1 = chain.thetaLogProbability(-1)
        #pt2 = chain.thetaLogProbability(-2)
        px1 = chain.xLogProbability(-1)
        px2 = chain.xLogProbability(-2)
        return px1+px2

    def endBlockMetropolis(self):
        """Samples x[-1], theta[-1] given x[-2] up to
        maxSamplesPerBlock times in order to get a metropolis sample
        """
        chain = self.chain

        #in case the sampling fails, revert to old
        """
        xold = chain.x[-1]
        thetaold = chain.theta[-1]
        """
        xold = chain.x[-2:]
        thetaold = chain.theta[-2:]

        oldlogw = self.endBlockLogImportance()
        for iters in xrange(self.maxSamplesPerBlock):
            self.endBlockSample()
            newlogw = self.endBlockLogImportance()
            alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
            #print newlogw,oldlogw
            if random.random() <= alpha:
                #accept move
                return True

        #restore to old pristine copy
        """
        chain.x[-1] = xold
        chain.theta[-1] = thetaold
        chain.cumtheta[-1] = chain.cumtheta[-2]+chain.theta[-1]
        """
        chain.x[-2:] = xold
        chain.theta[-2:] = thetaold
        chain.cumtheta[-2] = chain.cumtheta[-3]+chain.theta[-2]
        chain.cumtheta[-1] = chain.cumtheta[-2]+chain.theta[-1]
        return False

    def block2Sample(self,i):
        """Samples x[i] and x[i+1] given x[i-1] and x[i+2]. Returns true if
        successful.  Otherwise, the state of the variables theta[i:i+2],
        cumtheta[i:i+2] and  x[i:i+2] are undefined"""
        chain = self.chain
        #theta1 = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        theta1 = angleSampleRLG(chain.x[i-1],chain.cumtheta[i-1],chain.L[i],
                                chain.x[i+2],chain.L[i+1]+chain.L[i+2],chain.thetaPriors[i])
        cumtheta1 = chain.cumtheta[i-1]+theta1
        chain.x[i] = chain.x[i-1]+np.array([math.cos(cumtheta1),math.sin(cumtheta1)])*chain.L[i]
        if abs(distance(chain.x[i],chain.x[i+2]))<=chain.L[i+1]+chain.L[i+2]:
            #reachable
            try:
                chain.theta[i] = theta1
                chain.cumtheta[i] = theta1+chain.cumtheta[i-1]
                #solve for IK constraint
                chain.closeChainSample(i+1)
                #fix up thetas
                for j in xrange(i,len(chain.x)):
                    chain.cumtheta[j] = chain.cumtheta[j-1]+chain.theta[j]
                return True
            except ValueError:
                return False
        return False

    def block2LogImportance(self,i):
        """Returns the log-importance weight w'
          log P(x[i:i+1],theta[i:i+3]|x[i-1],x[i+2]) -
          log Q(x[i:i+1],theta[i:i+3]|x[i-1],x[i+2])
        to be used in importance sampling. """
        chain = self.chain
        pt1 = chain.thetaLogProbability(i+1)
        pt2 = chain.thetaLogProbability(i+2)
        pt3 = chain.thetaLogProbability(i+3) if i+3 < len(chain.theta) else 0.0
        px1 = chain.xLogProbability(i)
        px2 = chain.xLogProbability(i+1)
        try:
            return pt1+pt2+pt3+px1+px2+math.log(self.block2MetricSqrt(chain.cumtheta[i],chain.cumtheta[i+1],chain.cumtheta[i+2],chain.L[i],chain.L[i+1],chain.L[i+2]))
        except ValueError:
            #domain error at singularity
            return 1e308

    def block2Metropolis(self,i):
        """Samples x[i] and x[i+1] given x[i-1] and x[i+2] up to
        maxSamplesPerBlock times in order to get a metropolis sample
        """
        chain = self.chain
        R = distance(chain.x[i-1],chain.x[i+2])
        L = sum(chain.L[i:i+3])
        if abs(R-L)<1e-5:
            #straightened out, only 1 solution
            return True
        if R > L:
            print "Gibbs step: points too far away",R,">",L
            raise False

        #in case the sampling fails, revert to old
        xold = chain.x[i:i+2]
        thetaold = chain.theta[i:i+4] if (i+3 < len(chain.theta)) else chain.theta[i:i+3]

        oldlogw = self.block2LogImportance(i)
        for iters in xrange(self.maxSamplesPerBlock):
            if self.block2Sample(i):
                newlogw = self.block2LogImportance(i)
                alpha = (1.0 if newlogw >= oldlogw else math.exp(newlogw-oldlogw))
                if random.random() <= alpha:
                    #accept move
                    return True

        #restore to old pristine copy
        chain.x[i:i+2] = xold
        if i+3 < len(chain.theta):
            chain.theta[i:i+4] = thetaold
        else:
            chain.theta[i:i+3] = thetaold
        for j in xrange(i,len(chain.x)):
            chain.cumtheta[j] = chain.cumtheta[j-1]+chain.theta[j]
        return False


    def gibbsPassMetropolis(self,order=None):
        """Passes a Gibbs step through all links in the given order"""
        #sanity check
        if self.chain.endPosition!=None and any(self.chain.x[-1] != self.chain.endPosition):
            raise ValueError("End of chain must satisfy loop closure")
        if self.chain.startPosition!=None and any(self.chain.x[0] != self.chain.startPosition):
            raise ValueError("Start of chain must satisfy loop closure")

        if order == None:
            s = 1 if self.chain.startPosition!=None else 0
            e = len(self.chain.x)-2 if self.chain.endPosition!=None else len(self.chain.x)-1
            order = xrange(s,e)
        for i in order:
            if i == 0:
                self.startBlockMetropolis()
            elif i >= len(self.chain.x)-2:
                self.endBlockMetropolis()
            else:
                self.block2Metropolis(i)
        return

def particleGibbsPass(chainList,samplesPerParticle=1):
    assert samplesPerParticle >= 1
    for i in xrange(1,len(chainList[0].x)-2):
        newParticles = []
        newWeights = []
        for c in chainList:
            for s in xrange(samplesPerParticle):
                try:
                    (wnew,wold) = c.block2Step(i)
                except ValueError:
                    print "Warning, gibbs step failed"
                    continue
                newParticles.append(copy.deepcopy(c))
                newWeights.append(math.exp(wnew))

        #resample the particles
        chainList = [newParticles[i] for i in stratifiedSample(newWeights,len(chainList))]
    return chainList

def genGibbsMetropolis(chain,numsamples,burnin=0,skip=1):
    for s in xrange(burnin):
        print "burnin",s
        chain.gibbsPassMetropolis()
    for s in xrange(numsamples*skip):
        chain.gibbsPassMetropolis()
        if s % skip == 0:
            print "sample",s
            yield chain
    return

def genGibbsParticle(chain,numsamples,burnin=10):
    chainList = [copy.deepcopy(chain)]*numsamples
    for iters in xrange(burnin):
        chainList = particleGibbsPass(chainList)
    return chainList


def genRejectionSampling(chain,numsamples,r=0.1):
    numattempts = 0
    numsampled = 0
    #maintain values for current chain
    oldtheta = chain.theta[:]
    logpold = sum(chain.xLogProbability(i) for i in xrange(len(chain.x)))
    logpold += 0.5*math.log(chain.metricDeterminant())
    assert len(chain.L)==4
    #x1,x2 sampling metric
    logpold -= math.log(math.sqrt(0.5*chain.L[1]**2*chain.L[2]**2*(3.0-math.cos(2.0*(chain.cumtheta[1]-chain.cumtheta[2])))))
    while numsampled < numsamples:
        numattempts+=1
        for i in xrange(1,len(chain.x)):
            chain.theta[i] = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        chain.forwardKinematics()
        if abs(distance(chain.x[-1],chain.endPosition))<=r:
            try:
                chain.closeChainSample(len(chain.x)-2)
                logpnew = sum(math.log(chain.xPriors[i].probability(chain.x[i])) if chain.xPriors[i] != None else 0.0 for i in xrange(len(chain.x)))
                logpnew += 0.5*math.log(chain.metricDeterminant())
                logpnew -= math.log(math.sqrt(0.5*chain.L[1]**2*chain.L[2]**2*(3.0-math.cos(2.0*(chain.cumtheta[1]-chain.cumtheta[2])))))
                alpha = (1.0 if lowpnew >= logpold else math.exp(logpnew - logpold))
                if random.random() <= alpha:
                    #accept
                    yield chain
                    numsampled += 1
                    oldtheta = chain.theta[:]
                    logpold = logpnew
                else:
                    #reject
                    pass
            except ValueError:
                print "Warning, final closure failed"
                continue
    print numsamples,"/",numattempts,"within tolerance"
    return


def genRejectionSampling_onCircle(chain,numsamples,r=0.1):
    numattempts = 0
    numsampled = 0
    #maintain values for current chain
    oldtheta = chain.theta[:]
    logpold = sum(chain.xLogProbability(i) for i in xrange(len(chain.x)))
    logpold += chain.thetaLogProbability(-1)
    while numsampled < numsamples:
        numattempts+=1
        for i in xrange(1,len(chain.x)):
            chain.theta[i] = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        chain.forwardKinematics()
        if abs(distance(chain.x[-2],chain.endPosition)-chain.L[-1]) <= r:
            try:
                chain.closeChainSample(len(chain.x)-2)
                logpnew = sum(chain.xLogProbability(i) for i in xrange(len(chain.x)))
                logpnew += chain.thetaLogProbability(-1)
                alpha = (1.0 if logpnew >= logpold else math.exp(logpnew - logpold))
                if random.random() <= alpha:
                    #accept
                    yield chain
                    numsampled += 1
                    oldtheta = chain.theta[:]
                    logpold = logpnew
                else:
                    #reject
                    pass
            except ValueError:
                print "Warning, final closure failed"
                continue
    print numsamples,"/",numattempts,"within tolerance"
    return


def genRejectionSampling_IKSolve(chain,numsamples):
    numattempts = 0
    numsampled = 0
    #maintain values for current chain
    oldtheta = chain.theta[:]
    logpold = sum(chain.xLogProbability(i) for i in xrange(len(chain.x)))
    logpold += chain.thetaLogProbability(-2)
    logpold += chain.thetaLogProbability(-1)
    assert len(chain.L)>=4
    assert len(chain.theta)>=4
    #logpold -= 0.5*math.log(chain.block2MetricSqrt(*(chain.cumtheta[-3:]+chain.L[-3:])))
    oldtheta = chain.theta[:]
    while numsampled < numsamples:
        numattempts+=1
        for i in xrange(1,len(chain.x)):
            chain.theta[i] = chain.thetaPriors[i].sample() if chain.thetaPriors[i]!=None else random.uniform(0,2*math.pi)
        chain.forwardKinematics()
        chain.x[-1] = chain.endPosition
        try:
            chain.closeChainSample(len(chain.x)-2)
            logpnew = sum(chain.xLogProbability(i) for i in xrange(len(chain.x)))
            logpnew += chain.thetaLogProbability(-2)
            logpnew += chain.thetaLogProbability(-1)
            #logpnew -= 0.5*math.log(chain.block2MetricSqrt(*(chain.cumtheta[-3:]+chain.L[-3:])))
            if logpold > 5.0:
                exit(0)
            alpha = (1.0 if logpnew > logpold else math.exp(logpnew - logpold))
            if random.random() <= alpha:
                #accept move
                yield chain
                numsampled += 1
                logpold = logpnew
                oldtheta = chain.theta[:]
            else:
                #reject move
                chain.theta = oldtheta[:]
                chain.forwardKinematics()
                numsampled += 1
                yield chain
                pass
        except ValueError:
            continue
    print numsamples,"/",numattempts,"within tolerance"
    return

def test1():
    chain = Kinematic2DChain()
    chain.startBuild(np.array([0.0,0.0]))
    chain.add(1.0,math.pi/2,None,None)
    chain.add(1.0,-math.pi/2,None,None)
    chain.add(1.0,-math.pi/2,None,None)
    chain.endBuild(np.array([1.5,0.0]))
    chain.x[-1] = chain.endPosition
    chain.closeChainSample(len(chain.x)-2)

    numsamples = 10000
    print "Constrained rejection sampling"
    for s in genRejectionSampling(chain,numsamples):
        print angle(s.x[1],s.x[0]),anglesub(angle(s.x[2],s.x[1]),angle(s.x[1],s.x[0])),anglesub(angle(s.x[3],s.x[2]),angle(s.x[2],s.x[1]))

    #print "Circle rejection sampling"
    #for s in genRejectionSampling_onCircle(chain,numsamples,0.01):
    #    print angle(s.x[1],s.x[0]),anglesub(angle(s.x[2],s.x[1]),angle(s.x[1],s.x[0])),anglesub(angle(s.x[3],s.x[2]),angle(s.x[2],s.x[1]))

    #print "IK solve sampling"
    #for s in genRejectionSampling_IKSolve(chain,numsamples):
    #    print angle(s.x[1],s.x[0]),anglesub(angle(s.x[2],s.x[1]),angle(s.x[1],s.x[0])),anglesub(angle(s.x[3],s.x[2]),angle(s.x[2],s.x[1]))

def test3():
    chain = Gibbs2DChainSampler()
    chain.startBuild(np.array([0.0,0.0]))
    chain.add(1.0,math.pi/2,None,None)
    chain.add(1.0,-math.pi/2,None,None)
    chain.add(1.0,0,None,None)
    chain.add(1.0,-math.pi/2,None,None)
    chain.endBuild(np.array([2.0,0.0]))
    chain.x[-1] = chain.endPosition
    chain.closeChainSample(len(chain.x)-2)

    numsamples = 10
    print "Gibbs/metropolis"
    for s in genGibbsMetropolis(chain,numsamples):
        for xi in chain.x:
            print xi[0],xi[1],
        print

    print "Particle set Gibbs sampling"
    for s in genGibbsParticle(chain,numsamples):
        for xi in chain.x:
            print xi[0],xi[1],
        print

    print "Constrained rejection sampling"
    for s in genRejectionSampling_IKSolve(chain,numsamples):
        for xi in chain.x:
            print xi[0],xi[1],
        print

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

def testSamplingMethod(method,args,mcmc):
    t0 = time.time()
    samples = [np.concatenate(c.x) for c in method(*args)]
    t1 = time.time()
    numsamples = len(samples)
    print "Sampling time =",t1-t0
    print "Mean",
    for v in sum(samples)/numsamples: print v,
    print
    if mcmc:
        print "Mean moving average"
        for i in xrange(10):
            mean = sum(samples[i*numsamples/10:(i+1)*numsamples/10])/(numsamples/10)
            for v in mean: print v,
            print
    if mcmc:
        print "Autocorrelation"
        for i in xrange(1,20):
            print i,autocorrelation(samples,i,total=True)
        #print i,
        #for j in autocorrelation(samples,i):
        #    print j,
        #print 

def build_default_chain(n=4,theta=math.pi/3,thetapriorstd=0.0):
    thetaprior = None
    if thetapriorstd>0.0:
        thetaprior = AngularGaussianDistribution(0,thetapriorstd)
    chain = Kinematic2DChain()
    chain.startBuild(np.array([0.0,0.0]))
    chain.add(1.0,theta,None,None)
    for i in xrange(1,n/2):
        chain.add(1.0,0,thetaprior,None)
    if n%2 == 1:
        chain.add(1.0,-theta,thetaprior,None)
        chain.add(1.0,-theta,thetaprior,None)
    else:
        chain.add(1.0,-2.0*theta,thetaprior,None)
    for i in xrange(len(chain.x)-1,n):
        chain.add(1.0,0,thetaprior,None)

    size = 0.25
    """
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
    size = 0.1
    rad = n*0.5/math.pi/math.cos(2.0*math.pi/float(2*(n+1)))
    for i in xrange(n+1):
        u = float(i)/float(n+1)*2.0*math.pi
        chain.xPriors[i] = GaussianDistribution([rad*(1.0-math.cos(u)),rad*math.sin(u)],[size/(i+1),size/(i+1)])
    #sawtooth chain
    """
    for i in xrange(n+1):
        if i % 2 == 0:
            chain.xPriors[i] = GaussianDistribution([math.cos(theta)*i,0],[size,size])
        else:
            chain.xPriors[i] = GaussianDistribution([math.cos(theta)*i,math.sin(theta)],[size,size])
    """

    chain.endBuild(np.array([(n/2)*2*math.cos(theta)+n%2,0.0]))
    chain.forwardKinematics()
    chain.x[-1] = chain.endPosition
    assert len(chain.x) == n+1
    #chain.closeChainSample(len(chain.x)-2)
    return chain

def test2(n = 4):
    chain = Gibbs2DChainSampler()
    chain.copy(build_default_chain(n))

    numsamples = 1000
    print "n",n
    print "numsamples",numsamples
    
    print "Gibbs/metropolis"
    print "burnin",10
    testSamplingMethod(genGibbsMetropolis,[copy.deepcopy(chain),numsamples,10],mcmc=True)
    print "burnin",100
    testSamplingMethod(genGibbsMetropolis,[copy.deepcopy(chain),numsamples,100],mcmc=True)
    print "burnin",10,"skip",2
    testSamplingMethod(genGibbsMetropolis,[copy.deepcopy(chain),numsamples,10,2],mcmc=True)
    print "burnin",10,"skip",10
    testSamplingMethod(genGibbsMetropolis,[copy.deepcopy(chain),numsamples,10,10],mcmc=True)

    for b in xrange(1,5):
        print "Particle set Gibbs sampling"
        print "burnin",b
        testSamplingMethod(genGibbsParticle,[copy.deepcopy(chain),numsamples,b],mcmc=False)

    print "Constrained rejection sampling"
    testSamplingMethod(genRejectionSampling_IKSolve,[copy.deepcopy(chain),numsamples],mcmc=True)

#sanity check: am I computing distributions right? 
#test1()

#test autocorrelation for larger chains
#test2(4)
#test2(10)
#test2(20)
#test2(100)


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
		self.pack()
		# always make_buttons(), make_canvas() in order!
                self.make_buttons()
		self.make_canvas()
		self.redraw()
	
	def update(self):
            self.redraw()

        def toscreen(self,pt):
            w = HEIGHT/len(self.chain.theta)/0.9
            h = HEIGHT/len(self.chain.theta)/0.9
            ofsx = len(self.chain.theta)*0.25*w*WIDTH/HEIGHT
            ofsy = 0.5*HEIGHT
            return [int(pt[0]*w+ofsx),int(-pt[1]*h+ofsy)]

        def draw_chain(self):
            if self.drawlinks.get()==1:
                line = sum((self.toscreen(pt) for pt in self.chain.x),[])
                self.canvas.create_line(line)
            colors = ['red','orange','yellow','green','blue','purple','brown','gray']
            r = 3
            for i,pt in enumerate(self.chain.x):
                p = self.toscreen(pt)
		self.canvas.create_oval(p[0]-r,p[1]-r,p[0]+r,p[1]+r, fill=colors[i%len(colors)], outline='black')

        def score(self,x):
            self.chain.setState(x)
            lp = 0.0
            for i in xrange(1,len(self.chain.theta)):
                if self.chain.thetaPriors[i]!=None:
                    lp += self.chain.thetaPriors[i].log_probability(self.chain.theta[i])
                if self.chain.xPriors[i]!=None:
                    lp += self.chain.xPriors[i].log_probability(self.chain.x[i])
            return -lp

        def draw_priors(self):
            sigma = 3.0
            for i,prior in enumerate(self.chain.xPriors):
                if prior != None:
                    ll = self.toscreen([m-s*sigma for m,s in zip(prior.mean,prior.std)])
                    ur = self.toscreen([m+s*sigma for m,s in zip(prior.mean,prior.std)])
                    self.canvas.create_oval(ll[0],ll[1],ur[0],ur[1], fill=None, outline='black')

	def redraw(self):
            self.canvas.delete(Tkinter.ALL)
            self.canvas.create_rectangle(2,2,WIDTH,HEIGHT, fill='white', outline='black')
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
                #test
                oldx = self.chain.getState()
                self.chain.forwardKinematics()
                self.draw_chain()
                self.chain.setState(oldx)
            
            print "Saving to canvas.svg"
            canvasvg.saveall("canvas.svg",self.canvas)

        def toggle_draw_all(self):
            self.redraw()

        def toggle_draw_links(self):
            self.redraw()

        def toggle_draw_best(self):
            self.redraw()

        def toggle_fix_endpoints(self):
            if self.fix_endpoints.get()==0:
                self.chain.openEndpoints()
            else:
                self.chain.closeEndpoints()

	def call_with(self, fns, vals):
            for fn, val in zip(fns,vals):
                fn(*val)
			
	def make_chain(self):
            self.chain = build_default_chain(self.num_angles,math.pi/3,self.theta_stddev)
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
            print "Sampling with the RLG"
            rlg = RLG2DChainSampler(self.chain)
            rlg.sample()
            self.samples.append(self.chain.getState())
            return

            print "Sampling with Newton-Raphson loop closure"
            nr = NewtonRaphson2DChainSampler(self.chain)
            for step in xrange(100):
                if nr.sample():
                    self.samples.append(self.chain.getState())
                    break
            return

        def sample_atom(self):
            #HACK: hook to compare different loop closure method
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
            oldendpoints = (self.chain.startPosition,self.chain.endPosition)
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
            if oldendpoints[0]!=None: self.chain.startPosition=self.chain.x[0]
            if oldendpoints[1]!=None: self.chain.endPosition=self.chain.x[-1]
            #self.chain.closeEndpoints()
            
            self.redraw()
            
        def sample(self):
            #normal order
            order = None
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
                    gibbs.gibbsPassMetropolis(order)
                gibbs.gibbsPassMetropolis(order)
                self.samples.append(self.chain.getState())
            t1 = time.time()
            print "Sampling time",t1-t0
            self.redraw()

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
            
            # Pack it all into subframes and then into the main frame's grid
            self.guiitems = [self.angleslabel, self.anglesentry, self.stdlabel, self.stdentry, self.resetbutton, self.fixendpointcheck, self.numsampleslabel, self.numsamplesentry, self.skiplabel,self.skipentry, self.samplebutton, self.sample_selected_button, self.sample_metropolis_button, self.drawallcheck,self.drawlinkscheck,self.drawbestcheck,self.clear_samples_button]
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
