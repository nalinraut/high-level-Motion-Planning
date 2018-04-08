import numpy as np
import random
from .. import function, sample
from ..utils import *
import optimize


class DoubleIntegratorFunc(function.Numeric):
    """A double integrator function on an n-dimensional state space x=(q,q'), with the
    control u = q''.
    """
    def __init__(self,n=None):
        self.n = n
    def __call__(self,x,u):
        t = x[0]
        n = (len(x)-1)/2
        if self.n == None:
            self.n = n
        else:
            assert(n == self.n)
        p = x[1:n+1]
        v = x[n+1:]
        dt = u[0]
        a = u[1:]
        tn = t+dt
        pn = p + dt*v + (0.5*dt**2)*a
        vn = v + dt*a
        return np.concatenate(([tn],pn,vn))
    def arity(self):
        return 2
    def deriv(self,order,x,u):
        if order > 2: return None
        if order == 1:
            dx = np.zeros((len(x),len(x)))
            du = np.zeros((len(x),len(u)))
            n = (len(x)-1)/2
            p = x[1:n+1]
            v = x[n+1:]
            dt = u[0]
            a = u[1:]
            dx[0,0] = 1
            du[0,0] = 1
            for i in xrange(n):
                dx[1+i,1+i]=1
                dx[1+i,1+i]=dt
                du[1+i,1+i]=0.5*dt**2
                dx[1+i+n,1+i+n]=1
                du[1+i+n,1+i]=dt
            return (dx,du)
        if order == 2:
            print("TODO: implement second derivative")
            
            return None

def doubleIntegratorSimFunc(n,dt=0):
    """State is (t,p,v) with p and v n-dimensional vectors, and control
    denoting n-dimensional acceleration a.  If dt!=0, the system
    time step is dt and the control is u=a.  Otherwise, the u term consists
    of (dt,a)"""
    #t' = t + dt
    #v' = v + dt a
    #p' = 1/2 dt ^2 a + dt v + p
    if not dt:
        return DoubleIntegratorFunc()
    else:
        A = np.zeros((1+n*2,1+n*2))
        B = np.zeros((1+n*2,n))
        c = np.zeros((1+n*2,))
        c[0] = dt
        A[0][0] = 1
        for i in xrange(n):
            A[1+i][1+i] = 1.0
            A[1+i][1+n+i] = dt
            A[1+n+i][1+n+i] = 1.0
            B[1+i][i] = 0.5*dt**2
            B[1+n+i][i] = dt
        return function.LinearBivariate(A,B,c)

class NormConstraint(function.Boolean):
    def __init__(self,r):
        self.r = r
    def __call__(self,x):
        return np.dot(x,x) <= self.r**2
    def arity(self):
        return 1

def features(params):
    return flatten(params)

def featureNames(params,prefix=''):
    return flatten_keys(params,prefix)

def makeProblem(timeCost,effortCost,goal,goalcost):
    dt = 0.1
    n = 2
    xd = 2*n+1
    horizon = 20
    ubound = 1.0
    p = optimize.TrajectoryOptimizationProblem()
    p.params = dict()
    p.params['name'] = 'doubleIntegratorProblem'
    p.params['n'] = n
    p.params['timeCost'] = timeCost
    p.params['effortCost'] = effortCost
    p.params['goal'] = goal
    p.params['goalcost'] = goalcost
    p.params['dt'] = dt
    p.params['ubound']=ubound
    p.setStartState(np.array([0]*xd))
    p.setF(doubleIntegratorSimFunc(n,dt))
    Puu = np.eye(n)*effortCost*dt
    p.setIncrementalCost(function.QuadraticBivariate(np.zeros((xd,xd)),np.zeros((xd,n)),Puu,np.zeros((xd,)),np.zeros((n,)),timeCost*dt))
    goal = np.array(goal+[0,0])
    #p.setTerminalCost(lambda x:0 if np.linalg.norm(x[1:5]-goal)<=goalRadius else 1000)
    Pxx = np.eye(5)*goalcost
    Pxx[0,0] = 0
    p.setTerminalCost(function.Quadratic(Pxx,-2.0*goalcost*np.array([0]+goal.tolist()),np.dot(goal,goal)*goalcost))
    p.horizon = horizon
    p.addUConstraint(lambda x,u:np.linalg.norm(u)<=ubound)
    p.usampler = sample.BoxSampler([(-ubound,ubound),(-ubound,ubound)])
    return p

def genProblem(params):
    return makeProblem(1.0,params['timeCost'],params['goal'],params['goalcost'])

def sampleProblem(effortbound=[0.0,1.0],goalbound=[(-1,1),(-1,1)],goalcostbound=[0.0,50.0]):
    effortCost = random.uniform(effortbound[0],effortbound[1])
    goal = [random.uniform(a,b) for (a,b) in goalbound]
    goalcost = random.uniform(goalcostbound[0],goalcostbound[1])
    return makeProblem(1.0,effortCost,goal,goalcost)

def globalOptimizer(problem):
    """Return the global optimizer used by the primitive library"""
    #local optimizer parameters
    localparams = {'numIters':100,'tol':1e-4}
    #10 restarts
    #sample initial solution 100 times
    opt = optimize.globalOptimizer(problem,'random','ddp',localOptParams=localparams,samplecond=100,numIters=10)
    return opt

def adaptOptimizer(problem,seedProblemParams,seedPrimitive):
    """Return the adaptation optimizer used by the primitive library"""
    #adapt for 100 iterations
    opt = optimize.localOptimizer(problem,'ddp',tol=1e-4,seed=seedPrimitive,numIters=100)
    return opt
