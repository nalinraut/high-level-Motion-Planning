import math
import numpy as np
import copy
from sample import *
from function import *
from condition import *

class OptimizationProblem:
    """Specifies an optimization problem from a:
    - function f(x)
    - sampling space space
    - a constraint list constraints for which all constraints[i](x)
      must be True in a valid soln.
    """
    def __init__(self):
        self.f = None
        self.space = None
        self.constraints = []

    def setF(self,f):
        self.f = f

    def addConstraint(self,c):
        self.constraints.append(c)

    def evalCost(self,x):
        return self.f(x)

    def isFeasible(self,x):
        return all(c(x) for c in self.constraints)

    def isBoxBounded(self):
        return len(self.constraints)==1 and isinstance(self.constraints[0],BoxContains)

    def isObjectiveLinear(self):
        return hasattr(self.f,'order') and self.f.order<=1

    def isObjectiveQuadratic(self):
        return hasattr(self.f,'order') and self.f.order<=2

class LoggingInspector:
    def __init__(self,logPoints=False):
        self.logPoints = logPoints
        self.log = []

    def __call__(self,opt,operation,*args,**kwargs):
        if operation=='init':
            self.log = []
        elif operation=='step':
            if self.logPoints:
                self.log.append(opt.solution)
            else:
                self.log.append(opt.getSolutionValue())

class PrintingInspector:
    def __init__(self,frequency=1):
        self.frequency = frequency
        self.count = 0

    def __call__(self,opt,operation,*args,**kwargs):
        if operation=='init':
            self.count = 0
        elif operation=='done':
            print "Done"
        elif operation=='timeout':
            print "Done"
        elif operation=='step':
            self.count += 1
            if self.count % self.frequency == 0:
                print "Iteration:",self.count,"value",opt.getSolutionValue()

class Optimizer:
    """Given an optimization problem, optimizes it.  Multiple
    solution techniques are available.

    The best solution point and value are stored as a pair in the
    solution member.

    Logging is enabled by calling beginLogValues()/beginLogPoints() and the
    log is retreived using getLog().

    Progress printing is enabled by calling beginPrint(freq).

    Arbitrary inspection can be achieved by adding a new 'inspector' function
    into the parameters to run().  An inspector function is passed self and
    a string indicating the context in which it is called.

    Default parameters to the run() method can be given in the runparams
    dictionary.
    """
    
    def __init__(self,problem,**kwparams):
        self.problem = problem
        self.solution = None
        if 'x' in kwparams:
            x=kwparams['x']
            self.solution = x,self.problem.evalCost(x)
        self.inspector = kwparams.get('inspector',None)
        if self.inspector != None:
            self.inspector(self,'init')
        self.termCond = None
        if 'cond' in kwparams:
            self.termCond = kwparams['cond']
        elif 'numIters' in kwparams:
            self.termCond = CountCondition(kwparams['numIters'])

    def step(self,iterationNumber=None):
        """Take one step of the optimizer.  May raise StopIteration to
        indicate convergence."""
        raise NotImplementedError()

    def setSeed(self,x):
        self.solution = (x,self.problem.evalCost(x))

    def getSolution(self):
        return self.solution[0] if self.solution!=None else None

    def getSolutionValue(self):
        return self.solution[1] if self.solution!=None else None

    def getState(self):
        """Optional: return some representation of the solver's current
        state"""
        return None

    def setState(self,s):
        """Optional: set a representation of the solver's current
        state"""
        return None

    def run(self):
        if not self.termCond:
            raise ValueError("run() method needs a termination condition (cond, numIters params)")
            
        iters = 0
        done = False
        if hasattr(self.termCond,'start'):
            self.termCond.start()
        while self.termCond():
            try:
                self.step(iters)
            except StopIteration:
                done = True
            iters += 1
            if self.inspector != None:
                self.inspector(self,'step')
            if done:
                break
        if self.inspector != None:
            if done:
                self.inspector(self,'done')
            else:
                self.inspector(self,'timeout')
        return self.getSolution()

    def sampleFeasible(self,sampler):
        """Samples a feasible solution via a given sampling routine using
        rejection sampling.  Returns None if infeasible."""
        x = sampler()
        if self.problem.isFeasible(x):
            return x
        return None

    def beginLogValues(self):
        self.inspector=LoggingInspector()

    def beginLogPoints(self):
        self.inspector=LoggingInspector(True)

    def getLog(self):
        if hasattr(self.inspector,'log'):
            return self.inspector.log
        return None

    def beginPrint(self,freq=1):
        self.inspector=PrintingInspector(freq)
    
    def gradient(self,x):
        """Computes the gradient of f w.r.t. x.
        Output is a 1D vector. """
        g = self.problem.f.deriv(1,x)
        if len(g.shape)==2:
            return np.reshape(g,-1)
        return g

    def hessian(self,soln):
        """Computes the Hessian of f w.r.t. x"""
        return self.problem.f.deriv(2,x)

    def linesearch(self,xfx,dir,tol):
        x,fx = xfx
        alpha = 1.0
        gnorm = math.sqrt(np.dot(dir.T,dir))
        atol = tol / gnorm
        while alpha > atol:
            xtest = x+dir*alpha
            if self.problem.isFeasible(xtest):
                f = self.problem.evalCost(xtest)
                if f < fx:
                    return (xtest,f)
            alpha *= 0.5
        print "Line search converged with tolerance",tol
        return None

    def newtonStep(self,xfx,tol=1e-5,regularization=1e-2):
        """Newton's method.  The regularization parameter
        makes this the Levenberg-Marquardt algorithm"""
        x,fx = xfx
        H = self.hessian(x)
        g = self.gradient(x)
        for i in xrange(H.shape[0]):
            H[(i,i)] += regularization
        #take a step in the -H^-1 g direction
        d = -np.linalg.lstsq(H,g)[0]
        #print dirs
        res = self.linesearch(xfx,us,d,tol)
        return res

    def gradientDescentStep(self,xfx,tol=1e-3):
        """Take a gradient descent step to minimize cost"""
        x,fx = xfx
        g = self.gradient(x)
        #take a step in the -g direction
        res = self.linesearch(xfx,-g,tol)
        if res==None:
            return None
        return res

    def randomDescentStep(self,xfx,radius):
        """Performs randomized descent by sampling x with the given
        radius parameter (may be an array or list for axis-wise
        perturbations)"""
        x,fx = xfx
        res = self.sampleFeasible(self.BoxPerturbSampler(x,radius))
        if not res: return None
        fn = self.problem.evalCost(res)
        if fn < fx:
            return res
        return None

    def simulatedAnnealingStep(self,xfx,radius,T):
        """Performs simulated annealing at the given temperature T
        by sampling x with the given radius parameter (may be an
        array or list for axis-wise perturbations)"""
        x,fx = xfx
        res = self.sampleFeasible(self.BoxPerturbSampler(x,radius))
        if not res: return None
        fn = self.problem.evalCost(res)
        if fn < fx or random.random() < math.exp((fx-fn)/T):
            return res
        return None

class GradientDescentOptimizer(Optimizer):
    """Uses gradient descent to optimize a given point up to the indicated
    tolerance."""
    
    def __init__(self,problem,tol=1e-3,**kwargs):
        Optimizer.__init__(self,problem,**kwargs)
        self.tol = tol

    def step(self,iterationNumber=None):
        res = self.gradientDescentStep(self.solution,self.tol)
        if res: self.solution = res
        else: raise StopIteration()

class NewtonOptimizer(Optimizer):
    """Uses Newton's method to optimize a given point up to the indicated
    tolerance.
    
    The regularization parameter can also be a
    function of the number of iterations."""
    
    def __init__(self,problem,tol=1e-3,regularization=1e-2,**kwargs):
        Optimizer.__init__(self,problem,**kwargs)
        self.tol = tol
        self.regularization = regularization

    def step(self,iterationNumber=None):
        reg = self.regularization
        if hasattr(self.regularization,'__call__'):
            reg = self.regularization(iters)
        res = self.newtonDescentStep(self.solution,self.tol,reg)
        if res: self.solution = res
        else: raise StopIteration()

class RandomDescentOptimizer(Optimizer):
    """Uses random descent to optimize a given point until the termination
    condition is true.
    
    The radius parameter can also be a function of the number of iterations."""
    def __init__(self,problem,radius=0.1,**kwargs):
        Optimizer.__init__(self,problem,**kwargs)
        self.radius = radius

    def step(self,iterationNumber=None):
        rad = self.radius
        if hasattr(self.radius,'__call__'):
            rad = self.radius(iters)
        res = self.randomDescentStep(self.solution,tol,rad)
        if res: self.solution=res


class SimulatedAnnealingOptimizer(Optimizer):
    """Uses random descent to optimize a given point until the termination
    condition is true.
    
    The radius and temperature parameters can also be a function of the
    number of iterations."""    
    def __init__(self,problem,radius=0.1,temperature=(lambda iters:1.0/(iters+1)),**kwargs):
        Optimizer.__init__(self,problem,**kwargs)
        self.radius = radius
        self.temperature = temperature
        self.current = None

    def step(self):
        rad = self.radius
        if hasattr(self.radius,'__call__'):
            rad = self.radius(iters)
        T = self.temperature
        if hasattr(self.temperature,'__call__'):
            T = self.temperature(iters)
        if self.current == None:
            self.current = self.solution
        res = self.simulatedAnnealingStep(self.current,tol,rad,T)
        if res:
            self.current=res
            if res[1] < self.solution[1]:
                self.solution=res

class RandomRestartOptimizer(Optimizer):
    def __init__(self,problem,sample,localOpt,**kwargs):
        """Performs global optimization given the local optimizer
        localOpt.  Generates seeds using the sample routine and
        passes x=seed and localOptParams to the run method of the
        local optimizer."""
        Optimizer.__init__(self,problem,**kwargs)
        self.sample = sample
        
    def step(self,iterationNumber=None):
        x = self.sampleFeasible(self.sample)
        if x==None: return
        self.localOpt.setSeed(x)
        self.localOpt.run()

        #update best
        fbest = self.getSolutionValue()
        if fbest == None: fbest = 1e300
        fx = self.localOpt.getSolutionValue()
        if fx < fbest:
            self.solution = self.localOpt.solution

class ParallelDescentOptimizer(Optimizer):
    def __init__(self,problem,numPoints,survivalFraction,
                 sample,localOpt,**kwargs):
        """Performs global optimization given the local optimizer
        localOpt.  Generates numPoints points using the sample routine and
        passes x=seed to the local optimizer."""
        Optimizer.__init__(problem,**kwargs)
        xs = [self.sampleFeasible(sample) for i in range(numPoints)]
        self.points = [(p,self.problem.evalCost(p)) if p != None else (None,1e300) for p in xs]
        self.survivalFraction = survivalFraction
        self.sample = sample
        self.localOpts = [copy.deepcopy(localOpt) for i in range(numPoints)]
        
    def step(self,iterationNumber=None):
        #toss out worst x% of points
        self.points = sorted(self.points,lambda x:-x[1])
        ndel = int(len(self.points)*(1.0-self.survivalFraction))
        for i in xrange(ndel):
            x = self.sampleFeasible(self.sample)
            if x!=None:
                fx = self.problem.evalCost(x)
                self.points[i] = (x,fx)
                self.localOpts[i].setSeed(x)
        raise NotImplementedError()

def localOptimizer(problem,method='gradient',**kwparams):
    if method=='gradient':
        return GradientDescentOptimizer(problem,**kwparams)
    elif method=='newton':
        return NewtonOptimizer(problem,**kwparams)
    elif method=='random':
        return RandomDescentOptimizer(problem,**kwparams)
    elif method=='sa':
        return SimulatedAnnealingOptimizer(problem,**kwparams)
    else:
        raise ValueError("Unsupported local optimization method "+method)

def globalOptimizer(problem,method='random',localmethod='gradient',**kwparams):
    if method=='random':
        localoptp = dict()
        if 'localOptParams' in kwparams:
            localoptp = kwparams['localOptParams']
            del kwparams['localOptParams']
        lopt = localOptimizer(problem,localmethod,**localoptp)
        return RandomRestartOptimizer(problem,problem.sample,lopt,**kwparams)
    else:
        raise ValueError("Unsupported global optimization method "+method)
