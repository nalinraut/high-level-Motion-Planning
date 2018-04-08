import math
import numpy as np
from ..sample import *
from ..function import *
from .. import optimize
from ..condition import *

class TrajectoryOptimizationProblem:
    """Specifies a trajectory optimization problem from a:
    - start state x0 
    - simulation function x[i+1] = f(x[i],u[i])
    - control sampling space uSpace
    - incremental cost function incrementalCost(x[i],u[i]).
      may be None to indicate no cost, or a list of functions to indicate
      time-varying cost (the latter not yet supported)
    - terminal cost function terminalCost(x[T])
      may be None to indicate no cost, or a list of functions to indicate
      time-varying cost (the latter not yet supported)
    - time horizon horizon (None specifies an unknown horizon)
    - a state constraint list xConstraints for which all xConstraints[i](x)
      must be True in a valid soln.  May be empty.
    - a control constraint list uConstraints for which all uConstraints[i](x,u)
      must be True in a valid soln.  May be empty.
    - a goal set goalSet that must be reached
    """
    def __init__(self):
        self.x0 = None
        self.f = None
        self.uSpace = None
        self.incrementalCost = None
        self.terminalCost = None
        self.horizon = None
        self.xConstraints = []
        self.uConstraints = []
        self.goalSet = None

    def setStartState(self,x0):
        self.x0 = x0

    def setF(self,f):
        self.f = f
        
    def setIncrementalCost(self,c):
        self.incrementalCost = c

    def setTerminalCost(self,c):
        self.terminalCost = c

    def addXConstraint(self,c):
        self.xConstraints.append(c)

    def addUConstraint(self,c):
        self.uConstraints.append(c)

    def setGoalSet(self,goalSet):
        self.goalSet = goalSet

    def simulate(self,x,u):
        return self.f(x,u)

    def evalIncrementalCost(self,x,u,t=None):
        if self.incrementalCost:
            return self.incrementalCost(x,u)
        return 0

    def evalTerminalCost(self,x,t=None):
        if self.terminalCost:
            return self.terminalCost(x)
        return 0

    def evalCost(self,xs,us):
        """Evaluates the total cost of the trajectory with states xs and
        controls us.  xs is required to start at x0."""
        if len(xs)!=len(us)+1:
            raise ValueError("Invalid length arrays")
        #if self.x0 != None and not (xs[0] == self.x0).all():
        #    raise ValueError("Trajectory starts at incorrect state")
        c = 0.0
        if self.incrementalCost:
            for i,(x,u) in enumerate(zip(xs[:-1],us)):
                c += self.evalIncrementalCost(x,u,i)
        c += self.evalTerminalCost(xs[-1],len(us))
        return c

    def isXFeasible(self,x):
        return all(c(x) for c in self.xConstraints)

    def isUFeasible(self,x,u):
        return all(c(x,u) for c in self.uConstraints)

    def isTrajectoryFeasible(self,xs,us):
        """Evaluates whether the trajectory with states xs and
        controls us is feasible.  xs is required to start at x0."""
        if len(xs)!=len(us)+1:
            raise ValueError("Invalid length arrays")
        if self.x0 != None and not (xs[0] == self.x0).all():
            raise ValueError("Trajectory starts at incorrect state")
        for i,(x,u) in enumerate(zip(xs[:-1],us)):
            if not self.isXFeasible(x) or not self.isUFeasible(x,u):
                return False
        if not self.isXFeasible(xs[-1]):
            return False
        return True

    def isFeasibleSolution(self,xs,us):
        if not self.isTrajectoryFeasible(xs,us):
            return False
        if self.goalSet:
            if not self.goalSet(xs[-1]): return False
        return True

class FlattenPattern:
    """A subset of a numpy array to flatten"""
    def __init__(self,entries = []):
        self.entries = entries
    
    def flatten(self,x):
        if entries==None:
            return x.ravel().tolist()
        res = []
        for e in entries:
            res.append(x[e])
        return res

    def lift(self,p,xtemplate):
        if entries==None:
            return np.array(p).reshape(xtemplate.shape)
        x = np.copy(xtemplate)
        for (e,pe) in zip(entries,p):
            x[e] = pe
        return x

class TrajectoryOptimizer:
    """Given a trajectory optimization problem, optimizes it.  Multiple
    solution techniques are available.
    
    The best solution trajectory and value are stored as a pair in the
    solution member.

    Logging is enabled by setting the logPoints and logValues flags to True.

    Default parameters to the run() method can be given in the runparams
    dictionary.
    """
    
    def __init__(self,problem,**kwparams):
        self.problem = problem
        self.solution = None
        self.runparams = dict()

        if 'u' in kwparams:
            us=kwparams['u']
            self.solution = self.simulate(us),us
        if 'seed' in kwparams:
            self.solution = kwparams['seed']
        
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

    def run(self):          
        if self.termCond == None:
            raise ValueError("run() method needs a termination condition (cond, numIters)")
        iters = 0
        if hasattr(self.termCond,'start'):
            self.termCond.start()
        while self.termCond():
            done = False
            try:
                self.step(iters)
            except StopIteration:
                done = True
            iters += 1
            if self.inspector != None:
                self.inspector(self,'step')
            if done:
                break
        return self.getSolution()

    def beginLogValues(self):
        self.inspector=optimize.LoggingInspector()

    def beginLogPoints(self):
        self.inspector=optimize.LoggingInspector(True)

    def getLog(self):
        if hasattr(self.inspector,'log'):
            return self.inspector.log
        return None

    def beginPrint(self,freq=1):
        self.inspector=optimize.PrintingInspector(freq)

    def setSeed(self,x):
        self.solution = x

    def getSolution(self):
        return self.solution

    def getSolutionControls(self):
        return self.solution[0] if self.solution!=None else None

    def getSolutionPath(self):
        return self.solution[1] if self.solution!=None else None

    def getSolutionValue(self):
        return self.problem.evalCost(*self.solution) if self.solution!=None else None

    def simulate(self,us):
        """Given a list of controls, rolls out the simulation trace"""
        xs = [self.problem.x0]
        for u in us:
            xs.append(self.problem.simulate(xs[-1],u))
        return xs

    def sampleFeasibleControl(self,x,uSampler,cond):
        """Samples a feasible control u starting from x using the given
        sampling routine.  Calls cond() every sample, and repeats until cond
        returns False"""
        while cond():
            u = uSampler()
            if self.problem.isUFeasible(x,u):
                return u
        return None

    def sampleAndSimulateFeasibleControl(self,x,uSampler,cond):
        while cond():
            u = uSampler()
            if self.problem.isUFeasible(x,u):
                xn = self.problem.f(x,u)
                if self.problem.isXFeasible(xn):
                    return (u,xn)
        return None

    def sampleFeasibleSearch(self,usamplers,cond,branchingFactor):
        """Performs a depth first search for a feasible control sequence.
        Each search depth is limited by the branching factor."""
        counts = [0]
        xs = [self.problem.x0]
        us = []
        while cond():
            assert len(xs)==len(counts)
            assert len(xs) >= 1
            if len(xs)==len(usamplers)+1: return (xs,us)
            uSampler = usamplers[len(xs)-1]
            res = self.sampleAndSimulateFeasibleControl(xs[-1],uSampler,CountCondition(branchingFactor))
            counts[-1] += 1
            if res:
                us.append(res[0])
                xs.append(res[1])
                counts.append(0)
            elif res==None and len(xs) > 1:
                #backtrack
                us.pop(-1)
                xs.pop(-1)
                counts.pop(-1)
                while len(counts)>1 and counts[-1]==branchingFactor:
                    us.pop(-1)
                    xs.pop(-1)
                    counts.pop(-1)
        return (xs,us)

    def sampleInitialSolution(self,uSampler,cond):
        """Samples a trajectory (xs,us) that is feasible.  Repeats until cond
        returns False.  On failure returns None."""
        if self.problem.horizon == None and self.problem.goalSet == None:
            return ([self.problem.x0],[])
        if not self.problem.isXFeasible(self.problem.x0):
            raise ValueError("Initial state is not feasible")
        return self.sampleFeasibleSearch([uSampler]*self.problem.horizon,cond,5)

    def linearizeF(self,x,u):
        """Computes the linearized version of f(x+dx,u+du) ~= A dx + B du + c.
        The tuple (A,B,c) is returned."""
        if not hasattr(self.problem.f,'deriv'):
            raise ValueError("Simulation function is not differentiable")
        c = self.problem.f(x,u)
        A,B = self.problem.f.deriv(1,x,u)
        return (A,B,c)
    
    def gradient(self,soln):
        """Computes the gradient of the costs w.r.t. the soln=(xs,us).
        The gradient (gx,gu) is a sparse gradient"""
        xs,us = soln
        gx = dict()
        gu = dict()
        if self.problem.incrementalCost:
            if not hasattr(self.problem.incrementalCost,'deriv'):
                raise ValueError("Incremental cost is not differentiable")
            for i,(x,u) in enumerate(zip(xs[:-1],us)):
                gradx,gradu = self.problem.incrementalCost.deriv(1,x,u)
                if gradx != None: gx[i] = gradx
                if gradu != None: gu[i] = gradu
        if self.problem.terminalCost:
            if not hasattr(self.problem.terminalCost,'deriv'):
                raise ValueError("Terminal cost is not differentiable")
            gx[len(us)] = self.problem.terminalCost.deriv(1,xs[-1])
        return (gx,gu)

    def hessian(self,soln):
        """Computes the Hessian of the costs w.r.t. the soln=(xs,us).
        The hessians (Hxx,Hxu,Huu) are sparse"""
        xs,us = soln
        Hxx = dict()
        Hxu = dict()
        Huu = dict()
        if self.problem.incrementalCost:
            if not hasattr(self.problem.incrementalCost,'deriv'):
                raise ValueError("Incremental cost is not differentiable")
            for i,(x,u) in enumerate(zip(xs[:-1],us)):
                H = self.problem.incrementalCost.deriv(2,x,u)
                if H==None: continue
                if H[0]!=None and H[0][0]!=None:
                    Hxx[i] = H[0][0]
                if H[0]!=None and H[0][1]!=None:
                    Hxu[i] = H[0][1]
                if H[1]!=None and H[1][1]!=None:
                    Huu[i] = H[1][1]
        if self.problem.terminalCost:
            if not hasattr(self.problem.terminalCost,'deriv'):
                raise ValueError("Terminal cost is not differentiable")
            H = self.problem.terminalCost.deriv(2,xs[-1])
            if H!=None:
                Hxx[len(us)] = H
        return (Hxx,Hxu,Huu)

    def linesearch(self,x,u,dir,tol):
        f0 = self.problem.evalCost(x,u)
        alpha = 1.0
        gnorm = math.sqrt(sum([np.dot(d,d) for d in dir]))
        atol = tol / gnorm
        while alpha > atol:
            xtest = [x[0]]
            utest = []
            for (ui,d) in zip(u,dir):
                utest.append(ui+alpha*d)
                if not self.problem.isUFeasible(xtest[-1],utest[-1]):
                    utest[-1] = ui
                xtest.append(self.problem.f(xtest[-1],utest[-1]))
                #TODO: handle x constraints
            if self.problem.isTrajectoryFeasible(xtest,utest):
                f = self.problem.evalCost(xtest,utest)
                if f < f0:
                    return xtest,utest
            alpha *= 0.5
        print "Line search converged with tolerance",tol
        return None

    def ddpStep(self,soln,tol=1e-5,regularization=0.01):
        """Differential dynamic programming -- essentially a Newton's
        method.  Line search is conducted to the given tolerance.
        Regularization of the step direction is given by the regularization
        parameter."""
        xs,us = soln
        gx,gu = self.gradient(soln)
        nx = len(xs[0])
        nu = len(us[0])
        Hxx,Hxu,Huu = self.hessian(soln)
        #recursively compute the total gradient
        Jxx = [None]*len(xs)
        Juu = [None]*len(us)
        Jxu = [None]*len(us)
        Jx = [None]*len(xs)
        Ju = [None]*len(us)
        j = len(xs)-1
        Jx[j] = gx[j]
        Jxx[j] = Hxx.get(j,np.zeros((nx,nx)))
        for j in xrange(len(us),0,-1):
            A,B,c = self.linearizeF(xs[j-1],us[j-1])
            Jx[j-1] = gx[j-1] + np.dot(Jx[j],A)
            Ju[j-1] = gu[j-1] + np.dot(Jx[j],B)
            Jxx[j-1] = Hxx.get(j-1,0.0) + np.dot(A.T,np.dot(Jxx[j],A))
            Jxu[j-1] = Hxu.get(j-1,0.0) + np.dot(A.T,np.dot(Jxx[j],B))
            Juu[j-1] = Huu.get(j-1,0.0) + np.dot(B.T,np.dot(Jxx[j],B))
            Hf = self.problem.f.deriv(2,xs[j-1],us[j-1])
            if Hf != None and Hf != (None,None):
                #TODO: add terms relating second derivatives of f to Jxx, Jxu, Juu
                raise NotImplementedError()
            #print j-1,"hessian:",Juu[j-1]
            #print j-1,"gradient:",Ju[j-1]
        #take a step in the -Juu^-1 Ju direction
        dirs = [-np.linalg.lstsq(H,g,rcond=regularization)[0] for H,g in zip(Juu,Ju)]
        #print dirs
        res = self.linesearch(xs,us,dirs,tol)
        if res==None:
            return None
        return res

    def gradientDescentStep(self,soln,tol=1e-3):
        """Take a gradient descent step to minimize cost"""
        xs,us = soln
        gx,gu = self.gradient(soln)
        #recursively compute the total gradient
        Jx = [None]*len(xs)
        Ju = [None]*len(us)
        j = len(xs)-1
        Jx[j] = gx[j]
        for j in xrange(len(us),0,-1):
            A,B,c = self.linearizeF(xs[j-1],us[j-1])
            Jx[j-1] = gx[j-1] + np.dot(Jx[j],A)
            Ju[j-1] = gu[j-1] + np.dot(Jx[j],B)
        #take a step in the -Ju direction
        res = self.linesearch(xs,us,[-g for g in Ju],tol)
        if res==None:
            return None
        return res

    def samplePerturbedTrajectory(self,soln,radius,cond,branchingFactor):
        xs,us = soln
        return self.sampleFeasibleSearch([BoxPerturbSampler(u,radius) for u in us],cond,branchingFactor)

    def randomDescentStep(self,soln,radius,cond):
        """Performs randomized descent by sampling u's with the given
        radius parameter (may be an array or list for axis-wise
        perturbations)"""
        xs,us = soln
        f = self.problem.evalCost(*soln)
        res = self.samplePerturbedTrajectory(soln,radius,cond,5)
        if not res:
            return None
        xn,un = res
        fn = self.problem.evalCost(xn,un)
        if fn < f:
            return (xn,un)
        return None


    def simulatedAnnealingStep(self,soln,radius,T,cond):
        """Performs simulated annealing at the given temperature T
        by sampling u's with the given radius parameter (may be an
        array or list for axis-wise perturbations)"""
        xs,us = soln
        f = self.problem.evalCost(*soln)
        res = self.samplePerturbedTrajectory(soln,radius,cond,5)
        if not res:
            return None
        xn,un = res
        fn = self.problem.evalCost(xn,un)
        print "cost change %.02f -> %.02f, T %.02f, p(accept) %.02f"%(f,fn,T,math.exp((f-fn)/T))
        if fn < f or random.random() < math.exp((f-fn)/T):
            print "  accepted"
            return (xn,un)
        return None



class GradientDescentTrajectoryOptimizer(TrajectoryOptimizer):
    """Uses gradient descent to optimize a given trajectory up to the indicated
    tolerance."""
    
    def __init__(self,problem,tol=1e-3,**kwparams):
        TrajectoryOptimizer.__init__(self,problem,**kwparams)
        self.tol = tol

    def step(self,iterationNumber=None):
        res = self.gradientDescentStep(self.solution,self.tol)
        if res: self.solution = res
        else: raise StopIteration()

class DDPTrajectoryOptimizer(TrajectoryOptimizer):
    """Uses differential dynamic programming to optimize a
    given trajectory up to the indicated tolerance.
    
    The regularization parameter can also be a
    function of the number of iterations."""
    
    def __init__(self,problem,tol=1e-3,regularization=1e-2,**kwparams):
        TrajectoryOptimizer.__init__(self,problem,**kwparams)
        self.tol = tol
        self.regularization = regularization

    def step(self,iterationNumber=None):
        reg = self.regularization
        if hasattr(self.regularization,'__call__'):
            reg = self.regularization(iters)
        res = self.ddpStep(self.solution,self.tol,reg)
        if res: self.solution = res
        else: raise StopIteration()

class RandomDescentTrajectoryOptimizer(TrajectoryOptimizer):
    """Uses random descent to optimize a given trajectory until the termination
    condition is true.
    
    The radius parameter can also be a function of the number of iterations."""
    def __init__(self,problem,radius=0.1,samplecond=100,**kwparams):
        TrajectoryOptimizer.__init__(self,problem,**kwparams)
        self.radius = radius
        self.samplecond = samplecond

    def step(self,iterationNumber=None):
        rad = self.radius
        if hasattr(self.radius,'__call__'):
            rad = self.radius(iterationNumber)
        cond = self.samplecond
        if isinstance(cond,int):
            cond = CountCondition(cond)
        res = self.randomDescentStep(self.solution,tol,rad,cond)
        if res: self.solution=res


class SimulatedAnnealingTrajectoryOptimizer(TrajectoryOptimizer):
    """Uses random descent to optimize a given trajectory until the
    termination condition is true.
    
    The radius and temperature parameters can also be a function of the
    number of iterations."""    
    def __init__(self,problem,radius=0.1,samplecond=100,temperature=(lambda iters:1.0/(iters+1)),**kwparams):
        TrajectoryOptimizer.__init__(self,problem,**kwparams)
        self.radius = radius
        self.samplecond = samplecond
        self.temperature = temperature
        self.current = None

    def step(self,iterationNumber=None):
        rad = self.radius
        if hasattr(self.radius,'__call__'):
            rad = self.radius(iterationNumber)
        T = self.temperature
        if hasattr(self.temperature,'__call__'):
            T = self.temperature(iterationNumber)
        cond = self.samplecond
        if isinstance(cond,int):
            cond = CountCondition(cond)
        if self.current == None:
            self.current = self.solution
        res = self.simulatedAnnealingStep(self.current,rad,T,cond)
        if res:
            self.current=res
            if self.problem.evalCost(*res) < self.problem.evalCost(*self.solution):
                self.solution = res

class RandomRestartTrajectoryOptimizer(TrajectoryOptimizer):
    def __init__(self,problem,usample,localOpt,samplecond=100,**kwparams):
        """Performs global optimization given the local optimizer
        localOpt.  Generates seeds using the sample routine and
        passes the solution via the seed and localOptParams arguments for
        the run method of the local optimizer."""
        TrajectoryOptimizer.__init__(self,problem,**kwparams)
        self.usample = usample
        self.samplecond = samplecond
        self.localOpt = localOpt
        self.bestValue = 1e300
        
    def step(self,iterationNumber=None):
        cond = self.samplecond
        if isinstance(cond,int):
            cond = CountCondition(cond)
        x = self.sampleInitialSolution(self.usample,cond)
        if x==None: return
        print "*** Sampled initial solution with cost",self.problem.evalCost(*x),"***"
        self.localOpt.setSeed(x)
        self.localOpt.run()
        fbest = self.getSolutionValue()
        if fbest == None: fbest = 1e300
        fx = self.localOpt.getSolutionValue()
        print "*** Local optimize cost",fx,"***"
        if fx < fbest:
            self.solution = self.localOpt.solution

class RandomReplanTrajectoryOptimizer(TrajectoryOptimizer):
    def __init__(self,problem,planner,localOpt,**kwparams):
        """Performs global optimization given the given planner
        and the local optimizer localOpt.  Generates seeds using 
        planner() and then passes the solution via the seed and
        localOptParams argument for the() run method of the local optimizer."""
        TrajectoryOptimizer.__init__(self,problem,**kwparams)
        self.planner = planner
        self.localOpt = localOpt
        self.bestValue = 1e300
               
    def step(self,iterationNumber=None):
        x = self.planner()
        if x==None: return
        print "*** Replan",iterationNumber," found solution with cost",self.problem.evalCost(*x),"***"
        self.localOpt.setSeed(x)
        self.localOpt.run()
        fbest = self.getSolutionValue()
        if fbest == None: fbest = 1e300
        fx = self.localOpt.getSolutionValue()
        print "*** Local optimize",iterationNumber,"cost",fx,"***"
        if fx < fbest:
            self.solution = self.localOpt.solution


def localOptimizer(problem,method='gradient',**kwparams):
    """ Perform local trajectory optimization.

    Supported values for method:
        - gradient: gradient descent
        - ddp: differential dynamic programming
        - random: random descent
        - sa: simulated annealing
    """
    if method=='gradient':
        return GradientDescentTrajectoryOptimizer(problem,**kwparams)
    elif method=='ddp':
        return DDPTrajectoryOptimizer(problem,**kwparams)
    elif method=='random':
        return RandomDescentTrajectoryOptimizer(problem,**kwparams)
    elif method=='sa':
        return SimulatedAnnealingTrajectoryOptimizer(problem,**kwparams)
    else:
        raise ValueError("Unsupported local optimization method "+method)


def globalOptimizer(problem,method='random',localmethod='gradient',**kwparams):
    """ Perform global trajectory optimization.
    
    Supported values for method:
        - random: random restarts with control sampling.  The samplecond
          parameter limits the number of control sampling iterations.
          The localmethod and parameters are passed to localOptimizer.
        - randomrrt: random restarts with RRT planning.  The samplecond
          parameter limits the number of control sampling iterations.
          The localmethod and parameters are passed to localOptimizer.
    """
    if method=='random':
        localoptp = dict()
        if 'localOptParams' in kwparams:
            localoptp = kwparams['localOptParams']
            del kwparams['localOptParams']
        samplecond = 100
        if 'samplecond' in kwparams:
            samplecond = kwparams['samplecond']
            del kwparams['samplecond']
        lopt = localOptimizer(problem,localmethod,localoptp)
        opt =  RandomRestartTrajectoryOptimizer(problem,problem.usampler,lopt,samplecond=samplecond,**kwparams)
        opt.beginPrint()
        return opt
    if method=='randomrrt':
        import search
        localoptp = dict()
        if 'localOptParams' in kwparams:
            localoptp = kwparams['localOptParams']
            del kwparams['localOptParams']
        samplecond = 100
        if 'samplecond' in kwparams:
            samplecond = kwparams['samplecond']
            del kwparams['samplecond']
        lopt = localOptimizer(problem,localmethod,**localoptp)
        def rrt():
            planner = search.RRTSearch(problem,problem.xspace)
            r = planner.planMinimum(samplecond)
            res = None
            if r: res = planner.solutionPath(r)
            planner.destroy()
            return res
        rrt.problem = problem
        rrt.samplecond = samplecond
        opt = RandomReplanTrajectoryOptimizer(problem,rrt,lopt,**kwparams)
        opt.beginPrint()
        return opt
    else:
        raise ValueError("Unsupported global optimization method "+method)
