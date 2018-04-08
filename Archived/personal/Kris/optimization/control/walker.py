import klampt
import math
import numpy as np
import random
from .. import function
from .. import sample
import optimize
from geometry import se3,vectorops
import os

#states are WalkerState instances
#controls are PID commands (dt,qdes,dqdes)

def clamp(x,bounds):
    return [min(max(v,bmin),bmax) for v,(bmin,bmax) in zip(x,bounds)]

simTable = dict()

def getSim(simfile):
    if simfile in simTable:
        return simTable[simfile][1]
    world = klampt.WorldModel()
    if not world.readFile(simfile):
        raise ValueError("Invalid simulation file "+simfile)
    sim = klampt.Simulator(world)
    simTable[simfile] = (world,sim)
    return sim

class WalkerState:
    def __init__(self,sim,path=[]):
        self.state = sim.getState()
        self.q = sim.getActualConfig(0)
        self.v = sim.getActualVelocity(0)
        self.path = path

class WalkerControl:
    def __init__(self,v):
        self.dt = v[0]
        n = (len(v)-1)/2
        assert len(v)==1+2*n
        self.qdes = v[1:1+n]
        self.dqdes = v[1+n:1+2*n]


class SimStateDistanceCSpace:
    def __init__(self,sim):
        self.sim = sim
        self.robot = sim.getWorld().robot(0)
        robot = self.robot
        self.bounds = zip(*robot.getJointLimits()) + [(-v,v) for v in robot.getVelocityLimits()]
        self.bounds[0] = (-1,2)
        self.bounds[1] = (-1,2)
        self.bounds[2] = (0,2)
        #velocity limits on x,y,z,rz,ry,rx
        for i in range(12,18):
            self.bounds[i] = (-2,2)
        self.weights = [1.0/(b-a) for (a,b) in self.bounds]
        for i in range(0,3):
            self.weights[i] = 1.0
        for i in range(3,6):
            self.weights[i] = 0.1
        self.norm = 2.0

    def sample(self):
        return sample.BoxSampler(self.bounds)()
        
    def distance(self,a,b):
        if hasattr(a,'q'):
            a = a.q+a.v
        if hasattr(b,'q'):
            b = b.q+b.v
        d = vectorops.sub(a,b)
        for i in xrange(3,6):
            while d[i] > math.pi:
                d[i] -= 2.0*math.pi
            while d[i] < -math.pi:
                d[i] += 2.0*math.pi 
        if self.norm == 2.0:
            return math.sqrt(sum(wi*di**2 for di,wi in zip(d,self.weights) if wi != 0))
        elif self.norm == 'inf':
            return max(wi*abs(di) for di,wi in zip(d,self.weights) if wi != 0)
        elif self.norm == 1:
            return sum(wi*abs(di) for di,wi in zip(d,weights) if wi != 0)   
        else:
            return pow(sum(wi*pow(di,self.norm) for di,wi in zip(d,self.weights) if wi != 0),1.0/self.norm)

    def interpolate(self,a,b,u):
        if hasattr(a,'q'):
            a = a.q+a.v
        if hasattr(b,'q'):
            b = b.q+b.v
        return vectorops.madd(a,vectorops.sub(b,a),u)

class SimFunc(function.Numeric):
    def __init__(self,fn):
        self.sim = getSim(fn)
        self.world = self.sim.getWorld()
        self.stateTest = None
        
    def currentState(self):
        return WalkerState(self.sim)

    def setSimControl(self,x,u):
        wu = WalkerControl(u)
        c=self.sim.getController(0)
        qdes = clamp(vectorops.add(x.q[6:],wu.qdes),self.bounds[1:7])
        dqdes = clamp(vectorops.add(x.v[6:],wu.dqdes),self.bounds[7:13])
        c.setPIDCommand(qdes,dqdes)

    def __call__(self,x,u):
        self.sim.setState(x.state)
        dt = u[0]
        self.setSimControl(x,u)
        p = []
        #self.sim.simulate(self.dt)
        nsteps = int(math.ceil(dt/0.01))
        assert(dt >= 0)
        for i in xrange(nsteps):
            self.sim.simulate(dt/nsteps)
            p.append(WalkerState(self.sim))
            if self.stateTest and not self.stateTest(p[-1]):
                return p[-1]
        return WalkerState(self.sim,p)

    def arity(self):
        return 2

class SimIncrementalCost(function.Numeric):
    def __init__(self,sim,effortcost):
        self.sim = sim
        self.effortcost = effortcost
        self.torqueSensor = self.sim.getController(0).getNamedSensor("torqueSensors")
    def arity(self):
        return 2
    def __call__(self,x,u):
        sumsq = 0.0
        dt = u[0]
        for s in x.path:
            self.sim.setState(s.state)
            torques = self.sim.getActualTorques(0)
            sumsq += np.dot(torques,torques)/len(x.path)
        return sumsq*self.effortcost*dt

class SimPoint(function.Numeric):
    def __init__(self,sim,link,point):
        self.sim = sim
        self.robot = sim.getWorld().robot(0)
        self.link = self.robot.getLink(link)
        self.point = point
    def arity(self):
        return 1 
    def __call__(self,x):
        self.robot.setConfig(x.q)
        return self.link.getWorldPosition(self.point)

class SimVelocity(function.Numeric):
    def __init__(self,sim):
        self.sim = sim
        self.robot = sim.getWorld().robot(0)
    def arity(self):
        return 1
    def __call__(self,x):
        return x.v

class SimKECost(function.Numeric):
    def __init__(self,sim,weight):
        self.sim = sim
        self.robot = sim.getWorld().robot(0)
        self.weight = weight
    def arity(self):
        return 1
    def __call__(self,x):
        #hack to multiply by mass matrix
        self.robot.setConfig(x.q)
        self.robot.setVelocity([0.0]*len(x.v))
        Bq = self.robot.torquesFromAccel(x.v)
        return self.weight*0.5*vectorops.dot(Bq,x.v)

class SimInContact(function.Boolean):
    def __init__(self,sim,link):
        self.sim = sim
        world = sim.getWorld()
        self.linkID = world.robotLink(0,link).getID()
        self.envID = world.terrain(0).getID()
        sim.enableContactFeedback(self.linkID,self.envID)

    def arity(self):
        return 1

    def __call__(self,x):
        self.sim.setState(x.state)
        return self.sim.inContact(self.linkID,self.envID)


class SimLimitConstraint(function.Boolean):
    def __init__(self,sim):
        self.sim = sim
        self.qmin,self.qmax = self.sim.getWorld().robot(0).getJointLimits()
        self.dqmax = self.sim.getWorld().robot(0).getVelocityLimits()

    def arity(self):
        return 1

    def inJointLimits(self,x):
        q,dq = x.q,x.v
        """
        for i,(jd,jmin,jmax) in enumerate(zip(q,self.qmin,self.qmax)[6:]):
            if jd < jmin or jd > jmax:
                self.sim.setState(x.state)
                print "@ time",self.sim.getTime(),
                print "exceed JL",i,(jd,jmin,jmax)
                return False
        for i,(vd,vmax) in enumerate(zip(dq,self.dqmax)[6:]):
            if abs(vd) > vmax:
                print "@ time",self.sim.getTime(),
                print "exceed VL",i,(vd,vmax)
                return False
        """
        return all(jd >= jmin and jd <= jmax for jd,jmin,jmax in zip(q,self.qmin,self.qmax)[6:]) and all(abs(vd) <= vmax for vd,vmax in zip(dq,self.dqmax)[6:])

    def __call__(self,x):
        if not self.inJointLimits(x):
            return False;
        for s in x.path:
            if not self.inJointLimits(s):
                return False
        return True


class SimControlConstraint(function.Boolean):
    def __init__(self,sim):
        self.sim = sim
        self.qmin,self.qmax = self.sim.getWorld().robot(0).getJointLimits()
        self.dqmax = self.sim.getWorld().robot(0).getVelocityLimits()
    def arity(self):
        return 2
    def __call__(self,x,u):
        wu = WalkerControl(u)
        return wu.dt >= 0
        #return wu.dt >= 0 and all(jd+q >= jmin and jd+q <= jmax for jd,q,jmin,jmax in zip(wu.qdes,x.q[6:],self.qmin[6:],self.qmax[6:])) and all(abs(vd+v) <= vmax for vd,v,vmax in zip(wu.dqdes,x.v[6:],self.dqmax[6:]))


def features(params):
    simfile = params['simfile']
    goal = p.params['goal']
    sim = getSim(simfile)
    q = sim.getActualConfig(0)
    dq = sim.getActualVelocity(0)
    return q+dq+goal

def featureNames(params,prefix):
    simfile = params['simfile']
    goal = p.params['goal']
    effortcost = p.params['effortcost']
    sim = getSim(simfile)
    w = sim.getWorld()
    rob = w.robot(0)
    linkNames = [r.getLink(i).getName() for i in xrange(r.numLinks())]
    goalNames = [prefix+'goal[%d]'%(i,) for i in range(3)]
    return [prefix+'q[%s]'%(n,) for n in linkNames] + [prefix+'dq[%s]'%(n,) for n in linkNames] + goalNames

def testSimulationSaving(sim):
    start = sim.getState()
    sim.setState(start)
    if sim.getState() != start:
        print "original:",start
        print "new:",sim.getState()
        print "First diff",[i for i,(a,b) in enumerate(zip(start,sim.getState())) if a!=b][0]
        raise ValueError("Simulation error in get/setState")
    trace = []
    for i in xrange(10):
        sim.simulate(0.01)
        trace.append(sim.getState())
    sim.setState(start)
    for i in xrange(10):
        sim.simulate(0.01)
        if trace[i] != sim.getState():
            print "Simulation error at index",i,", not exactly reproduceable"
            print "original:",trace[i]
            print "new:",sim.getState()
            print "First diff",[i for i,(a,b) in enumerate(zip(trace[i],sim.getState())) if a!=b][0]
            raise ValueError("Simulation error")

def makeProblem(simfile,goal,effortcost):
    p = optimize.TrajectoryOptimizationProblem()
    p.params = dict()
    p.params['name'] = 'walker'
    p.params['simfile'] = simfile
    p.params['goal'] = goal
    p.params['effortcost'] = effortcost
    f = SimFunc(simfile)
    world = f.sim.getWorld()
    qcur, dqcur = world.robot(0).getConfig(),world.robot(0).getVelocity()
    f.sim.getController(0).setManualMode(True)
    f.sim.getController(0).setPIDCommand(qcur,dqcur)
    f.sim.simulate(0.001)
    f.sim.updateWorld()
    #testSimulationSaving(f.sim)
    p.setStartState(f.currentState())
    p.setF(f)
    p.setIncrementalCost(SimIncrementalCost(f.sim,effortcost))
    p.setTerminalCost(function.distance(SimPoint(f.sim,5,(0,0,0)),np.array(goal)) + 
                      function.dot_self(SimVelocity(f.sim))*0.05)
    #SimKECost(f.sim)*0.02)
    p.addUConstraint(SimControlConstraint(f.sim))
    p.addXConstraint(SimLimitConstraint(f.sim).and_(function.not_(SimInContact(f.sim,5))))
    absbounds = [(0.05,0.4)]+zip(*f.world.robot(0).getJointLimits())[6:]+[(-v,v) for v in f.world.robot(0).getVelocityLimits()][6:]
    relbounds = [(0.05,0.4)]+[(-0.4,0.4)]*6+[(-0.5,0.5)]*6
    p.f.bounds = absbounds
    p.f.stateTest = p.xConstraints[-1]
    p.usampler = sample.BoxSampler(relbounds)
    p.xspace = SimStateDistanceCSpace(f.sim)
    #p.setGoalSet((function.distance(SimPoint(f.sim,5,(0,0,0)),np.array(goal)) <= 0.1).and_
    #             (SimInContact(f.sim,8).or_(SimInContact(f.sim,11))))
    return p

def genProblem(params):
    return makeProblem(params['simfile'],params['goal'],params['effortcost'])

def sampleProblem(dir="walker",goalbounds=[(-1,1),(-1,1),(0.5,1.5)]):
    files = [os.path.join(dir,f) for f in os.listdir(dir)]
    simfile = random.choice(files)
    goal = [random.uniform(a,b) for (a,b) in goalbounds]
    print "Sampled problem file",simfile,"goal",goal
    return makeProblem(simfile,goal,0.0001)

def globalOptimizer(problem):
    """Return the global optimizer used by the primitive library"""
    #global optimizer parameters
    #5000 RRT iterations, 10 restarts
    #local optimizer parameters
    #100 simulated annealing iterations, 100 perturbation sampling iterations per iteration
    localparams = {'numIters':100,'radius':0.01,'temperature':(lambda x:0.2/(1+x*0.05)),'samplecond':100}
    opt = optimize.globalOptimizer(problem,'randomrrt','sa',localOptParams=localparams,samplecond=5000,numIters=10)
    return opt

def adaptOptimizer(problem,seedProblemParams,seedPrimitive):
    """Return the adaptation optimizer used by the primitive library"""
    #100 simulated annealing iterations, 100 perturbation sampling iterations per iteration
    opt = optimize.localOptimizer(problem,'sa',radius=0.01,samplecond=100,temperature=lambda x:0.2/(1.0+x*0.1),seed=seedPrimitive,numIters=100)
    return opt
