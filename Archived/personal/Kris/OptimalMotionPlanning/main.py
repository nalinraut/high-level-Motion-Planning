from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import math
from klampt.glprogram import GLProgram
from klampt import vectorops,so2,gldraw
from kinodynamicplanner import *
from rrtstarplanner import *
from configurationspace import *
from statespace import *
from edgechecker import *
from metric import *
import time
import copy

filename = {'ao-rrt':'ao_rrt',
            'ao-est':'ao_est',
            'rrt*':'rrtstar',
            'r-rrt':'repeated_rrt',
            'r-rrt-prune':'repeated_rrt_prune',
            'r-est':'repeated_est',
            'r-est-prune':'repeated_est_prune',
            'rrt':'rrt',
            'est':'est',
            'anytime-rrt':'anytime_rrt',
            'stable-sparse-rrt':'stable_sparse_rrt',
            'sst*':'stable_sparse_rrt_star'}

kinematicPlanners = set(['rrt*'])
optimalPlanners = set(['ao-rrt','ao-est','rrt*','r-rrt','r-rrt-prune','r-est','r-est-prune','anytime-rrt','stable-sparse-rrt','sst*'])

def makePlanner(type,space,start,goal,
                objective=None,
                checker=None,
                metric=euclideanMetric,
                heuristic=None,
                costLowerBound=None,
                **params):
    if isinstance(space,ControlSpace):
        if type in kinematicPlanners:
            raise ValueError("Cannot do kinodynamic planning with planner "+type)
        controlSpace = space
        space = space.configurationSpace()
    else:
        if type not in kinematicPlanners:
            controlSpace = ControlSpaceAdaptor(space)
            controlSpace.nextStateSamplingRange = popdefault(params,'nextStateSamplingRange',0.1,'makePlanner(): Warning, control space sampling range not provided, using 0.1')

    if isinstance(goal,(list,tuple)) or isinstance(goal,SingletonSubset):
        raise RuntimeError("Cannot yet handle singleton goals")

    if heuristic==None and costLowerBound != None:
        print "Constructing default heuristic from cost lower bound"
        heuristicCostToCome = lambda x:costLowerBound(start,x)
        if goal.project(start) == None:
            print "  No cost-to-go heuristic."
            heuristicCostToGo = None
        else:
            def h(x):
                xproj = goal.project(x)
                if xproj == None: return 0
                return costLowerBound(x,xproj)
        heuristicCostToGo = h
        heuristic = (heuristicCostToCome,heuristicCostToGo)
    else:
        if not isinstance(heuristic,(list,tuple)):
            heuristic = (None,heuristic)
        
    if checker==None:
        edgeCheckTolerance = popdefault(params,'edgeCheckTolerance',0.01,"makePlanner: Warning, edge checker and/or edge checking tolerance not specified, using default tolerance 0.01")
        checker = EpsilonEdgeChecker(space,edgeCheckTolerance)

    if type == 'rrt*':
        planner = RRTStar(space,metric,checker,**params)
    elif type == 'ao-rrt':
        planner = CostSpaceRRT(controlSpace,objective,metric,checker,**params)
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicCostControlSelector(planner.costSpace,controlSpace.nextStateSamplingRange))
    elif type == 'ao-est':
        planner = CostSpaceEST(controlSpace,objective,checker,**params)
    elif type =='rrt':
        planner = RRT(controlSpace,metric,checker)
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicControlSelector(planner.controlSpace,controlSpace.nextStateSamplingRange))
    elif type == 'est':
        planner = ESTWithProjections(controlSpace,checker,**params)
    elif type == 'r-est':
        planner = RepeatedEST(controlSpace,objective,checker,**params)
    elif type == 'r-est-prune':
        planner = RepeatedEST(controlSpace,objective,checker,**params)
        planner.doprune = True
    elif type == 'anytime-rrt':
        planner = AnytimeRRT(controlSpace,objective,metric,checker,**params)
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicControlSelector(controlSpace,controlSpace.nextStateSamplingRange))
    elif type == 'r-rrt':
        planner = RepeatedRRT(controlSpace,objective,metric,checker,**params)
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicControlSelector(controlSpace,controlSpace.nextStateSamplingRange))
    elif type == 'r-rrt-prune':
        planner = RepeatedRRT(controlSpace,objective,metric,checker,**params)
        planner.doprune = True
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicControlSelector(controlSpace,controlSpace.nextStateSamplingRange))
    elif type == 'stable-sparse-rrt':
        planner = StableSparseRRT(controlSpace,objective,metric,checker,**params)
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicControlSelector(controlSpace,controlSpace.nextStateSamplingRange))
    elif type == 'sst*':
        planner = StableSparseRRTStar(controlSpace,objective,metric,checker,**params)
        #set direct steering functions for kinematic spaces 
        if isinstance(controlSpace,ControlSpaceAdaptor):
            planner.setControlSelector(KinematicControlSelector(controlSpace,controlSpace.nextStateSamplingRange))
    else:
        raise RuntimeError("Invalid planner type "+type)
    planner.setBoundaryConditions(start,goal)
    if type.startswith=='ao' and heuristic != None:
        planner.setHeuristic(*heuristic)
    return planner

def testPlanner(planner,numTrials,maxTime,filename):    
    print "Testing planner for %d trials, %f seconds"%(numTrials,maxTime)
    print "Saving to",filename
    f = open(filename,'w')
    f.write("trial,plan iters,plan time,best cost\n")
    for trial in range(numTrials):
        print "Trial",trial+1
        planner.reset()
        curCost = float('inf')
        t0 = time.time()
        numupdates = 0
        iters = 0
        while time.time()-t0 < maxTime:
            planner.planMore(10)
            iters += 10
            if planner.bestPathCost != None and planner.bestPathCost != curCost:
                numupdates += 1
                curCost = planner.bestPathCost
                t1 = time.time()
                f.write(str(trial)+","+str(iters)+","+str(t1-t0)+","+str(curCost)+'\n')
        print "Final cost:",curCost
        if hasattr(planner,'stats'):
            planner.stats.pretty_print()
        f.write(str(trial)+","+str(iters)+","+str(maxTime)+","+str(curCost)+'\n')
    f.close()

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def drawGL(self,res=0.01):
        numdivs = int(math.ceil(self.radius*math.pi*2/res))
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*self.center)
        for i in xrange(numdivs+1):
            u = float(i)/float(numdivs)*math.pi*2
            glVertex2f(self.center[0]+self.radius*math.cos(u),self.center[1]+self.radius*math.sin(u))
        glEnd()


class Box:
    def __init__(self,x1=0,y1=0,x2=0,y2=0):
        self.bmin = (min(x1,x2),min(y1,y2))
        self.bmax = (max(x1,x2),max(y1,y2))
        
    def contains(self,point):
        return self.bmin[0] <= point[0] <= self.bmax[0] and self.bmin[1] <= point[1] <= self.bmax[1]

    def drawGL(self):
        glBegin(GL_QUADS)
        glVertex2f(*self.bmin)
        glVertex2f(self.bmax[0],self.bmin[1])
        glVertex2f(*self.bmax)
        glVertex2f(self.bmin[0],self.bmax[1])
        glEnd()


class Geometric2DCSpace (BoxConfigurationSpace):
    def __init__(self):
        BoxConfigurationSpace.__init__(self,[0,0],[1,1])
        self.obstacles = []
    def addObstacle(self,obs):
        self.obstacles.append(obs)
    def feasible(self,x):
        if not BoxConfigurationSpace.feasible(self,x): return False
        for o in self.obstacles:
            if o.contains(x): return False
        return True

    def toScreen(self,q):
        return (q[0]-self.box.bmin[0])/(self.box.bmax[0]-self.box.bmin[0]),(q[1]-self.box.bmin[1])/(self.box.bmax[1]-self.box.bmin[1])

    def toState(self,x,y):
        return (self.box.bmin[0]+x*(self.box.bmax[0]-self.box.bmin[0]),
                self.box.bmin[1]+y*(self.box.bmax[1]-self.box.bmin[1]))

    def beginDraw(self):
        if self.box.bmin != [0,0] or self.box.bmin != [1,1]:
            glPushMatrix()
            glScalef(1.0/(self.box.bmax[0]-self.box.bmin[0]),1.0/(self.box.bmax[1]-self.box.bmin[1]),1)
            glTranslatef(-self.box.bmin[0],-self.box.bmin[1],0)

    def endDraw(self):
        if self.box.bmin != [0,0] or self.box.bmin != [1,1]:
            glPopMatrix()

    def drawObstaclesGL(self):
        self.beginDraw()
        glColor3f(0.2,0.2,0.2)
        for o in self.obstacles:
            o.drawGL()
        self.endDraw()

    def drawVerticesGL(self,qs):
        self.beginDraw()
        glBegin(GL_POINTS)
        for q in qs:
            glVertex2f(q[0],q[1])
        glEnd()
        self.endDraw()

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        glPointSize(7.0)
        self.drawVerticesGL([q])

    def drawGoalGL(self,goal):
        self.beginDraw()
        if isinstance(goal,NeighborhoodSubset):
            q = goal.c
            glColor3f(1,0,0)
            gldraw.circle(q,goal.r,filled=False)
            glPointSize(7.0)
            glBegin(GL_POINTS)
            glVertex2f(q[0],q[1])
            glEnd()
        elif isinstance(goal,SingletonSubset):
            q = goal.x
            glColor3f(1,0,0)
            glPointSize(7.0)
            glBegin(GL_POINTS)
            glVertex2f(q[0],q[1])
            glEnd()
        else:
            glColor3f(1,0,0)
            glPointSize(7.0)
            glBegin(GL_POINTS)
            for i in xrange(50):
                q = goal.sample()
                glVertex2f(q[0],q[1])
            glEnd()
        self.endDraw()

    def drawInterpolatorGL(self,interpolator):
        self.beginDraw()
        if isinstance(interpolator,LinearInterpolator):
            #straight line paths
            glBegin(GL_LINES)
            glVertex2f(interpolator.a[0],interpolator.a[1])
            glVertex2f(interpolator.b[0],interpolator.b[1])
            glEnd()
        elif isinstance(interpolator,PiecewiseLinearInterpolator):
            glBegin(GL_LINE_STRIP)
            for x in interpolator.path:
                glVertex2f(x[0],x[1])
            glEnd()
        else:
            glBegin(GL_LINE_STRIP)
            for s in xrange(10):
                u = float(s) / (9.0)
                x = interpolator.eval(u)
                glVertex2f(x[0],x[1])
            glEnd()
        self.endDraw()

class DubinsVisualizer:
    def __init__(self,workspace):
        self.base = workspace

    def toScreen(self,q):
        return q[0],q[1]

    def toState(self,x,y):
        return (x,y,0)

    def drawObstaclesGL(self):
        self.base.drawObstaclesGL()

    def drawVerticesGL(self,qs):
        self.base.drawVerticesGL(qs)

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        glPointSize(7.0)
        self.drawVerticesGL([q])
        l = 0.05
        d = (math.cos(q[2]),math.sin(q[2]))
        glBegin(GL_LINES)
        glVertex2f(q[0],q[1])
        glVertex2f(q[0]+l*d[0],q[1]+l*d[1])
        glEnd()

    def drawGoalGL(self,goal):
        self.base.drawGoalGL(goal)
        if isinstance(goal,NeighborhoodSubset):
            l = 0.05
            q = goal.c
            glBegin(GL_LINE_STRIP)
            d = (math.cos(q[2]+goal.r*math.pi*2),math.sin(q[2]+goal.r*math.pi*2))
            glVertex2f(q[0]+d[0]*l,q[1]+d[1]*l)
            glVertex2f(q[0],q[1])
            d = (math.cos(q[2]-goal.r*math.pi*2),math.sin(q[2]-goal.r*math.pi*2))
            glVertex2f(q[0]+d[0]*l,q[1]+d[1]*l)
            glEnd()

    def drawInterpolatorGL(self,interpolator):
        self.base.drawInterpolatorGL(interpolator)

class DoubleIntegratorVisualizer:
    def __init__(self,workspace):
        self.base = workspace

    def toScreen(self,q):
        return q[0],q[1]

    def toState(self,x,y):
        return (x,y,0,0)

    def drawObstaclesGL(self):
        self.base.drawObstaclesGL()

    def drawVerticesGL(self,qs):
        self.base.drawVerticesGL(qs)

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        glPointSize(7.0)
        self.drawVerticesGL([q])
        l = 0.05
        glBegin(GL_LINES)
        glVertex2f(q[0],q[1])
        glVertex2f(q[0]+l*q[2],q[1]+l*q[3])
        glEnd()

    def drawGoalGL(self,goal):
        self.base.drawGoalGL(goal)

    def drawInterpolatorGL(self,interpolator):
        self.base.drawInterpolatorGL(interpolator)


class SO2Geodesic(GeodesicSpace):
    def dimension(self):
        return 1
    def distance(self,a,b):
        return abs(so2.diff(a[0],b[0]))
    def interpolate(self,a,b,u):
        return [so2.interp(a[0],b[0],u)]
    def difference(self,a,b):
        return [so2.diff(a[0],b[0])]
    def integrate(self,x,d):
        return [so2.normalize(x[0]+d[0])]

class SO2Space(GeodesicConfigurationSpace):
    """Space of angles [0,2pi) supporting proper wrapping"""
    def __init__(self):
        self.geodesic = SO2Geodesic()
    def bounds(self):
        return [0],[math.pi*2]
    def sample(self):
        return [random.uniform(0,math.pi*2)]

class SE2Space(MultiGeodesicSpace,MultiConfigurationSpace):
    def __init__(self):
        MultiGeodesicSpace.__init__(self,CartesianConfigurationSpace(2),SO2Space())
        MultiConfigurationSpace.__init__(self,CartesianConfigurationSpace(2),SO2Space())
        self.componentWeights = [1,1]
    def setTranslationDistanceWeight(self,value):
        self.componentWeights[0] = value
    def setRotationDistanceWeight(self,value):
        self.componentWeights[1] = value

class DubinsCarInterpolator(Interpolator):
    def __init__(self,space,x,u):
        Interpolator.__init__(self,x,space.nextState(x,u))
        self.space = space
        self.x = x
        self.control = u
    def length(self):
        return abs(self.control[0])
    def eval(self,u):
        return self.space.nextState(self.x,[self.control[0]*u,self.control[1]])
    def split(self,u):
        return DubinsCarInterpolator(self.space,self.x,[self.control[0]*u,self.control[1]]),DubinsCarInterpolator(self.space,self.eval(u),[self.control[0]*(1.0-u),self.control[1]])

class DubinsCarSpace (ControlSpace):
    """u = (distance,turnRate)"""
    def __init__(self,cspace):
        self.space = cspace
        self.space.setDistanceWeights([1,0.5/math.pi])
        self.controls = BoxSet([-1,-1],[1,1])
    def setDistanceBounds(self,minimum,maximum):
        self.controls.bmin[0] = minimum
        self.controls.bmax[0] = maximum
    def setTurnRateBounds(self,minimum,maximum):
        self.controls.bmin[1] = minimum
        self.controls.bmax[1] = maximum
    def configurationSpace(self):
        return self.space
    def controlSet(self,x):
        return self.controls
    def nextState(self,x,u):
        pos = [x[0],x[1]]
        fwd = [math.cos(x[2]),math.sin(x[2])]
        right = [-fwd[1],fwd[0]]
        phi = u[1]
        d = u[0]
        if abs(phi)<1e-8:
            newPos = vectorops.madd(pos,fwd,d)
            return newpos + [x[2]]
        else:
            #rotate about a center of rotation, with radius 1/phi
            cor = vectorops.madd(pos,right,1.0/phi)
            sign=cmp(d*phi,0)
            d = abs(d)
            phi = abs(phi)
            theta=0
            thetaMax=d*phi
            newpos = vectorops.add(so2.apply(sign*thetaMax,vectorops.sub(pos,cor)),cor)
            return newpos + [so2.normalize(x[2]+sign*thetaMax)]
    def interpolator(self,x,u):
        return DubinsCarInterpolator(self,x,u)
    def connection(self,x,y):
        #TODO: dubins curves
        return None


class PlanningProblem:
    def __init__(self,space,
                 start=None,
                 goal=None,
                 objective=None,
                 visualizer=None,
                 heuristic=None,
                 costLowerBound=None,
                 goalRadius=None,
                 euclidean=False):
        self.space = space
        if isinstance(space,ControlSpace):
            self.controlSpace = space
            self.configurationSpace = space.configurationSpace()
        else:
            self.controlSpace = None
            self.configurationSpace = space
        if goalRadius != None and isinstance(goal,(list,tuple)):
            goal = NeighborhoodSubset(self.configurationSpace,goal,goalRadius)
        self.start = start
        self.goal = goal
        self.objective = objective
        self.visualizer = visualizer
        self.heuristic = heuristic
        self.costLowerBound = costLowerBound
        self.euclidean = euclidean
    def planner(self,type,**params):
        d = euclideanMetric if self.euclidean else self.configurationSpace.distance
        return makePlanner(type,space=self.space,
                           start=self.start,goal=self.goal,
                           objective=self.objective,
                           heuristic=self.heuristic,
                           metric=d,
                           costLowerBound=self.costLowerBound,
                           **params)


class CSpaceObstacleProgram(GLProgram):
    def __init__(self,problem,**plannerParams):
        GLProgram.__init__(self)
        self.problem = problem

        if 'type' in plannerParams:
            self.plannerType = plannerParams['type']
            del plannerParams['type']
        else:
            self.plannerType = 'r-rrt'

        self.planner = problem.planner(self.plannerType,**plannerParams)
        self.path = None
        self.G = None

    def mousefunc(self,button,state,x,y):
        if state == 0:
            if hasattr(self.planner,'nextSampleList'):
                self.planner.nextSampleList.append(self.problem.visualizer.toState(float(x)/self.width,float(y)/self.height))
                print self.planner.nextSampleList
            self.refresh()
        
    def keyboardfunc(self,key,x,y):
        if key==' ':
            print "Planning 1..."
            self.planner.planMore(1)
            self.path = self.planner.getPath()
            self.G = self.planner.getRoadmap()
            self.planner.stats.pretty_print()
            glutPostRedisplay()
        elif key=='p':
            print "Planning 1000..."
            self.planner.planMore(1000)
            self.path = self.planner.getPath()
            self.G = self.planner.getRoadmap()
            self.planner.stats.pretty_print()
            glutPostRedisplay()
        elif key=='r':
            print "Resetting planner"
            self.planner.reset()
            self.path = None
            self.G = self.planner.getRoadmap()
            glutPostRedisplay()
        elif key=='t':
            maxTime = 30
            testPlanner(self.planner,10,maxTime,filename[self.plannerType]+'.csv')
       
    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0,1,1,0,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glEnable(GL_POINT_SMOOTH)
        glDisable(GL_DEPTH_TEST)

        glDisable(GL_LIGHTING)
        self.problem.visualizer.drawObstaclesGL()

        if hasattr(self.planner,'nextSampleList'):
            for p in self.planner.nextSampleList:
                self.problem.visualizer.drawRobotGL(p)

        if self.G:            #draw graph
            V,E = self.G
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0,0,0,0.5)
            glPointSize(3.0)
            self.problem.visualizer.drawVerticesGL(V)
            glColor4f(0.5,0.5,0.5,0.5)
            for (i,j,e) in E:
                interpolator = self.problem.space.interpolator(V[i],e)
                self.problem.visualizer.drawInterpolatorGL(interpolator)
            glDisable(GL_BLEND)

        if self.path:
            #draw path
            glColor3f(0,0.75,0)
            glLineWidth(5.0)
            for q,u in zip(self.path[0][:-1],self.path[1]):
                interpolator = self.problem.space.interpolator(q,u)
                self.problem.visualizer.drawInterpolatorGL(interpolator)
            glLineWidth(1)
            for q in self.path[0]:
                self.problem.visualizer.drawRobotGL(q)
        #else:
        self.problem.visualizer.drawRobotGL(self.problem.start)
        self.problem.visualizer.drawGoalGL(self.problem.goal)

def runVisualizer(problem,**plannerParams):
   
    program = CSpaceObstacleProgram(problem,**plannerParams)
    program.width = program.height = 640
    program.run()

def circleTest():
    space = Geometric2DCSpace()
    space.addObstacle(Circle(0.5,0.4,0.39))
    start=[0.06,0.25]
    goal=[0.94,0.25]
    objective = PathLengthObjectiveFunction()
    goalRadius = 0.1
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)


def rrtChallengeTest():
    w = 0.03
    eps = 0.01
    space = Geometric2DCSpace()
    space.box = BoxSet([0,0],[1,w*2+eps])
    space.addObstacle(Box(0,w*2+eps,1,1))
    space.addObstacle(Box(w,w,1,w+eps))
    start=[1-w*0.5,w+eps+w*0.5]
    goal=[1-w*0.5,w*0.5]
    goalRadius = w*0.5
    objective = PathLengthObjectiveFunction()
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)

def kinkTest():
    space = Geometric2DCSpace()
    w = 0.02
    space.addObstacle(Box(0.3,0,0.5+w*0.5,0.2-w*0.5))
    space.addObstacle(Box(0.5+w*0.5,0,0.7,0.3-w*0.5))
    space.addObstacle(Box(0.3,0.2+w*0.5,0.5-w*0.5,0.7))
    space.addObstacle(Box(0.5-w*0.5,0.3+w*0.5,0.7,0.7))
    start=[0.06,0.25]
    goal=[0.94,0.25]
    goalRadius = 0.1
    objective = PathLengthObjectiveFunction()
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)

def bugtrapTest():
    space = Geometric2DCSpace()
    w = 0.1
    space.addObstacle(Box(0.55,0.25,0.6,0.75))
    space.addObstacle(Box(0.15,0.25,0.55,0.3))
    space.addObstacle(Box(0.15,0.7,0.55,0.75))
    space.addObstacle(Box(0.15,0.25,0.2,0.5-w*0.5))
    space.addObstacle(Box(0.15,0.5+w*0.5,0.2,0.75))
    start=[0.5,0.5]
    goal=[0.65,0.5]
    goalRadius = 0.1
    objective = PathLengthObjectiveFunction()
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)


def circleDoubleIntegratorTest():
    cspace = Geometric2DCSpace()
    #cspace.addObstacle(Circle(0.5,0.4,0.39))
    vspace = BoxConfigurationSpace([-1,-1],[1,1])
    aspace = BoxConfigurationSpace([-5,-5],[5,5])
    start = [0.06,0.25,0,0]
    goal = [0.94,0.25,0,0]
    objective = TimeObjectiveFunction()
    goalRadius = 0.2
    controlSpace = CVControlSpace(cspace,vspace,aspace,dt=0.05,dtmax=0.5)
    return PlanningProblem(controlSpace,start,goal,
                           objective=objective,
                           visualizer=DoubleIntegratorVisualizer(cspace),
                           goalRadius = goalRadius,
                           euclidean = True)

class DubinsCarDistanceObjectiveFunction(ObjectiveFunction):
    def __init__(self,n):
        self.n = n
    def incremental(self,x,u):
        return sum(abs(u[i*2]) for i in xrange(self.n))

def dubinsCarTest():
    cspace = Geometric2DCSpace()
    vspace = BoxConfigurationSpace([-1,-1],[1,1])
    start = [0.5,0.3,0]
    goal = [0.5,0.7,0]
    controlSpace = DubinsCarSpace(MultiConfigurationSpace(cspace,SO2Space()))
    controlSpace.setDistanceBounds(-0.25,0.25)
    controlSpace.setTurnRateBounds(-3.14,3.14)
    numControlsPerSample = 1
    if numControlsPerSample > 1:
        controlSpace = RepeatedControlSpace(controlSpace,numControlsPerSample)
    objective = DubinsCarDistanceObjectiveFunction(numControlsPerSample)
    goalRadius = 0.1
    return PlanningProblem(controlSpace,start,goal,
                           objective=objective,
                           visualizer=DubinsVisualizer(cspace),
                           goalRadius=goalRadius,
                           costLowerBound=lambda x,y:vectorops.distance(x[:2],y[:2]))

def dubinsTest2():
    cspace = Geometric2DCSpace()
    cspace.addObstacle(Box(0.5,0.4,0.9,0.7))
    cspace.addObstacle(Box(0.0,0.4,0.2,0.7))
    cspace.addObstacle(Box(0.0,0.9,1.0,1.0))
    vspace = BoxConfigurationSpace([-1,-1],[1,1])
    start = [0.8,0.3,0]
    goal = [0.8,0.8,math.pi]
    controlSpace = DubinsCarSpace(MultiConfigurationSpace(cspace,SO2Space()))
    controlSpace.setDistanceBounds(-0.25,0.25)
    controlSpace.setTurnRateBounds(-3.14,3.14)
    numControlsPerSample = 1
    if numControlsPerSample > 1:
        controlSpace = RepeatedControlSpace(controlSpace,numControlsPerSample)
    objective = DubinsCarDistanceObjectiveFunction(numControlsPerSample)
    goalRadius = 0.1
    return PlanningProblem(controlSpace,start,goal,
                           objective=objective,
                           visualizer=DubinsVisualizer(cspace),
                           goalRadius=goalRadius,
                           costLowerBound=lambda x,y:vectorops.distance(x[:2],y[:2]))


class PendulumVisualizer:
    def __init__(self,pendulum):
        self.pendulum = pendulum

    def drawObstaclesGL(self):
        return

    def toState(self,x,y):
        return [(x*2*math.pi-math.pi)%(math.pi*2),y*(self.pendulum.omega_max-self.pendulum.omega_min)+self.pendulum.omega_min]

    def toScreen(self,q):
        return [((q[0]+math.pi)/math.pi*0.5)%1,(q[1]-self.pendulum.omega_min)/(self.pendulum.omega_max-self.pendulum.omega_min)]

    def drawVerticesGL(self,qs):
        glBegin(GL_POINTS)
        for q in qs:
            glVertex2f(*self.toScreen(q))
        glEnd()

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        glPointSize(7.0)
        glBegin(GL_POINTS)
        glVertex2f(*self.toScreen(q))
        glEnd()

    def drawGoalGL(self,goal):
        bmin = self.toScreen(self.pendulum.goalmin)
        bmax = self.toScreen(self.pendulum.goalmax)
        glColor3f(1,0,0)
        glLineWidth(2.0)
        glBegin(GL_LINE_STRIP)
        glVertex2f(1,bmin[1])
        glVertex2f(bmin[0],bmin[1])
        glVertex2f(bmin[0],bmax[1])
        glVertex2f(1,bmax[1])
        glEnd()
        glBegin(GL_LINE_STRIP)
        glVertex2f(0,bmax[1])
        glVertex2f(bmax[0],bmax[1])
        glVertex2f(bmax[0],bmin[1])
        glVertex2f(0,bmin[1])
        glEnd()
        glLineWidth(1.0)

    def drawInterpolatorGL(self,interpolator):
        if isinstance(interpolator,PiecewiseLinearInterpolator):
            glBegin(GL_LINE_STRIP)
            for x in interpolator.path:
                glVertex2f(*self.toScreen(x))
            glEnd()
        else:
            glBegin(GL_LINE_STRIP)
            for s in xrange(10):
                u = float(s) / (9.0)
                x = interpolator.eval(u)
                glVertex2f(*self.toScreen(x))
            glEnd()

class PendulumGoalSet(Set):
    def __init__(self,bmin,bmax):
        self.bmin = bmin
        self.bmax = bmax
    def bounded(self):
        return True
    def sample(self):
        return [random.uniform(self.bmin[0],self.bmax[0]),random.uniform(self.bmin[1],self.bmax[1])]
    def contains(self,x):
        return (0 <= so2.diff(x[0],self.bmin[0]) <= so2.diff(self.bmax[0],self.bmin[0])) and \
               (self.bmin[1] <= x[1] <= self.bmax[1])


class Pendulum:
    def __init__(self):
        self.theta_min = 0.0
        self.theta_max = 2*math.pi
        self.omega_min = -10.0
        self.omega_max = 10.0
        self.torque_min = -2.0
        self.torque_max = 2.0
        etheta = 0.1
        eomega = 0.5
        self.goalmin = [math.pi-etheta,-eomega]
        self.goalmax = [math.pi+etheta,+eomega]
        self.bangbang = True
        self.g = 9.8
        self.m = 1
        self.L = 1

    def controlSpace(self):
        return LambdaKinodynamicSpace(self.configurationSpace(),self.controlSet(),self.derivative,0.01,0.5)

    def controlSet(self):
        if self.bangbang:
            return FiniteSet([[self.torque_min],[0],[self.torque_max]])
        else:
            return BoxSet([self.torque_min],[self.torque_max])

    def startState(self):
        return [0.0,0.0]

    def configurationSpace(self):
        res =  MultiConfigurationSpace(SO2Space(),BoxConfigurationSpace([self.omega_min],[self.omega_max]))
        res.setDistanceWeights([1.0/(2*math.pi),1.0/(self.omega_max-self.omega_min)])
        return res

    def derivative(self,x,u):
        theta,omega = x
        return [omega,(u[0]/(self.m*self.L**2)-self.g*math.sin(theta)/self.L)]

    def dynamics(self):
        return PendulumDynamics(self)
    
    def goalSet(self):
        return PendulumGoalSet(self.goalmin,self.goalmax)

def pendulumTest():
    p = Pendulum()
    objective = TimeObjectiveFunction()
    return PlanningProblem(p.controlSpace(),p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=PendulumVisualizer(p))

class FlappyControlSpace(ControlSpace):
    def __init__(self,flappy):
        self.flappy = flappy
    def configurationSpace(self):
        return self.flappy.configurationSpace()
    def controlSet(self,x):
        return MultiSet(BoxSet([0],[self.flappy.time_range]),self.flappy.controlSet())
    def nextState(self,x,u):
        return self.eval(x,u,1.0)
    def eval(self,x,u,amount):
        x_i,y_i,vy_i = x
        t,thrust = u
        tc = t*amount
        #instantaneous version
        #net_acceler = self.flappy.gravity
        #vy_i += thrust*self.flappy.thrust
        #yilun's version
        net_acceler = self.flappy.gravity + thrust*self.flappy.thrust
        return [x_i+self.flappy.v_x*tc, 
                y_i+vy_i*tc+0.5*net_acceler*(tc**2), 
                vy_i+net_acceler*tc]

    def interpolator(self,x,u):
        return LambdaInterpolator(lambda s:self.eval(x,u,s),self.configurationSpace(),10)

class Flappy:
    def __init__(self):
        self.x_range = 1000
        self.y_range = 600
        self.min_altitude = 300
        self.max_velocity = 40

        self.start_state = [50, 250, 0]
        self.goal_state = [950, 200, 0]
        self.goal_radius = 50
        self.time_range = 20
        #u = lambda:round(random.random())

        self.obstacles = []
        self.obstacles = [(175, 450, 50, 100), (175, 0, 50, 100), (175, 150, 50, 200), 
             (375,200, 50, 300),(375, 0, 50, 100), 
             (575, 500, 50, 100), (575, 0, 50, 125), (575, 200, 50, 200), 
             (775, 200, 50, 400)]

        self.v_x = 5
        self.gravity = -1
        self.thrust = 4

    def controlSet(self):
        return FiniteSet([[0],[1]])

    def controlSpace(self):
        return FlappyControlSpace(self)

    def workspace(self):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0,0]
        wspace.box.bmax = [self.x_range,self.y_range]
        for o in self.obstacles:
            wspace.addObstacle(Box(o[0],o[1],o[0]+o[2],o[1]+o[3]))
        return wspace
    
    def configurationSpace(self):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0,0]
        wspace.box.bmax = [self.x_range,self.y_range]
        for o in self.obstacles:
            wspace.addObstacle(Box(o[0],o[1],o[0]+o[2],o[1]+o[3]))
        res =  MultiConfigurationSpace(wspace,BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]))
        return res

    def startState(self):
        return self.start_state

    def goalSet(self):
        r = self.goal_radius
        return BoxSubset(self.configurationSpace(),
                         [self.goal_state[0]-r,self.goal_state[1]-r,-float('inf')],
                         [self.goal_state[0]+r,self.goal_state[1]+r,float('inf')])


class FlappyObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,flappy,timestep=0.2):
        self.flappy = flappy
        self.space = flappy.controlSpace()
        self.timestep = timestep
    def incremental(self,x,u):
        e = self.space.interpolator(x,u)
        tmax = u[0]
        t = 0
        c = 0
        while t < tmax:
            t = min(tmax,t+self.timestep)
            xnext = e.eval(t / tmax)
            c += vectorops.distance(x,xnext)
            x = xnext
        return c


def flappyTest():
    p = Flappy()
    objective = FlappyObjectiveFunction(p)
    return PlanningProblem(p.controlSpace(),p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)






def testPlannerDefault(problem,problemName,maxTime,plannerType,**plannerParams):
    #checker = EpsilonEdgeChecker(problem.configurationSpace,0.01)
    print "Planning with",plannerType,'on problem',problemName
    planner = problem.planner(plannerType,**plannerParams)
    testPlanner(planner,10,maxTime,problemName+"/"+filename[plannerType]+'.csv')


all_planners = ['ao-est','ao-rrt','r-est','r-est-prune','r-rrt','r-rrt-prune','rrt*','anytime-rrt','stable-sparse-rrt']
rrt_planners = ['ao-rrt','anytime-rrt','r-rrt','r-rrt-prune','stable-sparse-rrt']
est_planners = ['ao-est','r-est','r-est-prune']

all_problems = {'Kink':kinkTest(),
                'Bugtrap':bugtrapTest(),
                'Dubins':dubinsCarTest(),
                'Dubins2':dubinsTest2(),
                'Flappy':flappyTest(),
                'DoubleIntegrator':circleDoubleIntegratorTest(),
                'Pendulum':pendulumTest()}

defaultParameters = {'maxTime':30}
customParameters = {'Kink':{'maxTime':20,'nextStateSamplingRange':0.15},
                    'Bugtrap':{'maxTime':20,'nextStateSamplingRange':0.15},
                    'Pendulum':{'edgeCheckTolerance':0.1},
                    'Flappy':{'edgeCheckTolerance':4}
                    }

def parseParameters(problem,planner):
    global defaultParameters,customParameters
    params = copy.deepcopy(defaultParameters)
    if problem in customParameters:
        params.update(customParameters[problem])
    if '(' in planner:
        #parse out key=value,... string
        name,args = planner.split('(',1)
        if args[-1] != ')':
            raise ValueError("Planner string expression must have balanced parenthesis, i.e.: func ( arglist )")
        args = args[:-1]
        args = args.split(',')
        for arg in args:
            kv = arg.split("=")
            if len(kv) != 2:
                raise ValueError("Unable to parse argument "+arg)
            try:
                params[kv[0]] = int(kv[1])
            except ValueError:
                try:
                    params[kv[0]] = float(kv[1])
                except ValueError:
                    params[kv[0]] = kv[1]
        planner = name
    return planner,params

def runTests(problems = None,planners = None):
    global all_planners,all_problems
    if planners == None or planners == 'all' or planners[0] == 'all':
        planners = all_planners

    if problems == None or problems == 'all' or problems[0] == 'all':
        problems = all_problems.keys()

    for prname in problems:
        pr = all_problems[prname]
        for p in planners:
            p,params = parseParameters(prname,p)
            maxTime = params['maxTime']
            del params['maxTime']
            if pr.controlSpace != None and p in kinematicPlanners:
                #p does not support differentially constrained problems
                continue
            testPlannerDefault(pr,prname,maxTime,p,**params)
            print "Finished test on problem",prname,"with planner",p
            print "Parameters:"
            for (k,v) in params.iteritems():
                print " ",k,":",v
    return

def runViz(problem,planner):
    #runVisualizer(rrtChallengeTest(),type=planner,nextStateSamplingRange=0.15,edgeCheckTolerance = 0.005)
    planner,params = parseParameters(problem,planner)
    if 'maxTime' in params:
        del params['maxTime']
    
    print "Planning on problem",problem,"with planner",planner
    print "Parameters:"
    for (k,v) in params.iteritems():
        print " ",k,":",v
    runVisualizer(all_problems[problem],type=planner,**params)
    
if __name__=="__main__":
    if len(sys.argv) > 1:
        print "Testing problems",sys.argv[1],"with planners",sys.argv[2:]
        runTests(problems=[sys.argv[1]],planners=sys.argv[2:])
    else:
        #runViz('Dubins','ao-rrt(numControlSamples=10,nearestNeighborMethod=bruteforce)')
        runViz('Flappy','anytime-rrt')
        runViz('Pendulum','stable-sparse-rrt(selectionRadius=0.2,witnessRadius=0.03)')
