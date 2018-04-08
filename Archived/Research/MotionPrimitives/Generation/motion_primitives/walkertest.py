from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import bisect
import sys
import math
from robot import *
from robot import se3,vectorops
from robot.glprogram import GLRealtimeProgram
from optimization.control import optimize,walker,search
import random
import numpy as np
import cPickle

class GLWalkerTest(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self,"GLTest")
        params = dict()
        params['name'] = 'walker'
        params['simfile'] = 'walker/step_-0.5.xml'
        params['goal'] = (1.0,0.0,0.5)
        params['effortcost'] = 1.0/600*0.1
        params['horizon'] = 20
        self.problem = walker.genProblem(params)
        self.sim = self.problem.f.sim
        self.world = self.sim.getWorld()
        print "Create RRT"
        self.rrt = search.RRTSearch(self.problem,self.problem.xspace)
        self.solution = None

        self.animate = False
        self.animateTime = 0
        self.animation = []
        self.keyframes = []
        
    def keyboardfunc(self,key,x,y):
        if key == 'p':
            print "Planning..."
            n = self.rrt.planMinimum(100)
            if n != None:
                self.solution = self.rrt.solutionPath(self.rrt.optimalNode())
                qopt = self.solution[0][-1].q
                print "Optimal:",qopt[0:3],"distance",vectorops.distance(qopt[0:3],self.problem.params['goal'])
                print "Distances:",[vectorops.distance(x.q[0:3],self.problem.params['goal']) for x in self.solution[0]]
                self.sim.setState(self.solution[0][-1].state)
            print 'tree size',len([x for x in self.rrt.root.dfsiter()])
            print 'xInfeasible',self.rrt.xInfeasible
            print 'uInfeasible',self.rrt.uInfeasible
            glutPostRedisplay()
        elif key == 'o':
            print "Optimizing..."
            if self.solution != None:
                opt = walker.adaptOptimizer(self.problem,None,self.solution)
                opt.solution = self.solution
                for iters in xrange(50):
                    opt.step(iters)
                self.solution = opt.getSolution()
                print "Distances:",[vectorops.distance(x.q[0:3],self.problem.params['goal']) for x in self.solution[0]]
                glutPostRedisplay()
        elif key == 'a':
            self.animate = not self.animate
            self.animateTime = 0
            if self.animate:
                self.animation = self.generateAnimation(self.solution,0.02)
            glutPostRedisplay()
        elif key == 's':
            print "Dumping to walker_soln.dat"
            with open('walker_soln.dat','w') as f:
                cPickle.dump(self.solution,f)
        elif key == 'l':
            print "Loading from walker_soln.dat"
            with open('walker_soln.dat','r') as f:
                self.solution = cPickle.load(f)
        elif key == 'r':
            #pick a random solution
            print "Picking a random solution"
            nodes = [n for n in self.rrt.root.dfsiter()]
            self.solution = self.rrt.solutionPath(random.choice(nodes))
            glutPostRedisplay()
        elif key == 'g':
            print "Testing global optimizer"
            opt = walker.globalOptimizer(self.problem)
            try:
                opt.run()
            except KeyboardInterrupt:
                print "Breaking global optimization and getting best solution"
                if opt.getSolution()==None:
                    opt.solution = opt.localOpt.getSolution()
                pass
            if opt.getSolution()!=None:
                self.solution = opt.getSolution()
            glutPostRedisplay()

    def drawEdge(self,px,nx):
        points = [(5,(0,0,0))]
        ppoints = []
        npoints = []
        self.sim.setState(px.state)
        robot = self.world.robot(0)
        ppoints = [se3.apply(self.sim.getBody(robot.getLink(pt[0])).getTransform(),pt[1]) for pt in points]
        self.sim.setState(nx.state)
        npoints = [se3.apply(self.sim.getBody(robot.getLink(pt[0])).getTransform(),pt[1]) for pt in points]
        colors = [(1,1,0)]
        glBegin(GL_LINES)
        for (a,b,c) in zip(ppoints,npoints,colors):
            glColor3f(*c)
            glVertex3f(*a)
            glVertex3f(*b)
        glEnd()

    def display(self):
        self.world.drawGL()
        if not self.animate:
            for n in self.rrt.root.dfsiter():
                if n.parent != None:
                    self.drawEdge(n.parent.data.x,n.data.x)
        else:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0.5,0.5,0.5,0.2])
            for k in self.keyframes:
                self.sim.setState(k)
                self.sim.updateWorld()
                self.world.robot(0).drawGL(False)
        glLineWidth(4.0)
        if self.solution != None:
            for i,j in zip(self.solution[0][0:-1],self.solution[0][1:]):
                self.drawEdge(i,j)
        glLineWidth(1.0)

    def generateAnimation(self,path,dt):
        if path == None: return None
        animation = []
        self.keyframes = []
        if len(path[1])==0:
            self.keyframes = [path[0][0].state]
            return [(0,path[0][0].state)]
        t = 0.0
        ut = 0.0
        self.sim.setState(path[0][0].state)
        animation.append((0,self.sim.getState()))
        for x,u in zip(path[0][:-1],path[1]):
            udt = u[0]
            n = (len(u)-1)/2
            self.keyframes.append(x.state)
            #uncomment this line if you care about ODE nondeterminism
            self.sim.setState(x.state)
            self.problem.f.setSimControl(x,u)
            while t+dt < ut+udt:
                self.sim.simulate(dt)
                t += dt
                animation.append((t,self.sim.getState()))
            if t < ut+udt:
                self.sim.simulate(ut+udt-t)
                t = ut+udt
                animation.append((t,self.sim.getState()))
            ut += udt
        return animation

    def idle(self):
        if self.animate:
            if self.animation:
                if len(self.animation)==1:
                    self.sim.setState(self.animation[0][1])
                else:
                    tloop = self.animateTime % self.animation[-1][0]
                    index = bisect.bisect_right(self.animation,(tloop,None))
                    if index+1 < len(self.animation):
                        u = (tloop - self.animation[index][0]) / (self.animation[index+1][0]-self.animation[index][0])
                    self.sim.setState(self.animation[index][1])
                self.animateTime += 1.0/30.0
            glutPostRedisplay()

program = GLWalkerTest()
program.run()
