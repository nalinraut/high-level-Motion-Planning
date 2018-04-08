from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import sys
import math
from robot.glprogram import GLProgram
from optimization.control import optimize,doubleintegrator,search
from optimization import sample,function,condition
import primitivelib
import numpy as np
import os

class BoxCSpace:
    def __init__(self,bounds,weights=None):
        self.bounds = bounds
        if weights == None:
            self.weights = [1.0/(b-a) for (a,b) in bounds]
        else:
            self.weights = weights
        self.norm = 2.0

    def sample(self):
        return sample.BoxSampler(self.bounds)()
        
    def distance(self,a,b):
        if self.norm == 2.0:
            return math.sqrt(sum(wi*(ai-bi)**2 for ai,bi,wi in zip(a,b,self.weights)))
        elif self.norm == 'inf':
            return max(wi*abs(ai-bi) for ai,bi,wi in zip(a,b,self.weights))
        elif self.norm == 1:
            return sum(wi*abs(ai-bi) for ai,bi,wi in zip(a,b,weights))            
        else:
            return pow(sum(wi*pow(ai-bi,self.norm) for ai,bi,wi in zip(a,b,self.weights)),1.0/self.norm)

class OptimalControlProgram(GLProgram):
    def __init__(self):
        GLProgram.__init__(self)
        self.problem = doubleintegrator.makeProblem(1.0,0.1,[1,0],10)
        self.optimizer = optimize.TrajectoryOptimizer(self.problem)
        self.solution = None
        space = BoxCSpace([(0.0,20.0),(-2.0,2.0),(-2.0,2.0),(-2.0,2.0),(-2.0,2.0)])
        self.problem.addXConstraint(function.BoxContains(space.bounds))
        self.rrt = search.RRTSearch(self.problem,space)
        self.rrt.numControlSamples = 10
        
    def keyboardfunc(self,key,x,y):
        if key=='z':
            g = self.problem.params['goal']
            g[1] += 0.1
            self.problem = makeDoubleIntegratorProblem(1.0,0.1,g,10)
            self.optimizer.problem = self.problem
            glutPostRedisplay()
        if key=='i':
            self.solution = self.optimizer.sampleInitialSolution(self.problem.usampler,condition.CountCondition(1000))
            glutPostRedisplay()
        if key=='r':
            if self.solution == None:
                self.solution = self.optimizer.sampleInitialSolution(self.problem.usampler,condition.CountCondition(1000))
            else:
                for iters in xrange(100):
                    res = self.optimizer.randomDescentStep(self.solution,0.01,condition.CountCondition(1000))
                    if res:
                        self.solution = res
                        print "Cost:",self.problem.evalCost(*self.solution)
            glutPostRedisplay()
        if key == 'g':
            if self.solution == None:
                self.solution = self.optimizer.sampleInitialSolution(self.problem.usampler,condition.CountCondition(1000))
            else:
                for iters in xrange(1):
                    res = self.optimizer.gradientDescentStep(self.solution)
                    if res:
                        self.solution = res
                        print "Cost:",self.problem.evalCost(*self.solution)
            glutPostRedisplay()
        if key == 'd':
            if self.solution == None:
                self.solution = self.optimizer.sampleInitialSolution(self.problem.usampler,condition.CountCondition(1000))
            else:
                for iters in xrange(1):
                    res = self.optimizer.ddpStep(self.solution)
                    if res:
                        self.solution = res
                        print "Cost:",self.problem.evalCost(*self.solution)
            glutPostRedisplay()
        if key == 'o':
            self.solution,costs = global_optimize(self.problem,10,100)
            glutPostRedisplay()
        if key == 'p':
            n = self.rrt.planMinimum(100)
            if n != None:
                self.solution = self.rrt.solutionPath(n)
            glutPostRedisplay()
        
    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-2,2,2,-2,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glDisable(GL_LIGHTING)
        glPointSize(5.0)
        glEnable(GL_POINT_SMOOTH)
        glColor3f(1,0,0)
        glBegin(GL_POINTS)
        glVertex2f(*self.problem.params['goal'])
        glEnd()
        if self.solution:
            glColor3f(0,1,0)
            glBegin(GL_LINE_STRIP)
            for q in self.solution[0]:
                glVertex2f(q[1],q[2])
            glEnd()

        glPointSize(3.0)
        glColor3f(0.5,0.5,0.5)
        #glBegin(GL_POINTS)
        #for n in self.rrt.root.dfsiter():
        #    glVertex2f(n.data.x[1],n.data.x[2])
        #glEnd()
        glBegin(GL_LINES)
        for n in self.rrt.root.dfsiter():
            if n.parent:
                glVertex2f(n.data.x[1],n.data.x[2])
                glVertex2f(n.parent.data.x[1],n.parent.data.x[2])
        glEnd()

if __name__=='__main__':
    if len(sys.argv) > 1:
        fdir = 'di_primitives'
        if len(sys.argv) > 2:
            fdir = sys.argv[2]
        if sys.argv[1]=='gen':
            library = PrimitiveLibrary()
            library.load_primitives(fdir)
            print "Saving primitives to",fdir
            library.generate(lambda :sampleDoubleIntegratorProblem([0.0,1.0],[(-1,1),(1,1)],[-0.0,50.0]),100,fdir)
        elif sys.argv[1]=='reopt':
            library = PrimitiveLibrary()
            library.load_primitives(fdir)
            library.reoptimize()
            print "Saving primitives to",fdir
            library.save_primitives(fdir)
        elif sys.argv[1]=='label':
            library = PrimitiveLibrary()
            library.load_primitives(fdir)
            library.label()
            print "Saving costs to",fdir
            library.save_costs(fdir)
        else:
            print "Valid commands are gen and label"
    else:
        program = OptimalControlProgram()
        program.name = "Optimal control test"
        program.width = program.height = 640
        program.run()
    
