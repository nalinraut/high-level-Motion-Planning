from numpy import poly1d
from numpy import arange
from numpy import asarray
import math
import functools
from trajectory import *

def interp(q1,q2,u):
    """Interpolates a segment between the vectors q1 and q2 using the
    parameter u.  Can also be used for extrapolation"""
    res = [0]*len(q1)
    for i in range(len(q1)):
        res[i] = q1[i]+u*(q2[i]-q1[i])
    return res

def finterp(q1,q2,ut,t):
    return interp(q1,q2,ut(t))

def NonlinearInterpolation(q1,q2,ut):
    return functools.partial(finterp,q1,q2,ut)

def choose(n,k):
        """Returns (n choose k).
        """
        assert(k<=n and k==int(k) and n==int(n)),"n=%f, k=%f"%(n,k)
        k = min(k,n-k)
        if k==0:
                c = 1
        else:
                c = n
                for i in xrange(1,k):
                        c*=n-i
                for i in xrange(1,k):
                        c//=i+1
        return c 

def shiftPoly(p,dx):
    """Returns the polynomial representing p(x+dx)"""
    if hasattr(p,'order'):
        #assume it's an poly1d
        return shiftPoly(p.c,dx)
    #order is m
    #order of entry i is m-i
    #entry for order k is m-k
    m = len(p)-1
    p2 = [0]*len(p)
    for i in range(len(p)):
        k = m-i
        for j in range(k+1):
            #get the x^j'th coefficient 
            p2[i+j] += p[i]*choose(k,j)*pow(dx,j)
    return poly1d(p2)

class ParabolicRamp1D:
    @staticmethod
    def __makePLP(x0,dx0,a1,v,a2,t1,t2,ttotal):
        s1 = poly1d([a1*0.5,dx0,x0])
        x1 = s1(t1)
        s2 = shiftPoly(poly1d([v,x1]), -t1)
        x2 = s2(t2)
        v2 = s2.deriv()(t2)
        s3 = shiftPoly(poly1d([a2*0.5,v2,x2]),-t2)
        return PiecewiseTrajectory([s1,s2,s3],[0,t1,t2,ttotal])

    @staticmethod
    def __makePP(x0,dx0,a1,a2,t1,ttotal):
        s1 = poly1d([a1*0.5,dx0,x0])
        x1 = s1(t1)
        v1 = s1.deriv()(t1)
        s2 = shiftPoly(poly1d([a2*0.5,v1,x1]),-t1)
        return PiecewiseTrajectory([s1,s2],[0,t1,ttotal])

    @staticmethod
    def solveBrakingTrajectory(x0,v0,dmax):
        if v0 > 0:
            return PiecewiseTrajectory([poly1d([-dmax*0.5,v0,x0])],[0,v0/dmax])
        else:
            return PiecewiseTrajectory([poly1d([dmax*0.5,v0,x0])],[0,-v0/dmax])

    @staticmethod
    def solveMinTime(xv0,xv1,amax,vmax,dmax=None):
        if dmax==None:
            dmax = amax
        x0 = xv0
        v0 = 0
        x1 = xv1
        v1 = 0
        if hasattr(xv0,"__iter__"):
            x0 = xv0[0]
            v0 = xv0[1]
        if hasattr(xv1,"__iter__"):
            x1 = xv1[0]
            v1 = xv1[1]

        #create new parabolic ramp with these characteristics
        if v0 != 0 or v1 != 0:
            raise ValueError("Haven't implemented nonzero velocities yet")
        plp = ParabolicRamp1D. __solveMinTimePLP((x0,v0),(x1,v1),amax,vmax,dmax)
        if plp == None:
            (a1,t1,a2,ttotal) = ParabolicRamp1D.__solveMinTimePP((x0,v0),(x1,v1),amax,dmax)
            return ParabolicRamp1D.__makePP(x0,v0,a1,a2,t1,ttotal)
        else:
            (a1,t1,v,t2,a2,ttotal) = plp
            return ParabolicRamp1D.__makePLP(x0,v0,a1,v,a2,t1,t2,ttotal)

    @staticmethod
    def solveMinAccel(xv0,xv1,endtime,amax,vmax,dmax=None):
        if dmax==None:
            dmax = amax
        x0 = xv0
        v0 = 0
        x1 = xv1
        v1 = 0
        if hasattr(xv0,"__iter__"):
            x0 = xv0[0]
            v0 = xv0[1]
        if hasattr(xv1,"__iter__"):
            x1 = xv1[0]
            v1 = xv1[1]

        #create new parabolic ramp with these characteristics
        if v0 != 0 or v1 != 0:
            raise ValueError("Haven't implemented nonzero velocities yet")
        plp = ParabolicRamp1D. __solveMinAccelPLP((x0,v0),(x1,v1),endtime,amax,vmax,dmax)
        if plp == None:
            (a1,t1,a2,ttotal) = ParabolicRamp1D.__solveMinAccelPP((x0,v0),endTime,(x1,v1),amax,dmax)
            return ParabolicRamp1D.__makePP(x0,v0,a1,a2,t1,ttotal)
        else:
            (a1,t1,v,t2,a2,ttotal) = plp
            return ParabolicRamp1D.__makePLP(x0,v0,a1,v,a2,t1,t2,ttotal)            

    @staticmethod
    def __solveMinTimePP(xv0,xv1,amax,dmax):
        if xv0[1] != 0 or xv1[1] != 0:
            raise ValueError("Haven't implemented nonzero velocities yet") 
        if xv1[0] < xv0[0]:
            #swap
            (a1,t1,a2,ttotal) = ParabolicRamp1D.__solveMinTimePP((-xv0[0],-xv0[1]),(-xv1[0],-xv1[1]),amax,dmax)
            return (-a1,t1,-a2,ttotal)
        # y1(t) = x0 + t^2 a/2
        # y2(t) = x1 - (T-t)^2 d/2
        # y1'(t1) = y2'(t1) => a t1 = (T-t1)d
        #        => (a+d)/d t1 = T => T-t1 = a/d t1
        # y1(t1) = y2(t1) => x0 + t1^2 a/2 = x1 - (T-t)^2 d/2
        #        => t1^2 (a/2 + a^2/2d) = x1 - x0
        t1sq = (xv1[0]-xv0[0])*2.0/(amax + amax*amax/dmax)
        t1 = math.sqrt(t1sq)
        ttotal = (amax+dmax)/dmax*t1
        #print("a1=%f, t1=%f, a2=%f, T=%f"%(amax,t1,-dmax,ttotal))
        return (amax,t1,-dmax,ttotal)

    @staticmethod
    def __solveMinTimePLP(xv0,xv1,amax,vmax,dmax):
        if xv0[1] != 0 or xv1[1] != 0:
            raise ValueError("Haven't implemented nonzero velocities yet") 
        if xv1[0] < xv0[0]:
            #swap
            (a1,t1,v,t2,a2,ttotal) = ParabolicRamp1D.__solveMinTimePLP((-xv0[0],-xv0[1]),(-xv1[0],-xv1[1]),amax,vmax,dmax)
            return (-a1,t1,-v,t2,-a2,ttotal)
        t1 = vmax/amax
        Tmt2 = vmax/dmax
        t2mt1 = (xv1[0]-xv0[0])/vmax - vmax*0.5/amax - vmax*0.5/dmax
        if t2mt1 < 0:
            return None
        #print("a1=%f, t1=%f, v=%f, t2=%f, a2=%f, T=%f"%(amax,t1,vmax,t2mt1+t1,-dmax,Tmt2+t2mt1+t1))
        return (amax,t1,vmax,t2mt1+t1,-dmax,Tmt2+t2mt1+t1)

    @staticmethod
    def __solveMinAccelPP(xv0,xv1,endtime,amax,dmax):
        if xv0[1] != 0 or xv1[1] != 0:
            raise ValueError("Haven't implemented nonzero velocities yet") 
        if xv1[0] < xv0[0]:
            #swap
            (a1,t1,a2,ttotal) = ParabolicRamp1D.__solveMinAccelPP((-xv0[0],-xv0[1]),(-xv1[0],-xv1[1]),endtime,amax,dmax)
            return (-a1,t1,-a2,ttotal)
        # y1(t) = x0 + t^2 a/2
        # y2(t) = x1 - (T-t)^2 d/2
        # y1'(t1) = y2'(t1) => a t1 = (T-t1)d
        # constrain a/d = amax/dmax = y
        #        => (a+d)/d t1 = T => t1 = d/(d+a)*T
        # y1(t1) = y2(t1) => x0 + t1^2 a/2 = x1 - (T-t1)^2 d/2
        t1 = endtime * dmax / (amax+dmax)  #proportional to the ratio of dec/acc
        ttotal = endtime
        a = 2.0*(x1-x0)/(t1*t1*(1.+amax*amax/dmax*dmax))
        d = dmax*a/amax
        #print("a1=%f, t1=%f, a2=%f, T=%f"%(a,t1,-d,ttotal))
        return (a,t1,-d,ttotal)

class ParabolicRampND:
    @staticmethod
    def solveBrakingTrajectory(x0,v0,dmax):
        if len(x0)!=len(v0):
            raise ValueError("Incorrect size of velocities")
        if len(x0)!=len(dmax):
            raise ValueError("Incorrect size of deceleration bound")
        segs = []*len(x0)
        for i in xrange(len(x0)):
            segs[i]=ParabolicRamp1D.solveBrakingTrajectory(x0[i],v0[i],dmax[i])
        return PiecewiseTrajectory.stack(segs)

    @staticmethod
    def solveMinTime(x0,v0,x1,v1,amax,vmax,dmax=None):
        if dmax==None:
            dmax = amax

        mins = [ParabolicRamp1D.solveMinTime((x0[i],v0[i]),(x1[i],v1[i]),amax[i],vmax[i],dmax[i]) for i in xrange(len(x0))]
        mints = [s.endTime for s in mints]
        mint = min(mints)
        imint = mints.index(mint)
        segs = []*len(x0)
        for i in xrange(len(x0)):
            if i != imint:
                segs[i] = ParabolicRamp1D.solveMinAccel((x0[i],v0[i]),(x1[i],v1[i]),mint,amax[i],vmax[i],dmax[i])
            else:
                segs[i] = mins[i]
        return PiecewiseTrajectory.stack(segs)

    @staticmethod
    def solveMinAccel(x0,v0,x1,v1,dt,amax,vmax,dmax=None):
        if dmax==None:
            dmax = amax

        segs = []*len(x0)
        for i in xrange(len(x0)):
            segs[i] = ParabolicRamp1D.solveMinAccel((x0[i],v0[i]),(x1[i],v1[i]),dt,amax[i],vmax[i],dmax[i])
        return PiecewiseTrajectory.stack(segs)

class RampSolver:
    def __init__(self,amax,vmax,dmax=None,type="mintime"):
        self.amax = amax
        self.vmax = vmax
        self.type = type
        if self.type != "mintime" and self.type != "fixedtime":
            raise ValueError("Can only have mintime or fixedtime types")
        if dmax == None:
            self.dmax = amax
        else:
            self.dmax = dmax

    def __call__(self,xv0,xv1,dt=None):
        if hasattr(self.amax,'__iter__'):
            if self.type == "mintime":
                return ParabolicRampND.solveMinTime(xv0[0],xv0[1],xv1[0],xv1[1],amax,vmax,dmax)
            else:
                return ParabolicRampND.solveMinAccel(xv0[0],xv0[1],xv1[0],xv1[1],dt,amax,vmax,dmax)
        else:
            if self.type == "mintime":
                return ParabolicRamp1D.solveMinTime(xv0,xv1,amax,vmax,dmax)
            else:
                return ParabolicRamp1D.solveMinAccel(xv0,xv1,dt,amax,vmax,dmax)


def testRamp():
    f = ParabolicRamp1D.solveMinTime(0.0,1.0,amax=3.0,vmax=5.0,dmax=5.0)
    print("Result: [%f,%f]"%(f.startTime(),f.endTime()))
    plot = open("parabolicramp_test.csv","w")
    #plot.write("#time,value\n")
    #for t in arange(f.startTime(),f.endTime()+1.0,0.001):
    #    plot.write("%f,%f\n"%(t,f(t)))
    #plot.close()
    plot.write("#time,f(t),v1,v2,v3\n")
    for t in arange(f.startTime()-1.0,f.endTime()+1.0,0.001):
        plot.write("%f,%f,"%(t,f(t)))
        plot.write(','.join([str(s(t)) for s in f.segments]))
        plot.write('\n')
    plot.close()

if __name__=="__main__":
    testRamp()
