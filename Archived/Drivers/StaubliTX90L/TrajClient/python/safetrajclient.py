from trajclient import *
from parabolicramp import *
from numpy import arange
from numpy import array
from numpy import ix_
from numpy import zeros
from numpy import dot

def rotationMatrix(x,y,z,rads):
    c = math.cos(rads)
    s = math.sin(rads)
    #cross = numpy.matrix([[0.0,-z,y],[z,0.0,-x],[-y,x,0.0]])
    #return numpy.eye(3)+c*cross-s*cross*cross;
    val = [[c,0.0,0.0],[0.0,c,0.0],[0.0,0.0,c]]
    rrt = [[x*x,x*y,x*z],[y*x,y*y,y*z],[z*x,z*y,z*z]]
    cross = [[0.0,-z,y],[z,0.0,-x],[-y,x,0.0]]
    for i in range(3):
        for j in range(3):
            val[i][j] += (1.0-c)*rrt[i][j] + s*cross[i][j]
    return val

def val3TransformToMatrix(xform):
    """Given the return values from TrajServer.getTransform, produces the 4x4
    numpy.array representing the transformation in homogeneous coordinates.

    Converts mm to meters, and converts degrees to radians.
    """
    mat = numpy.zeros((4,4))
    #translation
    mat[3,0]=xform[0]*0.001
    mat[3,1]=xform[1]*0.001
    mat[3,2]=xform[2]*0.001
    mat[3,3]=1
    rx = xform[4]*math.pi/180.0
    ry = xform[5]*math.pi/180.0
    rz = xform[6]*math.pi/180.0
    #rotations applied as rx ry rz in that order
    Rx = array(rotationMatrix(1.,0.,0.,rx))
    Ry = array(rotationMatrix(0.,1.,0.,ry))
    Rz = array(rotationMatrix(0.,0.,1.,rz))
    mat[ix_([0,1,2],[0,1,2])] = dot(Rx,dot(Ry,Rz))
    return mat


class SafeTrajClient:
    """A TrajClient API that avoids violating joint, velocity, and
    acceleration constraints.  Trajectories are required to terminate at a
    stationary configuration.

    Note: safety is only guaranteed between method calls.  If the connection to
    the server is lost during a motion command, then bad things may happen!
    Also, the user is responsible for avoiding environment collisions and
    self-collisions.
    """
    def __init__(self,host,port=1000):
        self.__t = TrajClient(host,port)
        (self.qmin,self.qmax) = self.__t.getJointLimits()
        self.qmin0 = self.qmin
        self.qmax0 = self.qmax
        self.vmax0 = self.vmax = self.__t.getVelocityLimits()
        self.amax0 = self.amax = self.__t.getAccelerationLimits()
        self.dmax0 = self.dmax = self.__t.getDecelerationLimits()
        self.segments = []
        self.maxSegments = self.__t.getMaxSegments()
        self.milestoneBatch = 100
        self.maxRate = self.__t.rate()
        self.endConfig = self.__t.getEndConfig()
        self.endTime = self.__t.getTrajEndTime()
        self.latency = 0
        self.verbose = True

    def client(self):
        """Returns a safe version of the TrajClient object used by this
        object.  Motion commands and setting joint/vel/acc limits are not
        allowed from this object."""
        return ReadOnlyTrajClient(self.__t)

    def calibrateLatency(self,iters=10):
        t0 = time.time()
        for i in xrange(iters):
            self.__t.echo("blah")
        t1 = time.time()
        self.latency = (t1-t0)/iters

    def setSpeedScale(self,rate):
        """Sets a maximum velocity and acceleration multiplier.
        Rate must be in the range (0,1]
        """
        assert rate > 0 and rate <= 1, "Speed limits must be between 0 and 1"
        self.vmax = [x*rate for x in self.vmax0]
        self.amax = [x*rate for x in self.amax0]
        self.dmax = [x*rate for x in self.dmax0]
        self.__t.setVelocityLimits(self.vmax)
        self.__t.setAccelerationLimits(self.amax)
        self.__t.setDecelerationLimits(self.dmax)

    def brakeAbs(self,t):
        self.resetTrajectoryAbs(t)
        q = self.endConfig
        dq = self.__t.getEndVelocity()
        f = ParabolicRampND.solveBrakingTrajectory(q,dq,self.dmax)
        self.__addTrajectory(f,f.endTime())

    def resetTrajectoryAbs(self,t):
        """Abruptly resets the trajectory at time t.
        Note: unsafe, unless velocity=0 at time t, or if this is immediately
        followed by addMilestone commands."""
        self.__t.resetTrajectoryAbs(t)
        self.endConfig = self.__t.getEndConfig()
        self.endTime = t

    def resetTrajectoryRel(self,t):
        self.__t.resetTrajectoryRel(t)
        self.endConfig = self.__t.getEndConfig()
        self.endTime = self.__t.getTrajEndTime()

    def makemove(self,q0,qend,rate=1.0,dt=None):
        """Produces a smooth linear move from q0 to the configuration qend.

        If dt is provided, then the move is attempted so that it ends
        exactly at dt.  If dt is too short to achieve the motion feasibly,
        then a ValueError is raised.

        If rate is provided, then the robot's
        maximum speed and acceleration is multiplied by rate.

        Return value is a NonlinearInterpolation,time pair.
        """
        if rate > 1.0 or rate <= 0.0:
            raise ValueError("Rate must be in (0,1]")

        #correction factor for numerical errors
        rate *= 0.99
        
        uvmax = 1e30
        uamax = 1e30
        udmax = 1e30
        dq = [0]*6
        for i in xrange(6):
            dq[i] = qend[i]-q0[i]
            if abs(dq[i])>1e-6:
                uvmax = min(uvmax,self.vmax[i]*rate/abs(dq[i]))
                uamax = min(uvmax,self.amax[i]*rate/abs(dq[i]))
                udmax = min(uvmax,self.dmax[i]*rate/abs(dq[i]))
        if uvmax == 1e30:
            #already at desired
            return None
        #interpolate u(t) between 0 and 1 with the given vel/acc limits
        u=None
        if dt == None:
            u = ParabolicRamp1D.solveMinTime(0,1,uamax,uvmax,udmax)
        else:
            u = ParabolicRamp1D.solveMinAccel(0,1,uamax,uvmax,udmax,dt)
        return (NonlinearInterpolation(q0,qend,u),u.endTime())

    def makesafetraj(self,q0,q,dq=None,dt=None,rate=1.0):
        """Produces a PiecewiseTrajectory that defines
        a complete move from q0 through the configuration/velocity
        milestones q,dq.  If the segment time durations dt are provided,
        then it tries to meet the milestone at the given times.

        The returned trajectory will start and end with zero velocity.
        If dq is provided and the final dq is nonzero, then a braking
        trajectory will be appended.

        Warning: Because parabolicramp doesn't yet have functionality for
        nonzero intermediate velocities, this function currently only
        supports dq=None.  Use the C++ library if velocities are desired.
        """
        for qi in q:
            assert len(qi)==len(self.qmin),"q element incorrect size"
        if dq != None:
            assert len(dq) == len(q),"q and dq are not the same size"
            for dqi in dq:
                assert len(dqi)!=len(self.qmin),"dq element not the right size"
        if dt != None:
            assert len(dt) != len(q),"q and dt are not the same size"
            for dti in dt:
                assert dti >= 0,"Negative time duration in dt"
        if len(q)==0:
            return
        
        segs=[None]*len(q)
        x = []
        if dq != None:
            x = zip(q,dq)
        else:
            x = q

        #correction factor for numerical errors
        rate *= 0.99
        
        ramax = [rate*x for x in self.amax]
        rdmax = [rate*x for x in self.dmax]
        rvmax = [rate*x for x in self.vmax]
        if dts != None:
            solver = RampSolver(ramax,rvmax,rdmax,"fixedtime")
            segs[0] = solver(q0[i],x[0],dts[0])
            for i in xrange(len(x)-1):
                segs[i+1] = solver(x[i],x[i+1],dts[i+1])
        else:
            solver = RampSolver(ramax,rvmax,rdmax,"mintime")
            segs[0] = solver(q0[i],x[0])
            for i in xrange(len(x)-1):
                segs[i+1] = solver(x[i],x[i+1])

        if dq != None and any([e != 0 for e in x[-1][1]]):
            #final dq is nonzero, add a braking trajectory
            segs += [ParabolicRampND.solveBrakingTrajectory(x[-1][0],x[-1][1],dmax)]
        
        times = [s.endTime() for s in segs]
        return PiecewiseTrajectory(segs,times,relative=True)

    def moveto(self,q,rate=1.0,dt=None):
        """Appends a smooth linear move from the trajectory queue's end
        to the configuration q (ending in zero velocity).
        See  makemove documentation for more details."""
        res = self.makemove(self.endConfig,q,rate,dt)
        if res==None: return
        (t,endtime) = res
        self.checkTrajectory(t,endtime)
        self.__addTrajectory(t,endtime)

    def movetraj(self,q,dq=None,dt=None,rate=1.0):
        """Produces a move through the trajectory defined by milestones q,
        and possibly with intermediate velocities and times.
        self.makesafetraj(self.endConfig,q,dq,dt,rate).
        See makesafetraj documentation for more details
        """
        t = self.makesafetraj(self.endConfig,q,dq,dt,rate)
        self.checkTrajectory(t,t.endTime())
        self.__addTrajectory(t,t.endTime())
        
    def addTrajectory(self,f,tmax):
        """Sends the trajectory f(t), for t in [0,tmax], to the
        controller at the maximum rate.  If f(t) is not safe, then an
        exception is raised."""

        self.checkTrajectory(f,tmax)
        self.__addTrajectory(f,tmax)

    def checkState(self,q,dq=None,ddq=None):
        for i in xrange(len(q)):
            if q[i] < self.qmin[i] or q[i] > self.qmax[i]:
                raise ValueError("State out of joint limits")
        if dq != None:
            for i in xrange(len(q)):
                if abs(dq[i]) > self.vmax[i]:
                    raise ValueError("State out of velocity limits")
        if ddq != None:
            dt = 1.0/self.maxRate
            for i in xrange(len(q)):
                if abs(dq[i]+ddq[i]*dt) > abs(dq[i]): #accelerating
                    if abs(ddq[i]) > self.amax[i]:
                        raise ValueError("State %d out of acceleration limits |%g| > %g "%(i,ddq[i],self.amax[i]))
                else:
                    if abs(ddq[i]) > self.dmax[i]:
                        raise ValueError("State %d out of deceleration limits |%g| > %g "%(i,ddq[i],self.dmax[i]))


    def checkTrajectory(self,f,tmax):
        assert tmax >= 0, "Time duration must be positive"
        dt = 1.0/self.maxRate
        qprev = array(self.endConfig)
        dqprev = zeros((len(self.endConfig)))
        if any(qprev != f(0)):
            raise ValueError("Trajectory doesn't start at current end config")
        
        t = dt
        while t < tmax:
            q = array(f(t))
            dq = (q-qprev)/dt
            ddq = (dq-dqprev)/dt
            self.checkState(q,dq,ddq)
            t += dt
            qprev = q
            dqprev = dq

        #check last point/velocity
        q = array(f(tmax))
        dq = (q-qprev)/dt
        ddq = (dq-dqprev)/dt
        self.checkState(q,dq,ddq)
        
        #check last acceleration
        ddq = dq/dt
        dq = zeros((len(self.endConfig)))
        self.checkState(q,dq,ddq)

    def __addTrajectory(self,f,tmax,dt=None):
        """Sends the trajectory f(t), for t in [0,tmax], to the
        controller at intervals dt.  If dt is not specified, it sends
        milestones at the maximum rate.

        *Important note*
        *The caller is responsible for ensuring that the trajectory is safe!*

        If the controller's buffer will not allow tmax/dt
        milestones to be sent at once, this routine will block and send them
        off in batches until complete."""
        if dt==None:
            dt = 1.0/self.maxRate
        assert dt >= 0, "Dt must be positive"
        ts = [dt]*int(math.ceil(tmax/dt))
        ts[-1] = tmax - dt*(len(ts)-1)
        qs = [[]]*len(ts)
        t = 0
        for i in xrange(len(ts)):
            t += ts[i]
            qs[i] = f(t)
        if self.verbose: print "Appending %d milestones"%(len(qs))

        #f = open("traj.csv","a")
        #for i in xrange(len(ts)):
        #    f.write("%f,%f\n"%(ts[i],qs[i][1]))
        #f.close()

        cs = self.__t.getCurSegments()
        i=0
        while i < len(qs):
            if self.verbose: print "Current segments: %d, remaining %d"%(cs,self.maxSegments-cs)
            lim = min(self.milestoneBatch,self.maxSegments-cs)
            imax = min(i+lim,len(qs)-1)
            if self.verbose: print "Sending "+str(imax-i+1)+" milestones"
            self.__t.appendMilestonesQuiet(ts[i:imax+1],qs[i:imax+1])
            i = imax+1
            if i < len(qs) and lim != self.milestoneBatch:
                sleept = 0.5*self.milestoneBatch*dt;
                time.sleep(sleept)
                cs = self.__t.getCurSegments()
            else:
                cs += lim

        #update internal representation
        self.endConfig = qs[-1]
        self.endTime = self.endTime+tmax
