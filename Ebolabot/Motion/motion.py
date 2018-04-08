"""Python interface to almost all of the Ebolabot motion functionality
(excluding sensors)

To use: see block at bottom of file after "if __name__ == '__main__':".
First call setup(), robot.startup(), and then use the Robot methods
as described below.  Before quitting, call robot.shutdown() to cleanly
turn off the robot.

Can switch between client, simulation, and real robot using the 'mode' keyword
argument to setup().

Some functionality requires an appropriate Klamp't model to be provided
to setup().

Notes: arms may be in one of three modes: direct control, motion
queue, or planning (+ motion queue) mode.
 - Direct control: sends commands directly to Baxter (may be position,
   velocity, effort, or raw position commands as described in the Baxter
   SDK)
 - Motion queue: maintains an internal motion queue that smoothly
   interpolates between configurations.  To send commands to the motion
   queue, use the MotionQueue classes robot.left_ee/right_ee. To stop
   a motion queue, send a direct control command.
 - Planning (+ motion queue): uses a motion planner to avoid obstacles.
   switches the robot to motion queue mode.

Caveats:
 - Accepts Rethink electric gripper and Reflex hands (in physical robot,
   not working in simulation mode at the moment)
 - Planner mode is not thoroughly tested.
 - Safe send/append milestone is not available yet.
"""

from ctypes import *
import json
import platform
import signal

LEFT=0
RIGHT=1
BOTH=2

numLimbs = 2
numLimbDofs = 7

numDofs = {LEFT:7,RIGHT:7,BOTH:14}

#global variables
motion_lib = None
motion_mode = None
robot = None

class Head:
    def nodding(self):
        """Returns true if the head is nodding"""
        return motion_lib.isHeadNodding()
    def pan(self):
        """Returns the head pan angle"""
        retval = c_double()
        if not motion_lib.getHeadPan(byref(retval)): return False
        return retval.value
    def setPan(self,pan,speed):
        """Moves the head to a given pan angle at the given speed"""
        return motion_lib.sendHeadPan(c_double(pan),c_double(speed))

class MobileBase:
    def enabled(self):
        """Returns true if the mobile base is enabled"""
        return motion_lib.isMobileBaseEnabled()
    def moving(self):
        """Returns true if the mobile base is moving"""
        return motion_lib.isMobileBaseMoving()
    def moveTime(self):
        """Returns the time until the mobile base stops, or -1 if it's doing a velocity command"""
        return motion_lib.getMobileBaseMoveTime().value
    def target(self):
        """Returns the target for the mobile base in local coordinates"""
        x,y,theta = c_double(),c_double(),c_double()
        if not motion_lib.getMobileBaseTarget(byref(x),byref(y),byref(theta)): return False
        return (x.value,y.value,theta.value)
    def odometryTarget(self):
        """Returns the target for the mobile base in absolute (odometry) coordinates"""
        x,y,theta = c_double(),c_double(),c_double()
        if not motion_lib.getMobileBaseOdometryTarget(byref(x),byref(y),byref(theta)): return False
        return (x.value,y.value,theta.value)
    def velocity(self):
        """Returns mobile base velocity in local coordinates"""
        x,y,theta = c_double(),c_double(),c_double()
        if not motion_lib.getMobileBaseVelocity(byref(x),byref(y),byref(theta)): return False
        return (x.value,y.value,theta.value)
    def odometryPosition(self):
        """Returns odometry coordinates of the mobile base"""
        x,y,theta = c_double(),c_double(),c_double()
        if not motion_lib.getMobileBaseOdometry(byref(x),byref(y),byref(theta)): return False
        return (x.value,y.value,theta.value)
    def commandedVelocity(self):
        """Returns commanded mobile base velocity in local coordinates"""
        x,y,theta = c_double(),c_double(),c_double()
        if not motion_lib.getMobileBaseCommandedVelocity(byref(x),byref(y),byref(theta)): return False
        return (x.value,y.value,theta.value)
    def moveOdometryPosition(self,x,y,theta):
        """Sends a move target for the mobile base, in absolute (odometry) coordinates"""
        return motion_lib.sendMobileBaseOdometryPosition(c_double(x),c_double(y),c_double(theta))
    def movePosition(self,xrel,yrel,thetarel):
        """Sends a move target for the mobile base, in local coordinates"""
        return motion_lib.sendMobileBasePosition(c_double(xrel),c_double(yrel),c_double(thetarel))
    def moveVelocity(self,dx,dy,dtheta):
        """Sends a move velocity for the mobile base, in local coordinates"""
        return motion_lib.sendMobileBaseVelocity(c_double(dx),c_double(dy),c_double(dtheta))
    def stop(self):
        """Sends a stop command"""
        return self.moveVelocity(0,0,0)

class Limb:
    def __init__(self,limb):
        self.limb = limb
        self.temp = (c_double*numDofs[limb])()
        self.temp_p = cast(self.temp,POINTER(c_double))
        self.itemp = None
    def mode(self):
        """Returns 'position', 'velocity', 'effort', 'raw_position',
        'motion_queue', 'end_effector_drive',
        or 'planner' depending on the current operational mode of the limb."""
        if motion_lib.isPlannerEnabled(self.limb): return 'planner'
        elif motion_lib.isEndEffectorDriveEnabled(self.limb): return 'end_effector_drive'
        elif motion_lib.isMotionQueueEnabled(self.limb): return 'motion_queue'
        elif motion_lib.isLimbPositionMode(self.limb): return 'position'
        elif motion_lib.isLimbVelocityMode(self.limb): return 'velocity'
        elif motion_lib.isLimbEffortMode(self.limb): return 'effort'
        elif motion_lib.isLimbRawPositionMode(self.limb): return 'raw_position'
        raise RuntimeError("Couldn't get limb mode?")
    def enableSelfCollisionAvoidance(self,enabled):
        """If enabled=True, the limb will use Baxter position commands for the
        motion queue and drive modes.  Position commands are filtered using 
        Baxter's internal self-collision avoidance (on by default).
        Otherwise, it will use raw position commands, which do not.  Disable
        only if you are sure that your trajectories are collision free!
        """
        return motion_lib.enableLimbSelfCollisionAvoidance(self.limb,enabled)
    def sensedPosition(self):
        """Returns the sensed limb position"""
        if not motion_lib.getLimbPosition(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def sensedVelocity(self):
        """Returns the sensed limb velocity"""
        if not motion_lib.getLimbVelocity(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def sensedEffort(self):
        """Returns the sensed limb effort"""
        if not motion_lib.getLimbEffort(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def commandedPosition(self):
        """Returns the commanded limb position"""
        if not motion_lib.getLimbCommandedPosition(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def commandedVelocity(self):
        """Returns the commanded limb velocity"""
        if not motion_lib.getLimbCommandedVelocity(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def positionCommand(self,angles):
        """Sends a limb position command"""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendLimbPosition(self.limb,self.temp_p)
    def rawPositionCommand(self,angles):
        """Sends a limb raw position command"""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendLimbRawPosition(self.limb,self.temp_p)
    def velocityCommand(self,dangles):
        """Sends a limb velocity command"""
        assert len(dangles)==len(self.temp)
        for i,v in enumerate(dangles): self.temp[i] = v
        return motion_lib.sendLimbVelocity(self.limb,self.temp_p)
    def effortCommand(self,efforts):
        """Sends a limb velocity command"""
        assert len(efforts)==len(self.temp)
        for i,v in enumerate(efforts): self.temp[i] = v
        return motion_lib.sendLimbEffort(self.limb,self.temp_p)
    def configFromKlampt(self,qklampt):
        """Given a Klamp't configuration qklampt, returns the limb
        configuration corresponding to the limb dofs in the klamp't model"""
        if self.itemp == None:
            self.itemp = (c_int*numDofs[self.limb])()
            if not motion_lib.getKlamptLimbIndices(self.limb,cast(self.itemp,POINTER(c_int))): return False;
        return [qklampt[i] for i in self.itemp]
    def configToKlampt(self,qlimb,qklampt):
        """Given a limb configuration qlimb and a Klamp't configuration
        qklampt, returns the klamp't configuration with the limb dofs
        in the klamp't model set to qlimb """
        if self.itemp == None:
            self.itemp = (c_int*numDofs[self.limb])()
            if not motion_lib.getKlamptLimbIndices(self.limb,cast(self.itemp,POINTER(c_int))): return False;
        res = qklampt[:]
        for i,v in zip(self.itemp,qlimb):
            res[i] = v
        return res

class EndEffector:
    def __init__(self,limb):
        self.limb = limb
        self.tempr = (c_double*9)()
        self.tempp = (c_double*3)()   
        self.tempr_p = cast(self.tempr,POINTER(c_double))
        self.tempp_p = cast(self.tempp,POINTER(c_double))
    def setOffset(self,localPosition):
        """Sets the offset of the "end effector" in local coordiantes, as thought of by the
        remainder of the EndEffector methods."""
        for i,v in enumerate(localPosition): self.tempp[i] = v   
        return motion_lib.setEndEffectorOffset(self.limb,self.tempp_p)
    def sensedTransform(self):
        """Returns the end effector transformation.   The position of the end effector
        is that of the gripper base. The mobile base odometry is ignored.
        Return value is an (R,t) pair describing the base-centric rotation R and translation
        t (same format as klampt.se3 objects)."""
        if not motion_lib.getEndEffectorSensedTransform(self.limb,self.tempr_p,self.tempp_p): return False
        return [c for c in self.tempr],[c for c in self.tempp]
    def sensedVelocity(self):
        """Returns the end effector velocity.   The position of the end effector
        is that of the gripper base. The mobile base velocity / odometry is ignored. 
        Return value is (angVel,vel) which is a pair of 3-element arrays denoting the gripper
        angular velocity / velocity in world space."""
        if not motion_lib.getEndEffectorSensedVelocity(self.limb,self.tempr_p,self.tempp_p): return False
        return [c for c in self.tempr[:3]],[c for c in self.tempp]
    def commandedTransform(self):
        """Returns the end effector transformation.   The position of the end effector
        is that of the gripper base. The mobile base odometry is ignored. 
        Return value is an (R,t) pair describing the base-centric rotation R and translation
        t (same format as klampt.se3 objects)."""
        if not motion_lib.getEndEffectorCommandedTransform(self.limb,self.tempr_p,self.tempp_p): return False
        return [c for c in self.tempr],[c for c in self.tempp]
    def commandedVelocity(self):
        """Returns the end effector velocity.   The position of the end effector
        is that of the gripper base. The mobile base velocity / odometry is ignored. 
        Return value is (angVel,vel) which is a pair of 3-element arrays denoting the gripper
        angular velocity / velocity in world space."""
        if not motion_lib.getEndEffectorCommandedVelocity(self.limb,self.tempr_p,self.tempp_p): return False
        return [c for c in self.tempr[:3]],[c for c in self.tempp]
    def moveTo(self,R,t,maxRotationError=0.0,maxPositionError=0.0,maxJointDeviation=0.0):
        """Sends an end effector move-to command.   The position of the end effector
        is that of the gripper base. The mobile base velocity / odometry is ignored. 
        Note: The interpolation is NOT linear in cartesian space.
        Return value is an (R,t) pair describing the base-centric rotation R and translation
        t (same format as klampt.se3 objects)."""
        assert len(R)==9
        assert len(t)==3
        for i,v in enumerate(R): self.tempr[i] = v
        for i,v in enumerate(t): self.tempp[i] = v
        return motion_lib.sendEndEffectorMoveTo(self.limb,self.tempr_p,self.tempp_p,c_double(maxRotationError),c_double(maxPositionError),c_double(maxJointDeviation))
    def velocityCommand(self,angVel,vel):
        """Sends an end effector velocity command.   The position of the end effector
        is that of the gripper base. The mobile base velocity / odometry is ignored. 
        Arguments (angVel,vel) are a pair of 3-element arrays denoting the gripper
        angular velocity / velocity in world space.  If angVel == None, then angular
        velocity is left unconstrained."""
        assert len(vel)==3
        for i,v in enumerate(vel): self.tempp[i] = v
        if angVel != None: 
            assert len(angVel)==3
            for i,v in enumerate(angVel): self.tempr[i] = v
            return motion_lib.sendEndEffectorVelocity(self.limb,self.tempr_p,self.tempp_p)
        else:
            return motion_lib.sendEndEffectorPositionVelocity(self.limb,self.tempp_p)
    def driveCommand(self,angVel,vel):
        """Sends an end effector drive command.   The position of the end effector
        is that of the gripper base. The mobile base velocity / odometry is ignored. 
        Arguments (angVel,vel) are a pair of 3-element arrays denoting the gripper
        angular velocity / velocity in world space.  If angVel == None, then angular
        velocity is left unconstrained.

        This differs from velocityCommand() in that the end effector is
        consistently driven to follow a screw motion in Cartesian space starting from
        its current commanded transform.  velocityCommand only sends an
        instantaneous velocity which is translated into joint velocities, which drifts
        away from the integrated Cartesian motion over time.
        """
        assert len(vel)==3
        for i,v in enumerate(vel): self.tempp[i] = v
        if angVel != None:
            assert len(angVel)==3
            for i,v in enumerate(angVel): self.tempr[i] = v
            return motion_lib.sendEndEffectorDrive(self.limb,self.tempr_p,self.tempp_p)
        else:
            return motion_lib.sendEndEffectorPositionDrive(self.limb,self.tempp_p)


class Gripper:
    def __init__(self,limb):
        self.limb = limb
        self.temp = None
        self.ktemp = None
        self.nd = None
    def _init_buffers(self):
        if self.temp == None:
            self.nd = self.numDofs()
            self.temp = (c_double*self.nd)()
            self.tempv = (c_double*self.nd)()
            self.tempf = (c_double*self.nd)()
            self.temp_p = cast(self.temp,POINTER(c_double))
            self.tempv_p = cast(self.tempv,POINTER(c_double))
            self.tempf_p = cast(self.tempf,POINTER(c_double))
        return
    def enabled(self):
        """Returns true if the gripper is enabled"""
        return motion_lib.isGripperEnabled(self.limb)
    def numDofs(self):
        """Returns the number of DOFs for controlling the gripper"""
        if self.nd == None:
            return motion_lib.numGripperDofs(self.limb)
        return self.nd
    def type(self):
        """Returns the type string of the gripper"""
        p = create_string_buffer(100)
        res = motion_lib.getGripperType(self.limb,p,100)
        if not res: raise RuntimeError()
        return repr(p.value)
    def moving(self):
        """Returns true the gripper is moving to the desired setpoint"""
        return motion_lib.isGripperMoving(self.limb)
    def moveTime(self):
        """Returns the estimated time until the gripper stops"""
        return motion_lib.getGripperMoveTime(self.limb)
    def position(self):
        """Returns the gripper position"""
        self._init_buffers()
        if not motion_lib.getGripperPosition(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def target(self):
        """Returns the gripper target"""
        self._init_buffers()
        if not motion_lib.getGripperTarget(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def effort(self):
        """Returns the gripper effort"""
        self._init_buffers()
        if not motion_lib.getGripperEffort(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def open(self):
        """Tells the gripper to open"""
        return motion_lib.sendOpenGripper(self.limb)
    def close(self):
        """Tells the gripper to close"""
        return motion_lib.sendCloseGripper(self.limb)
    def command(self,pos,vel,force):
        """Tells the gripper to open/close to the given position / speed, stopping at the given effort"""
        self._init_buffers()
        assert len(pos)==len(self.temp)
        assert len(vel)==len(self.temp)
        for i,v in enumerate(pos): self.temp[i] = v
        for i,v in enumerate(vel): self.tempv[i] = v
        for i,v in enumerate(force): self.tempf[i] = v
        return motion_lib.sendSetGripper(self.limb,self.temp_p,self.tempv_p,self.tempf_p)
    def configFromKlampt(self,qklampt):
        """Given a Klamp't configuration qklampt, returns the gripper
        command corresponding to the gripper dofs in the klamp't model"""
        self._init_buffers()
        if self.ktemp == None:
            n = motion_lib.getKlamptNumDofs()
            self.ktemp = (c_double*n)()
        for i,v in enumerate(qklampt): self.ktemp[i] = v
        if not motion_lib.getKlamptGripper(cast(self.ktemp,POINTER(c_double)),self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def configToKlampt(self,qgripper,qklampt):
        """Given a gripper command qgripper and a Klamp't configuration
        qklampt, returns the klamp't configuration with the gripper dofs
        in the klamp't model set to qgripper"""
        self._init_buffers()
        if self.ktemp == None:
            n = motion_lib.getKlamptNumDofs()
            self.ktemp = (c_double*n)()
        for i,v in enumerate(qklampt): self.ktemp[i] = v
        for i,v in enumerate(qgripper): self.temp[i] = v
        if not motion_lib.setKlamptGripper(self.temp_p,self.limb,cast(self.ktemp,POINTER(c_double))): return False
        return [c for c in self.ktemp]

class LimbMotionQueue:
    def __init__(self,limb):
        self.limb = limb
        self.temp = (c_double*numDofs[limb])()
        self.tempv = (c_double*numDofs[limb])()
        self.temp_p = cast(self.temp,POINTER(c_double))
        self.tempv_p = cast(self.tempv,POINTER(c_double))
    def enabled(self):
        """Returns true if the limb is being controlled by the motion queue"""
        return motion_lib.isMotionQueueEnabled(self.limb) 
    def moving(self):
        """Returns true if the limb's motion queue currently stores an active motion"""
        return motion_lib.isMotionQueueMoving(self.limb)
    def moveTime(self):
        """Returns the estimated amount of time until the motion queue stops"""
        return motion_lib.getMotionQueueMoveTime(self.limb)
    def target(self):
        """Returns the end configuration of the motion queue"""
        if not motion_lib.getMotionQueueTarget(self.limb,self.temp_p): return False
        return [c for c in self.temp]
    def setLinear(self,duration,angles):
        """Interpolates linearly from the current configuration to the desired angles over the given duration"""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendMotionQueueLinear(self.limb,c_double(duration),self.temp_p)
    def setCubic(self,duration,angles,dangles):
        """Interpolates cubically from the current config/velocity to the desired config/velocity over the given duration"""
        assert len(angles)==len(self.temp)
        assert len(dangles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        for i,v in enumerate(dangles): self.tempv[i] = v
        return motion_lib.sendMotionQueueCubic(self.limb,c_double(duration),self.temp_p,self.tempv_p)
    def setRamp(self,angles,speed=1):
        """Moves the limb smoothly from the current config to the desired milestone according to
        the robot's velocity/acceleration limits.  Requires a Klamp't model to be set"""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendMotionQueueRamp(self.limb,self.temp_p,c_double(speed))
    def appendLinear(self,duration,angles):
        """Appends a linear interpolation to the desired angles over the given duration."""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendMotionQueueAppendLinear(self.limb,c_double(duration),self.temp_p)
    def appendCubic(self,duration,angles,dangles):
        """Appends a cubic interpolation  to the desired config/velocity over the given duration"""
        assert len(angles)==len(self.temp)
        assert len(dangles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        for i,v in enumerate(dangles): self.tempv[i] = v
        return motion_lib.sendMotionQueueAppendCubic(self.limb,c_double(duration),self.temp_p,self.tempv_p)
    def appendRamp(self,angles,speed=1):
        """Appends a smooth ramp to the given milestone to the motion queue according to the robot's
        velocity/acceleration limits.  Requires a Klamp't model to be set."""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendMotionQueueAppendRamp(self.limb,self.temp_p,c_double(speed))
    def appendLinearRamp(self,angles,speed=1):
        """Appends a smooth ramp to the given milestone to the motion queue according to the robot's
        velocity/acceleration limits.  Differs from appendRamp in that it moves along a straight
        joint-space trajectory.  Requires a Klamp't model to be set."""
        assert len(angles)==len(self.temp)
        for i,v in enumerate(angles): self.temp[i] = v
        return motion_lib.sendMotionQueueAppendLinearRamp(self.limb,self.temp_p,c_double(speed))
    def setTrajectory(self,times,milestones,dmilestones=None):
        """Sends a trajectory starting from the current configuration.  Milestones are given as a list of limb
        configurations.  If dmilestones is provided, then it is a list of limb joint velocities, and cubic
        interpolation is performed.  Otherwise, linear interpolation is performed."""
        assert len(times)==len(milestones)
        k = len(times)
        for m in milestones:
            assert len(m)==len(self.temp),"Milestone has incorrect size"
        ttemp = (c_double*k)()
        mtemp = (c_double*k*len(self.temp))()
        vtemp = None
        for i,v in enumerate(times): ttemp[i] = v
        i = 0
        for m in milestones:
            for j,v in enumerate(m):
                mtemp[i+j] = v
            i += len(m)
        if dmilestones is not None:
            assert len(times)==len(dmilestones)
            for m in dmilestones:
                assert len(m)==len(self.temp),"Milestone velocity has incorrect size"
            vtemp = (c_double*k*len(self.temp))()
            i = 0
            for m in dmilestones:
                for j,v in enumerate(m):
                    vtemp[i+j] = v
                i += len(m)
        return motion_lib.sendMotionQueueTrajectory(self.limb,k,cast(ttemp,POINTER(c_double)),cast(mtemp,POINTER(c_double)),cast(vtemp,POINTER(c_double)))
    def appendTrajectory(self,times,milestones,dmilestones=None):
        """Sends a trajectory starting from the end of the motion queue.  Milestones are given as a list of
        limb  configurations.  If dmilestones is provided, then it is a list of limb joint velocities, and cubic
        interpolation is performed.  Otherwise, linear interpolation is performed."""
        assert len(times)==len(milestones)
        k = len(times)
        for m in milestones:
            assert len(m)==len(self.temp),"Milestone has incorrect size"
        ttemp = (c_double*k)()
        mtemp = (c_double*k*len(self.temp))()
        vtemp = None
        for i,v in enumerate(times): ttemp[i] = v
        i = 0
        for m in milestones:
            for j,v in enumerate(m):
                mtemp[i+j] = v
            i += len(m)
        if dmilestones is not None:
            assert len(times)==len(dmilestones)
            for m in dmilestones:
                assert len(m)==len(self.temp),"Milestone velocity has incorrect size"
            vtemp = (c_double*k*len(self.temp))()
            i = 0
            for m in dmilestones:
                for j,v in enumerate(m):
                    vtemp[i+j] = v
                i += len(m)
        return motion_lib.sendMotionQueueAppendTrajectory(self.limb,k,cast(ttemp,POINTER(c_double)),cast(mtemp,POINTER(c_double)),cast(vtemp,POINTER(c_double)))

class LimbPlanner:
    def __init__(self,limb):
        self.limb = limb
    def enabled(self):
        """Returns true the if limb's motion queue is being controlled by the planner"""
        return motion_lib.isPlannerEnabled(self.limb)
    def setMilestone(self,angles,speed=1):
        """Invalid"""
        return motion_lib.sendSafeArmsSetMilestone(angles,c_double(speed))
    def appendMilestone(self,angles,speed=1):
        """Invalid"""
        return motion_lib.sendSafeArmsAppendMilestone(angles,c_double(speed))
    def setObjective(self,obj):
        """Sends a JSON object describing the PlannerObjective to the planner
        (see documentation of Klampt/Planning/PlanningObjective.h)."""
        objstr = json.dumps(obj)
        return motion_lib.sendPlannerObjectiveStr(objstr)
    def stop(self):
        """Stops the planner, if currently active."""
        return motion_lib.stopPlanner()

class Planner:
    def setWorldFile(self,worldFile):
        """Sets the world file used by the planner"""
        return motion_lib.setPlannerWorldFile(c_char_p(worldFile));
    def addObstacle(self,obstacleFile,margin=0):
        """Adds an obstacle to the planner, returns its ID"""
        return motion_lib.addPlannerObstacle(c_char_p(obstacleFile),c_double(margin));
    def deleteObstacle(self,id):
        """Deletes an obstacle from the planner, given its id"""
        return motion_lib.deletePlannerObstacle(id);
    def setObstacleMargin(self,id,margin):
        """Sets the obstacle avoidance margin for the given obstacle"""
        return motion_lib.setPlannerObstacleMargin(id,margin)
    def clear(self):
        """Clears the world used by the planner"""
        return motion_lib.clearPlannerWorld()

class Motion:
    def __init__(self):
        self.left_limb = Limb(LEFT)
        self.right_limb = Limb(RIGHT)
        self.left_ee = EndEffector(LEFT)
        self.right_ee = EndEffector(RIGHT)
        self.left_gripper = Gripper(LEFT)
        self.right_gripper = Gripper(RIGHT)
        self.head = Head()
        self.base = MobileBase()
        self.left_mq = LimbMotionQueue(LEFT)
        self.right_mq = LimbMotionQueue(RIGHT)
        self.arms_mq = LimbMotionQueue(BOTH)
        self.planner = Planner()
        self.left_planner = LimbPlanner(LEFT)
        self.right_planner = LimbPlanner(RIGHT )
        self.arms_planner = LimbPlanner(BOTH)
        self.temp = None
    def publishState(self,addr='tcp://localhost:4568'):
        """Tell the robot to publish state to an SSPP state server on the given address.
        Must be called before calling startup()"""
        return motion_lib.publishState(c_char_p(addr))
    def setKlamptModel(self,modelfn):
        """If you wish to use the motion queue or the planner, you must 
        call this before calling robot.startup(). Load the Klampt baxter.rob model
        from the given file.
        
        Not necessary if the 'klampt_model' argument was provided to setup().
        """
        return motion_lib.setKlamptModel(c_char_p(modelfn))
    def loadCalibration(self,fn):
        return motion_lib.loadCalibration(c_char_p(fn))
    def getKlamptModel(self):
        """Returns the type string of the gripper"""
        p = create_string_buffer(4096)
        res = motion_lib.getKlamptModel(p,4096)
        if not res: raise RuntimeError()
        return p.value
    def time(self):
        """Returns the time since robot startup, in s"""
        return motion_lib.getTime()
    def startup(self):
        """Starts up the robot"""
        res = motion_lib.sendStartup()
        if res == False: return False
        #overrides the default Ctrl+C behavior which kills the program
        def interrupter(x,y):
            self.shutdown()
            raise KeyboardInterrupt()
        signal.signal(signal.SIGINT,interrupter)
        return res
    def shutdown(self):
        """Shuts down the robot"""
        return motion_lib.sendShutdown()
    def isStarted(self):
        """Returns true if the robot is started"""
        return motion_lib.isStarted()
    def isTorsoEnabled(self):
        """Returns true if the Baxter torso is enabled"""
        return motion_lib.isTorsoEnabled()
    def moving(self):
        """Returns true if the robot is currently moving."""
        return self.left_mq.moving() or self.right_mq.moving()  or self.head.nodding()
    def stopMotion(self):
        """Stops all motion"""
        return motion_lib.stopMotion()    
    def enableCollisionChecking(self,enabled=True):
        """Turns on collision checking for many motion commands, including position commands, motion queue
        commands, and end effector commands."""
        return motion_lib.enableCollisionChecking(int(enabled))
    def getKlamptSensedPosition(self):
        """Retrieves the sensed configuration as a Klamp't configuration.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptSensedPosition(temp_p): return False;
        return [c for c in self.temp]
    def getKlamptCommandedPosition(self):
        """Retrieves the commanded configuration as a Klamp't configuration.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptCommandedPosition(temp_p): return False;
        return [c for c in self.temp]
    def getKlamptSensedVelocity(self):
        """Retrieves the sensed joint velocities as a Klamp't vector.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptSensedVelocity(temp_p): return False;
        return [c for c in self.temp]
    def getKlamptCommandedVelocity(self):
        """Retrieves the commanded joint velocities as a Klamp't vector.  The Klamp't model must be set"""
        if self.temp == None:
            n = motion_lib.getKlamptNumDofs()
            if n < 0: return False
            self.temp = (c_double*n)()
        temp_p = cast(self.temp,POINTER(c_double))
        if not motion_lib.getKlamptCommandedVelocity(temp_p): return False;
        return [c for c in self.temp]


robot = Motion()

def setup(mode=None,libpath="",klampt_model=None,server_addr=None):
    """Motion DLL setup.  Call this before calling any other
    functions in the motion module.  This will return the Motion
    object used to interface with the robot.
    
    Arguments:
    - mode: can be None, 'kinematic', 'physical', or 'client', depending on whether
      you want to simulate or operate the real robot.  If it's None, then it
      will use whatever is currently linked to by libmotion.so.  If it's client, then
      it assumes the MotionServer_x program is running, and it will send commands to
      the motion server.
    - libpath: the path to the libmotion_x.so shared library
    - klampt_model: if not None, the path to the Klampt model of the robot.
    - server_addr: if not None, the IP address of the server.  Defaults to
      'localhost'
    """
    global motion_lib,motion_mode,robot
    #check to see if setup was called before?
    if motion_lib != None:
        if motion_mode == mode:
            print "motion.setup(): Warning, already called setup before"
            return
        else:
            raise ValueError("motion.setup(): previously set up "+motion_mode+" mode, now requesting "+mode)

    motion_mode = mode

    #load the C library and set up the return types
    if mode==None:
        dllname = "motion"
    else:
        dllname = "motion_"+mode
    print "Trying to load from library path",libpath
    print "Trying to load dll named",dllname
    if platform.system() == 'Windows':
        motion_lib = windll.LoadLibrary(libpath+dllname+".dll")
    elif platform.system().startswith('CYGWIN'):
        motion_lib = cdll.LoadLibrary(libpath+"cyg"+dllname+".dll")
    else:
        motion_lib = cdll.LoadLibrary(libpath+"lib"+dllname+".so")

    if mode=='client' and server_addr != None:
        motion_lib.setServerAddr(c_char_p(server_addr))        

    #set up some return types
    motion_lib.getHeadPan.restype = c_double
    motion_lib.getMobileBaseMoveTime.restype = c_double
    motion_lib.getTime.restype = c_double
    motion_lib.getMotionQueueMoveTime.restype = c_double
    motion_lib.getGripperMoveTime.restype = c_double

    #set up the robot interface
    if klampt_model != None:
        existing_model = robot.getKlamptModel()
        if len(existing_model) > 0:
            if existing_model != klampt_model:
                raise RuntimeError("Klamp't model has already been set! "+existing_model)
        else:
            if not robot.setKlamptModel(klampt_model):
                raise RuntimeError("Error setting Klamp't model "+klampt_model)
    else:
        klampt_model = robot.getKlamptModel()

    print
    print "****** Motion Python API started *******"
    print "   Mode:",mode if mode!=None else "Default"
    print "   Klamp't model:",klampt_model
    print "******************************************"
    print 
    return robot

if __name__=="__main__":
    import os
    import sys
    import time

    print "Testing Ebolabot Motion Module..."
    #assumes Klampt is in the home directory
    klampt_model = os.path.join(os.path.expanduser("~"),"Klampt/data/robots/baxter_col.rob")
    mode = None
    if len(sys.argv)>1:
        mode = sys.argv[1]
    robot = setup(mode=mode,klampt_model=klampt_model,libpath='../')
    if not robot.isStarted():
        res = robot.startup()
        if not res:
            raise RuntimeError("Ebolabot Motion could not be started")
    else:
        print "Robot started by another process"
        
    print "Is robot started?",robot.isStarted()
    print
    print "STATUS"
    print "Base:"
    print "   enabled:",robot.base.enabled()
    print "   moving:",robot.base.moving()
    print "   odometry:",robot.base.odometryPosition()
    print "   velocity:",robot.base.velocity()
    print "Left arm:"
    print "   configuration:",robot.left_limb.sensedPosition()
    print "   velocity:",robot.left_limb.sensedVelocity()
    print "   effort:",robot.left_limb.sensedEffort()
    print "   motion queue enabled:",robot.left_mq.enabled()
    print "   motion queue moving:",robot.left_mq.moving()
    print "Right arm:"
    print "   configuration:",robot.right_limb.sensedPosition()
    print "   velocity:",robot.right_limb.sensedVelocity()
    print "   effort:",robot.right_limb.sensedEffort()
    print "   motion queue enabled:",robot.right_mq.enabled()
    print "   motion queue moving:",robot.right_mq.moving()
    print "Left gripper:"
    print "   end effector xform:",robot.left_ee.sensedTransform()
    print "   type:",robot.left_gripper.type()
    print "   enabled:",robot.left_gripper.enabled()
    print "   moving:",robot.left_gripper.moving()
    print "   position:",robot.left_gripper.position()
    print "Right gripper:"
    print "   end effector xform:",robot.right_ee.sensedTransform()
    print "   type:",robot.right_gripper.type()
    print "   enabled:",robot.right_gripper.enabled()
    print "   moving:",robot.right_gripper.moving()
    print "   position:",robot.right_gripper.position()
    print
    print "Shutting down..."
    robot.shutdown()
    print "Shutdown completed"
