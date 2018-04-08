from task_controller import TaskController
from motion import robot
from klampt.math import vectorops

class JointVelocityController(TaskController):
    """Task: JointVelocity
    Arguments:
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - velocity: array of size 7 (single arm) or 14 (dual arm)
    - maxTime*: a maximum amount of time to apply the velocity, default 1.
    - safe*: either 1 (true) or 0 (false).  If true, performs planning
      to avoid obstacles.  Otherwise, does no planning.  Default false.
    """
    def __init__(self):
        TaskController.__init__(self)
        self.mqs = []
    def taskName(self):
        return 'JointVelocity'
    def start(self,args):
        if 'limb' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'velocity' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        limb = args['limb']
        dangles = args['velocity']
        maxTime = args.get('maxTime',1.0)
        safe = args.get('safe',False)
        #deadband
        for i,v in enumerate(dangles):
            if abs(v) < 1e-4: dangles[i] = 0
        if safe:
            self.onInvalidParameters("Safe mode not supported yet")
            return False
        if limb == 'left' or limb == 0:
            if len(dangles)!=7:
                self.onInvalidParameters("invalid 'value' argument, not size 7")
                return False
            angles = vectorops.madd(robot.left_limb.commandedPosition(),dangles,maxTime)
            robot.left_mq.setLinear(maxTime,angles)
            self.mqs = [robot.left_mq]
        elif limb == 'right' or limb == 1:
            if len(dangles)!=7:
                self.onInvalidParameters("invalid 'value' argument, not size 7")
                return False
            angles = vectorops.madd(robot.right_limb.commandedPosition(),dangles,maxTime)
            robot.right_mq.setLinear(maxTime,angles)
            self.mqs = [robot.right_mq]
        elif limb == 'both' or limb == 2:
            if len(dangles)!=14:
                self.onInvalidParameters("invalid 'value' argument, not size 14")
                return False
            langles = vectorops.madd(robot.left_limb.commandedPosition(),dangles[:7],maxTime)
            rangles = vectorops.madd(robot.right_limb.commandedPosition(),dangles[7:],maxTime)
            robot.left_mq.sendLinear(maxTime,langles)
            robot.right_mq.sendLinear(maxTime,rangles)
            self.mqs = [robot.left_mq,robot.right_mq]
        else:
            self.onInvalidParameters("invalid 'limb' setting: %s"%(limb,))
            return False
        #a zero velocity command is just a stop
        if max(abs(v) for v in dangles) == 0:
            self._status = 'done'
        else:
            self._status = 'ok'
        return True
    def status(self):
        if self._status == 'ok':
            #check if the motion queue is moving
            if not any(mq.moving() for mq in self.mqs):
                self.onDone()
        return self._status
    def stop(self):
        if self._status == 'ok':
            robot.stopMotion()
        self._status = ''
        return None


def make():
    return JointVelocityController()
