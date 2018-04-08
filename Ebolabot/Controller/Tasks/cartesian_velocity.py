from task_controller import TaskController
from motion import robot

class CartesianVelocityController(TaskController):
    """Task: CartesianVelocity
    Arguments:
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - linear: velocity in torso space, array of size 3 (single arm) or
      6 (dual arm)
    - angular: angular velocity in torso space, array of size 3 (single arm)
      or 6 (dual arm)
    - toolCenterPoint*: position in the local frame of the end effector (wrist) at 
      which the desired velocity is considered to be applied.
    - maxTime*: a maximum amount of time to apply the velocity, default 1.
    - safe*: either 1 (true) or 0 (false).  If true, performs planning
      to avoid obstacles.  Otherwise, does no planning.  Default false.
    """
    def __init__(self):
        TaskController.__init__(self)

    def taskName(self):
        return 'CartesianVelocity'

    def start(self,args):
        if 'limb' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'linear' not in args and 'angular' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        limb = args['limb']
        dlinear = args['linear']
        dangular = args['angular']
        toolCenterPoint = args.get('toolCenterPoint',None)
        maxTime = args.get('maxTime',1.0)
        safe = args.get('safe',False)
        #deadband
        for i,v in enumerate(dlinear):
            if abs(v) < 1e-4: dlinear[i] = 0
        for i,v in enumerate(dangular):
            if abs(v) < 1e-4: dangular[i] = 0
        if safe:
            robot.enableCollisionChecking(True)
        else:
            robot.enableCollisionChecking(False)
        if limb == 'left' or limb == 0:
            if len(dlinear) > 0 and len(dlinear) !=3:
                self.onInvalidParameters("invalid 'linear' argument for linear velocity, not size 3")
                return False
            if len(dangular) > 0 and len(dangular)!=3:
                self.onInvalidParameters("invalid 'angular' argument for angular velocity, not size 3")
                return False
            if len(dlinear) == 3 and len(dangular) == 0:
                dangular = [0.0, 0.0, 0.0]
            if len(dangular) == 3 and len(dlinear) == 0:
                dlinear = [0.0, 0.0, 0.0]
            if toolCenterPoint is not None:
                robot.left_ee.setOffset(toolCenterPoint)
            robot.left_ee.driveCommand(dangular, dlinear)
        elif limb == 'right' or limb == 1:
            if len(dlinear) > 0 and len(dlinear) !=3:
                self.onInvalidParameters("invalid 'linear' argument for linear velocity, not size 3")
                return False
            if len(dangular) > 0 and len(dangular)!=3:
                self.onInvalidParameters("invalid 'angular' argument for angular velocity, not size 3")
                return False
            if len(dlinear) == 3 and len(dangular) == 0:
                dangular = [0.0, 0.0, 0.0]
            if len(dangular) == 3 and len(dlinear) == 0:
                dlinear = [0.0, 0.0, 0.0]
            if toolCenterPoint is not None:
                robot.right_ee.setOffset(toolCenterPoint)
            robot.right_ee.driveCommand(dangular, dlinear)
        elif limb == 'both' or limb == 2:
            if len(dlinear) > 0 and len(dlinear) !=6:
                self.onInvalidParameters("invalid 'linear' argument for linear velocity, not size 6")
                return False
            if len(dangular) > 0 and len(dangular)!=6:
                self.onInvalidParameters("invalid 'angular' argument for angular velocity, not size 6")
                return False
            if len(dlinear) == 6 and len(dangular) == 0:
                dangular = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            if len(dangular) == 6 and len(dlinear) == 0:
                dlinear = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            if toolCenterPoint is not None:
                robot.right_ee.setOffset(toolCenterPoint)
                robot.left_ee.setOffset(toolCenterPoint)
            robot.right_ee.driveCommand(dangular[0:3], dlinear[0:3])
            robot.left_ee.driveCommand(dangular[3:6], dlinear[3:6])
        else:
            self.onInvalidParameters("invalid 'limb' setting: %s"%(limb,))
            return False
        if max(abs(v) for v in dangular+dlinear) == 0:
            self._status = 'done'
        else:
            self._status = 'ok'
        return True
    def status(self):
        if self._status == 'ok':
            pass
        return self._status
    def stop(self):
        self._status = ''
        robot.enableCollisionChecking(False)
        return None


def make():
    return CartesianVelocityController()
