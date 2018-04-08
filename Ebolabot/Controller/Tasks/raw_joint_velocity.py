from task_controller import TaskController
from motion import robot

class RawJointVelocityController(TaskController):
    """Task: RawJointVelocity
    Arguments:
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - value: array of size 7 (single arm) or 14 (dual arm)
    """
    def __init__(self):
        TaskController.__init__(self)
    def taskName(self):
        return 'RawJointVelocity'
    def start(self,args):
        if 'limb' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'value' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        limb = args['limb']
        angles = args['value']
        if limb == 'left' or limb == 0:
            if len(angles)!=7:
                self.onInvalidParameters("invalid 'value' argument, not size 7")
                return False
            robot.left_limb.velocityCommand(angles)
        elif limb == 'right' or limb == 1:
            if len(angles)!=7:
                self.onInvalidParameters("invalid 'value' argument, not size 7")
                return False
            robot.right_limb.velocityCommand(angles)
        elif limb == 'both' or limb == 2:
            if len(angles)!=14:
                self.onInvalidParameters("invalid 'value' argument, not size 14")
                return False
            robot.left_limb.velocityCommand(angles[:7])
            robot.right_limb.velocityCommand(angles[7:])
        else:
            self.onInvalidParameters("invalid 'limb' setting %s:"%(limb,))
            return False
        self.onDone()
        return True

def make():
    return RawJointVelocityController()
