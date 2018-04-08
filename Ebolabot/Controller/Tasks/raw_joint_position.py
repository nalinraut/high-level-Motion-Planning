from task_controller import TaskController
from motion import robot

class RawJointPositionController(TaskController):
    """Task: RawJointPosition
    Arguments:
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - value: array of size 7 (single arm) or 14 (dual arm)
    """
    def __init__(self):
        TaskController.__init__(self)
    def taskName(self):
        return 'RawJointPosition'
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
                print "RawJointPosition: invalid 'value' argument, not size 7"
                return False
            robot.left_limb.positionCommand(angles)
        elif limb == 'right' or limb == 1:
            if len(angles)!=7:
                print "RawJointPosition: invalid 'value' argument, not size 7"
                return False
            robot.right_limb.positionCommand(angles)
        elif limb == 'both' or limb == 2:
            if len(angles)!=14:
                print "RawJointPosition: invalid 'value' argument, not size 14"
                return False
            robot.left_limb.positionCommand(angles[:7])
            robot.right_limb.positionCommand(angles[7:])
        else:
            print "RawJointPosition: invalid 'limb' setting:",limb
            self._status = 'invalid parameters'
            return False
        self._status = 'done'
        return True
    def status(self):
        return self._status
    def stop(self):
        self._status = ''
        return None


def make():
    return RawJointPositionController()
