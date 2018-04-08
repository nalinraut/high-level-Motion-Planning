from task_controller import TaskController
from motion import robot

class JointPoseController(TaskController):
    """Task: JointPose
    Arguments:
    - part*: 'left', 'right', or 'base' naming the thing to move.  They may
      also be given as indices 0, 1, or 2.
    - parts*: a list of named parts.  Overrides the part argument/
    - position*: a desired position. For 'base' part, must base
      size 3.  For 'left' or 'right', must be size 7.
    - positions*: array of arrays giving the desired positions.  Overrides
      the position argument.
    - speed*: a speed multiplier, default 1.
    - safe*: either 1 (true) or 0 (false).  If true, performs planning
      to avoid obstacles.  Otherwise, does no planning.  Default false.
    """
    def __init__(self):
        TaskController.__init__(self)
        self.doneTests = []
    def taskName(self):
        return 'JointPose'
    def start(self,args):
        print "Calling JointPose start",args
        if 'part' not in args and 'parts' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'position' not in args and 'positions' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'part' in args:
            parts = [args['part']]
        else:
            parts = args['parts']
        if 'position' in args:
            angles = [args['position']]
        else:
            angles = args['positions']

        if len(parts) != len(angles):
            self.onInvalidParameters("Must have same number of parts as positions")
        speed = args.get('speed',1.0)
	if 'safe' in args:
            print "Safety flag:",args['safe']
        safe = args.get('safe',False)
        if safe:
            robot.enableCollisionChecking(True)
            print "Collision checking enabled on JointPose task"
        else:
            robot.enableCollisionChecking(False)
            print "Collision checking disabled on JointPose task"
        self.doneTests = []
        for part,angle in zip(parts,angles):
            if part == 'left' or part == 0:
                if len(angle)!=7:
                    self.onInvalidParameters("invalid 'position' argument for left arm, not size 7")
                    return False
                robot.left_mq.setRamp(angle,speed)
                self.doneTests.append(lambda : not robot.left_mq.moving())
            elif part == 'right' or part == 1:
                if len(angle)!=7:
                    self.onInvalidParameters("invalid 'position' argument for right arm, not size 7")
                    return False
                robot.right_mq.setRamp(angle)
                self.doneTests.append(lambda : not robot.right_mq.moving())
            elif part == 'base' or part == 2:
                if len(angle)!=3:
                    self.onInvalidParameters("invalid 'position' argument for base, not size 3")
                    return False
                robot.base.moveOdometryPosition(angle[0],angle[1],angle[2])
                self.doneTests.append(lambda : not robot.base.moving())
            else:
                self.onInvalidParameters("invalid 'part' setting: %s"%(str(part),))
                return False
        if safe:
            robot.enableCollisionChecking(False)
        self._status = 'ok'
        return True
    def status(self):
        if self._status == 'ok':
            #check if the motion queue is moving
            if all(test() for test in self.doneTests):
                self.onDone()
                self.doneTests = []
        return self._status
    def stop(self):
        if self._status == 'ok':
            robot.stopMotion()
        self._status = ''
        return None


def make():
    return JointPoseController()
