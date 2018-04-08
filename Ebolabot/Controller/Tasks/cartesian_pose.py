from task_controller import TaskController
from motion import robot

class CartesianPoseController(TaskController):
    """Task: CartesianPose

    Description: generates a motion of an end effector to a desired position
    and optionally rotation in cartesian space.  Can set the speed.  In the future
    it will also be able to check for collisions.

    Arguments:
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - position: array of size 3 (single arm) or 6 (dual arm) indicating the
      desired position of end effector in torso space.
    - rotation*: array of size 9 (single arm) or 18 (dual arm) indicating the
      entries of the rotation matrix in torso space.
    - toolCenterPoint*: position in the local frame of the end effector (wrist) at
      which the desired end effector transform is considered to be applied.
    - speed*: a speed at which to apply the movement.  (default 1)
    - maxJointDeviation*: a maximum amount to move, in joint space (default
      unlimited)
    - safe*: either 1 (true) or 0 (false).  If true, performs planning
      to avoid obstacles.  Otherwise, does no planning.  Default false.
    """
    def __init__(self):
        TaskController.__init__(self)
        self.mqs = []

    def taskName(self):
        return 'CartesianPose'

    def start(self,args):
        print "Calling CartesianPose start",args
        if 'limb' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'position' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        limb = args['limb']
        position = args['position']
        rotation = args.get('rotation',[])
        toolCenterPoint = args.get('toolCenterPoint',None)
        maxJointDeviation = args.get('maxJointDeviation',0)
        speed = args.get('speed',1.0)
        safe = args.get('safe',False)
        if safe:
            print 'safe'
            robot.enableCollisionChecking(True)
            robot.left_limb.enableSelfCollisionAvoidance(False)
            robot.right_limb.enableSelfCollisionAvoidance(False)
        else:
            robot.enableCollisionChecking(False)
            robot.left_limb.enableSelfCollisionAvoidance(True)
            robot.right_limb.enableSelfCollisionAvoidance(True)
        if limb == 'left' or limb == 0:
            if len(position) !=3:
                self.onInvalidParameters("invalid 'position' argument, not size 3")
                return False
            if len(rotation) > 0 and len(rotation)!=9:
                self.onInvalidParameters("invalid 'rotation' argument, not size 3")
                return False
            if len(rotation) == 0:
                rotation = robot.left_ee.commandedTransform()[0]
            if toolCenterPoint is not None:
                robot.left_ee.setOffset(toolCenterPoint)
            robot.left_ee.moveTo(rotation, position, maxJointDeviation=maxJointDeviation)
            self.mqs = [robot.left_mq]
        elif limb == 'right' or limb == 1:
            if len(position) !=3:
                self.onInvalidParameters("invalid 'position' argument, not size 3")
                return False
            if len(rotation) > 0 and len(rotation)!=9:
                self.onInvalidParameters("invalid 'rotation' argument, not size 9")
                return False
            if len(rotation) == 0:
                rotation = robot.right_ee.commandedTransform()[0]
            if toolCenterPoint is not None:
                robot.right_ee.setOffset(toolCenterPoint)
            robot.right_ee.moveTo(rotation, position, maxJointDeviation=maxJointDeviation)
            self.mqs = [robot.right_mq]
        elif limb == 'both' or limb == 2:
            if len(position) != 6:
                self.onInvalidParameters("invalid 'position' argument, not size 6")
                return False
            if len(rotation) > 0 and len(rotation)!=18:
                self.onInvalidParameters("invalid 'rotation' argument, not size 18")
                return False
            if len(rotation) == 0:
                rotation = robot.left_ee.commandedTransform()[0] + robot.right_ee.commandedTransform()[0]
            if toolCenterPoint is not None:
                robot.right_ee.setOffset(toolCenterPoint)
                robot.left_ee.setOffset(toolCenterPoint)
            resl = robot.left_ee.moveTo(rotation[0:9], position[0:3], maxJointDeviation=maxJointDeviation)
            resr = robot.right_ee.moveTo(rotation[9:18], position[3:6], maxJointDeviation=maxJointDeviation)
            print "Left result",resl
            print "Right result",resr
            self.mqs = [robot.left_mq,robot.right_mq]
        else:
            self.onInvalidParameters("invalid 'limb' setting: %s"%(limb,))
            return False
        if safe:
            robot.enableCollisionChecking(False)
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
    return CartesianPoseController()
