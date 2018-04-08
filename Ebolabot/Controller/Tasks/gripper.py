from task_controller import TaskController
from motion import robot
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import Command
class GripperController(TaskController):
    """Task: Gripper

    Either closes to a specified position, or to a specified force.
    
    Arguments (either command or position must be present):
    - limb: can be 'left', 'right, 'both', 0, 1, or 2
    - command*: can be 'open', 'close' (in future may support other names,
      like 'pinch-open', 'pinch-close')
    - position*: array of size 4 (single hand) or 8 (dual hand)
    - speed*: a speed multiplier, default 1.  Can be speed-per finger as well
    - force*: a force multiplier, default 1
    """
    def __init__(self):
        TaskController.__init__(self)
        self.grippers = []
    def taskName(self):
        return 'Gripper'
    def start(self,args):
        print "Calling Gripper start",args
        if 'limb' not in args:
            self.onInvalidParameters("invalid argument list")
            return False
        if 'position' not in args and 'command' not in args:
            self.onInvalidParameters("invalid argument list, must have either position or command")
            return False
        limb = args['limb']
        if limb == 0: limb = 'left'
        elif limb == 1: limb = 'right'
        elif limb == 2: limb = 'both'
        if limb == 'left': self.grippers = [robot.left_gripper]
        elif limb=='right': self.grippers = [robot.right_gripper]
        else: self.grippers = [robot.left_gripper,robot.right_gripper]
        if 'position' in args:
            values = args['position']
            speed = args.get('speed',1.0)
            force = args.get('force',1.0)
            n = robot.left_gripper.numDofs()
  	    if limb == 'both':
                n *= 2
            if not hasattr(values,'__iter__'):
                values = [values]*n
            if not hasattr(speed,'__iter__'):
                speed = [speed]*n
            if not hasattr(force,'__iter__'):
                force = [force]*n
            if len(values) != n:
                self.onInvalidParameters("Invalid size of position argument")
                return False
            if len(speed) != n:
                self.onInvalidParameters("Invalid size of speed argument")
                return False
            if len(force) != n:
                self.onInvalidParameters("Invalid size of force argument")
                return False
            if limb == 'left':
                robot.left_gripper.command(values,speed,force)
            elif limb == 'right':
                robot.right_gripper.command(values,speed,force)
            else:
                n /= 2
                robot.left_gripper.command(values[:n],speed[:n],force[:n])
                robot.right_gripper.command(values[n:],speed[n:],force[n:])
	else:
            command = args['command']
            if command=='open':
                for g in self.grippers:
                    g.open()
            elif command=='close':
                for g in self.grippers:
                    g.close()
            else:
                self.onInvalidParameters("Unsupported command "+command)
                return False
        self._status = 'ok'
        return True
    def status(self):
        if self._status == 'ok':
            #check if the gripper is moving
            if not any(g.moving() for g in self.grippers):
                self.onDone()
        return self._status
    def stop(self):
        if self._status == 'ok':
            robot.stopMotion()
        self._status = ''
        return None


def make():
    return GripperController()
