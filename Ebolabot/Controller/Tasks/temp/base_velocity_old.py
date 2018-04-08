from task_controller import TaskController
from motion import robot
import time

class BaseVelocityController(TaskController):
    """Task: BaseVelocity
    Moves the mobile base along a given velocity dx,dy,dtheta
    
    Arguments (either command or position must be present):
    - velocity: a (dx,dy,dtheta) tuple (units m/s,m/s,radians/s)
    - maxTime*: a timeout (default 1)
    """
    def __init__(self):
        TaskController.__init__(self)
    def taskName(self):
        return 'BaseVelocity'
    def start(self,args):
        print "Calling BaseVelocity start",args
        if 'velocity' not in args:
            self.onInvalidParameters("invalid argument list, must have velocity specified")
            return False
        if len(args['velocity']) != 3:
            self.onInvalidParameters("invalid argument list, velocity command is not length 3")
            return False
        velocity = args['velocity']
        maxTime = args.get('maxTime',1.0)
        if maxTime < 0:
            self.onInvalidParameters("maxTime argument is negative")
            return False
        #deadband
        for i,v in enumerate(velocity):
            if abs(v) < 1e-3: velocity[i] = 0
        robot.base.moveVelocity(velocity[0],velocity[1],velocity[2])
        self.endTime = time.time() + maxTime
        if max(abs(v) for v in velocity) == 0:
            self._status = 'done'
        else:
            self._status = 'ok'
        return True
    def status(self):
        if self._status == 'ok':
            #check if the end time has passed or the velocity is zero
            if not robot.base.moving():
                print "Ending base movement task due to not moving"
                #robot.base.moveVelocity(0,0,0)
                self.onDone()
            if time.time() > self.endTime:
                print "Ending base movement due to timeout"
                robot.base.moveVelocity(0,0,0)
                self.onDone()
        return self._status
    def stop(self):
        if self._status == 'ok':
            robot.stopMotion()
        self._status = ''
        return None


def make():
    return BaseVelocityController()