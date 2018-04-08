from task_controller import TaskController
#TODO: set the sys.path variable so that you can import pointer_client.py
from pointer_client import PPUClient

class PPUJointPoseController(TaskController):
    """Task: PPUJointPose
    Arguments:
    - pan: pan DOF, in radians, from -pi to pi
    - tilt: tilt DOF value, in radians, from 0 to pi/2
    - ext: linear actuator stroke value, in fraction [0,1]
    - *speed: speed of pan and tilt, values 1-30.  Default 1
    - *halt: if nonzero, stops the PPU in its current configuration
    """
    def __init__(self):
        TaskController.__init__(self)
        self.ppu = None
    def taskName(self):
        return 'PPUJointPose'
    def start(self,args):
        print "Calling PPUJointPose start",args
        if self.ppu is None:
            try:
                self.ppu = PPUClient()
            except RuntimeError:
                self.onError("PPU Server has a problem or is not running")
                return False
        #TODO error checking not complete
        if args.get('halt',0) != 0:
            self.ppu.setHalt()
            self.onDone()
            return True
        pan = args['pan']
        tilt = args['tilt']
        ext = args['ext']
        speed = args.get('speed',1)
        if ext < 0 or ext > 1:
            self.onInvalidParameters("invalid 'ext' setting: %s, must be in range [0,1]"%(str(ext),))
            return False
        if speed < 1 or speed > 30:
            self.onInvalidParameters("invalid 'speed' setting: %s, must be in range [1,30]"%(str(speed),))
            return False
        self.ppu.setSpeed(speed)
        self.ppu.setCmdConfig([pan,tilt,ext],'rad/prct')
        self._status = 'ok'
        return True
    def stop(self):
        if self._status == 'ok':
            self._status = ''
        self._status = ''
        return None


def make():
    return PPUJointPoseController()
