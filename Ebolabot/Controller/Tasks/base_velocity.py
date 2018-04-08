from task_controller import TaskController
from motion import robot
import time, sys, threading

# sys.path.append('/home/motion/iml-internal/Ebolabot/Controller/Tasks/SafeBaseControl')
from SafeBaseControl.safe_base_control import SafeBaseControl

class BaseVelocityController(TaskController):
    """Task: BaseVelocity

    Moves the mobile base along a given velocity dx,dy,dtheta
    
    Arguments (either command or position must be present):
    - velocity: a (dx,dy,dtheta) tuple (units m/s,m/s,radians/s)
    - safe: a boolean toggle (default=False)
    - maxTime*: a timeout (default 1)
    """
    def __init__(self):
        TaskController.__init__(self)

        #--- Safe Control ---
        self.sbc = SafeBaseControl()
        self.sbc.set_previous_velocity((0,0,0))

        # enable safe base control thread
        self.control_factor_thread = threading.Thread(target=self.sbc.update_control_factor)
        self.control_factor_thread.setDaemon(True)
        self.control_factor_thread.start()

        self.message("Safe base control initialized")

        self.safe = False # toggle starting value
        self.safe_prev = 0
        #--- End Safe Control ---

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

        #--- Safe Control ---
        if args.get('safe', 0) == 1 and self.safe_prev is not args.get('safe', 0):
            self.safe = not self.safe
            if self.safe:
                # enable safe base control thread
                self.sbc.enabled = True
        self.safe_prev = args.get('safe', 0)
        if self.safe:
            try:
                if self.sbc.enabled and self.sbc.isConnected:
                    self.message("Safe control: ENABLED")
                    cf = self.sbc.cf    # Read most recent cf value from control factor thread
                    self.message("Control factor: " + str(cf))
                    self.sbc.set_previous_velocity(velocity)
                    controlled_velocity = self.sbc.get_controlled_velocity(velocity, cf) # Update velocity using most recent control factor
                    print(self.sbc.collision_point)
                    velocity = controlled_velocity
                if not self.sbc.isConnected:
                    self.onError("Lidar connection error")
                    self.sbc.enabled = False
            except:
                self.message(str(sys.exc_info()))
                self.onError("Error running safe base control")
        else:
            self.message("Safe control: DISABLED")
        #--- End Safe Control ---

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
                #robot.base.moveVelocity(0,0,0)
                print "Ending base movement task due to not moving"

                # #--- Safe Control ---
                # self.sbc.enabled = False
                # #--- End Safe Control ---

                self.onDone()
            if time.time() > self.endTime:
                robot.base.moveVelocity(0,0,0)
                print "Ending base movement due to timeout"

                #--- Safe Control ---
                self.sbc.enabled = False
                #--- End Safe Control ---

                self.onDone()
        return self._status
    def stop(self):
        if self._status == 'ok':
            robot.stopMotion()

            # #--- Safe Control ---
            # self.sbc.enabled = False
            # #--- End Safe Control ---


        self._status = ''
        return None

    def close(self):
        """Called by dispatcher to tell the controller the program is ending,
        and to do any final cleanup."""

        # #--- Safe Control ---
        # self.sbc.enabled = False
        # time.sleep(0.2)
        # #--- End Safe Control -


def make():
    return BaseVelocityController()
