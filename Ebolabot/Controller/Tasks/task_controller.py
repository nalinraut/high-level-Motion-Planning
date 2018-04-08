"""Defines the TaskController interface that a task controller should
fill out in order to be compatible with the ControllerService dispatcher.
"""

from motion import robot

class TaskController:
    """Base class for all tasks used in a ControllerService dispatcher.
    The subclass must overload taskName(), start(), and stop() at a minimum.
    Optionally, drawGL may be overriden as well if ControllerDispatcher is
    run with a -v flag.

    Subclasses should also call the message() function to feed back messages
    to the UI, the onDone() function to report that the task is done, or
    the other onX() functions to report errors.

    Attributes:
        _status: a string indicating the return status.  See status() for
                 more information about this parameter.
    """
    def __init__(self):
        self._status = ''
        self.world = None
        return
    def init(self,world):
        """Sets a shard Klamp't world"""
        self.world = world
    def taskName(self):
        """Returns the name of the task that this controller is supposed to
        run.
        """
        return None
    def start(self,args):
        """Called by dispatcher to start the controller.  Return True if
        successful."""
        return False
    def status(self):
        """Called by dispatcher to monitor the status of the controller. Return
        results can be:
        - Empty string: not running / Stop() successful 
        - "ok": task is running, and everything is ok
        - "done": task was completed successfully
        - "unavailable": the task is not currently available
        - "invalid parameters": Start() was called with invalid parameters
        - "error": generic error
        - "incomplete": the controller internally determined that had to
        stop, and the task was not achieved
        - "stopping": the controller is currently stopping (either Stop() was
        called or the task was determined to need to stop).
        - "fatal": fatal error, shut down the robot
        """
        return self._status
    def stop(self):
        """Called by dispatcher to tell the controller to stop."""
        self._status = ''
        return None
    def close(self):
        """Called by dispatcher to tell the controller the program is ending,
        and to do any final cleanup."""
        pass
    def drawGL(self):
        """Called by visual dispatchers for visual debugging. Optional.
        """
    
    def message(self,message,type='error'):
        """Subclass can call this to log a message.  Future implementations
        will feed these messages back to the UI."""
        print self.taskName()+": "+message
    def onError(self,message,errorType='error'):
        """Subclass can call this to log an error."""
        self.message(message,'error')
        self._status = errorType
    def onInvalidParameters(self,message="Invalid parameters"):
        """Subclass can call this to log invalid parameters."""
        self.onError(message,'invalid parameters')
    def onDone(self):
        """Subclass should call this once the task is done."""
        self._status = 'done'
    def onFatal(self):
        """Subclass should call this to indicate the robot should be
        shut down."""
        self._status = 'fatal'


def make():
    """In your module, you will implement a make() function that returns
    an instance of your controller."""
    return TaskController()
        
