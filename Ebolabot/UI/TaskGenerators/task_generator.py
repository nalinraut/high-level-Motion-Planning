class TaskGenerator:
    """Base class used by the UI's task multiplexer.  Subclasses need to
    implement the name(), start(), stop(), and get().  Optionally,
    close() and glPlugin() should be implemented.

    The _status attribute should be used to indicate the status of the
    UI device. Valid values are
    - '' (empty): indicates not started
    - 'ok': indicates started
    - 'error': some error occurred
    - 'done': the server finished its job.
    """
    def init(self,world):
        self.world = world
        self._status = ''
        pass
    def name(self):
        """Subclasses should override this returning the name of the server"""
        return "Task"
    def start(self):
        """Called on startup.  Returns true if successful"""
        return False
    def stop(self):
        """Called by multiplexer to indicate that this server will stop
        having control over the current task"""
        self._status=''
    def close(self):
        """Called by multiplexer to tell the controller the program is ending,
        and to do any final cleanup."""
        pass
    def get(self):
        """Returns the current task that should be sent to the controller.
        This should be either None or a dictionary specifying the task in
        Task API format.

        Note: the current commanded configuration is provided inside the WorldModel
        instance self.world.
        """
        return None
    def glPlugin(self):
        """Called by multiplexer to draw debugging information on the
        world display.  The return value should be either None or an
        instance of a klampt.GLPluginBase class."""
        return None

    def status(self):
        """Returns a status message."""
        return self._status
    def messages(self):
        """Optionally returns messages to send to the UI. Not done yet."""
        return None
