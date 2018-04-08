VAL3 Trajectory Server
version 1.0

Kris Hauser
10/3/2010

**** Background ****

The VAL3 Trajectory Server program allows a remote computer to command
arbitrary robot trajectories through an Ethernet connection.  It gives the
programmer greater motion flexibility than using the CS8 controllers' native
VAL3 language, for example, in responding to input from visual sensors, and
also enables robot programming in more familiar programming languages and
environments.

This document describes the installation of the program and the communication
protocol.  Typically, client programs will not use the protocol directly; we
have implemented C++ and Python APIs that are more convenient to use.
Nevertheless this document will be important to understand some of the
semantics of the system.


**** Command and Motion Queue Semantics ****

There are three general categories of commands: miscellaneous, robot state
reading, and managing the motion queue.  Miscellaneous and state-reading
commands are fairly self-explanatory (see the Command List section for more
details).

Motion queue commands are the primary mechanism through which the robot's
motion is commanded.  The main concept is that of a *motion queue* which is 
stored on the controller.  The motion queue is a piecewise linear reference
trajectory MQ(t) that the robot follows at the controller's maximum rate
(250Hz).  MQ(t) interpolates linearly between *milestones* which are
(time,configuration) pairs (t[1],q[1]),...,(t[n],q[n]).  A motion queue has
a maximum size, which can be queried (command 'gms').  If the motion queue
is empty, the robot remains in the currently commanded configuration.

Clients have two commands with which they can modify the motion queue:

1) APPEND a new milestone.  The milestone is placed at the end of the
   motion queue.

2) RESET the motion queue.  This command truncates the queue beyond a
   specified time Tr.  The resulting motion queue MQ'(t) is identical to the
   original MQ(t) up to time Tr, with its final milestone at MQ(Tr).

RESET commands are used to respond to sensor input.  A typical mode of operation
is to append as many milestones as needed in order to generate smooth motion
until the next sensor update, plus some padding Tp that covers the amount of
time needed to generate new motion commands.  Upon the sensor update, the client
sends a RESET to the controller to cut off the motion queue at time Tp.  The
client calculates a new trajectory segment (starting from the configuration
and velocity at the end of the motion queue), and then appends the segment
to the motion queue.

RESET commands are also used to start a motion sequence at a precisely
specified time.  To begin a motion at time Ts, the client should call RESET
with the time Ts.  If the motion queue is currently empty, this will set the
motion queue to a constant, ending at time Ts.  Then milestones can be
appended as needed.

There are three notions of time in this API:

- Absolute time is relative to the start of the controller program.

- Relative time is relative to whenever the command is received by the
  controller.  This is used only by the 'rtrel' (RESET, relative) command.

- Delta time is relative to the end of the current motion queue. This is used
  only used by the 'am' (APPEND) command.

Clients should use absolute time resets ('rtabs') whenever precise timing is
needed.


**** Installation ****

These steps only need to be performed once.

Step 1.
  Launch SRS and the CS8 controller.  Copy the trajServer program to the
  controller using the transfer manager.

Step 2.
  On the CS8 pendant, navigate to "Control Panel/IO/Sockets" and add a
  new socket called "server" with port 1000.  The timeout can be arbitrary,
  although setting it to 0 will not allow the server to recover in case the
  client disconnects.


**** Execution ****

Step 1.
  Launch the CS8 controller if not already running.  Open the trajServer
  program and run it.  To enable arm movement, enable network mode using
  the pendant, follow the on-screen instructions (set speed to 100%, 
  release any movement holds), and then select Start (F1).

Step 2.
  Launch your client program and launch a standard TCP/IP socket to connect
  to the controller's IP address, port 1000.

Step 3.
  Send messages from your client program and receive replies from the server
  as desired, using the protocol specified below. 

To greatly simplify the programming of steps 2 and 3, use the Python RPC
bindings in trajserver.py.  Bindings for other languages should become
available as needed.



**** Client-server message ****

First token is an identifier that is repeated on the reply message, or '*' if no reply is needed.
After a whitespace, can have 0 or more comma separated commands.
Terminated by a semicolon (;)

Format:
ID CMD1(ARGS1),CMD2(ARGS2),...,CMDn(ARGSn);



**** Server-client reply message **** 

If the ID of the message is not '*', a reply is generated.
The reply begins with  same ID of the incoming message, along with the
comma-separated return values of each command.
The reply is terminated by a semicolon.

Format:
ID RET1,RET2,...,RETn;

(In our current implementation, whatever delimiters used in the input
message are duplicated in the reply, so any delimiter that is not a
semicolon can safely be used.  To avoid confusion, whitespace and periods
are not recommended.)


**** Specifications ****

The maximum length of IDs, command names, and argument strings is limited
to 128 characters because of limitations of Staubli's internal VAL3 string
representations.  This limits the size of argument lists to approximately
15-20 floats.  This cap may be raised in future implementations.

Reply messages are limited to 4096 characters, which shouldn't pose an issue
during normal usage.  For example, this allows approximately 100 'gc'
calls to be returned in a single message.

There are currently a maximum of 500 piecewise linear segments in the
trajectory queue.  The CS8 controller runs at 250Hz, so this gives at least
2 seconds of fine-grained motion.


**** Command list ****

Miscellaneous commands
echo(s): returns the string s.
version(): returns the trajClient version string.
rate(): returns the controller's maximum cycle rate in Hz.

Robot state commands
gj(): returns the arm's currently commanded joint configuration.
gv(): returns the currently commanded joint velocities.
gx(): returns the flange's coordinate transform x,y,z,Rx,Ry,Rz (where
      Rx,Ry,Rz are the roll-pitch-yaw) relative to the robot's world frame.
gjmin(): returns the active joint minimum.
gjmax(): returns the active joint minimum.
gvl(): returns the active velocity limits.
gal(): returns the active acceleration limits.
gdl(): returns the active deceleration limits.
sjmin(qmin): sets the active joint minimum (clipped to hardware limits).
sjmax(qmax): sets the active joint maximum (clipped to hardware limits).
svl(vmax): sets the active velocity limits (clipped to hardware limits).
sal(amax): sets the active acceleration limits (clipped to hardware limits).
sdl(dmax): sets the active deceleration limits (clipped to hardware limits).

Trajectory management commands
gct(): returns the current time along the trajectory.
gd(): Returns the remaining duration of the trajectory.
get(): Returns the trajectory end time.
gms(): returns the maximum number of trajectory segments.
gcs(): returns the number of trajectory segments currently in use.
check(): returns 0 if the current trajectory satisfies joint, velocity, and
         acceleration limits, otherwise returns 1.
am(dt q): Appends the milestone q at time dt after the current end time.
rtabs(dt): Resets the trajectory at absolute time t.
rtrel(dt): Resets the trajectory at time dt past the time the message is
           received.

All variables 'qX', 'vX', 'aX', 'dX' denote 6-element joint vectors,
and are formatted with spaces between each element.


Return values

"Invalid": the command is not supported.
"Error": a (generic) semantic error occured with the arguments.
floating point precision: current implementation uses thousandths.