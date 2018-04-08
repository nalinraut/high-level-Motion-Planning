"""Common code for handling free-floating moving bases"""

from klampt.math import so3

def make_moving_base_robot(robotfile,world,name=None,tempfile="temp.rob",floating=False):
	"""Converts the given fixed-base robot into a moving base robot
	and loads it into the given world.

	Arguments:
	- robotfile: the .rob or .urdf file that should be mounted onto the moving base.
	- world: a RobotWorld instance
	- name: a name of the robot subgroup excluding the floating base.  If none, everything
	  is flattened into a single flat robot.
	- tempfile: the name of the temporary file that will be saved to disk
	- floating: if true, the moving base is set up with a floating joint.  This cannot
	  be actuated in a normal way.  If false, it is set up with x,y,z linear actuators
	  and yaw,pitch,roll revolute actuators.  The advantage of floating joints is that
	  editors realize what it is and a transform widget is properly instantiated.
	Return value is the RobotModel of the created robot.

	Note: saves a .rob file to disk as a temporary file.  Doesn't delete this file.
	"""
	pattern = """
### Boilerplate kinematics of a floating (translating and rotating) cube with a robot hand mounted on it
TParent 1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  
parents -1 0 1 2 3 4 
axis 1 0 0   0 1 0    0 0 1     0 0 1     0 1 0     1 0 0 
jointtype p p p r r r 
links x y z yaw pitch roll
qMin -1 -1 -1  -inf -inf -inf
qMax  1  1  1   inf  inf  inf 
q 0 0 0 0 0 0 
geometry   ""   ""   ""   ""    ""    ""
mass       0.1 0.1 0.1 0.1 0.1 0.1
com 0 0 0   0 0 0   0 0 0   0 0 0   0 0 0   0 0 0   
inertia 0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 
torqueMax  500 500 500 50 50 50 
accMax     4 4 4 4 4 4 4
velMax     2 2 2 3 3 3

property sensors <sensors><ForceTorqueSensor link="5" hasForce="1 1 1" hasTorque="1 1 1" /></sensors>
mount 5 "%s" 1 0 0   0 1 0   0 0 1   0 0 0 %s
"""

	if not floating:
		pattern += """joint normal 0
joint normal 1
joint normal 2
joint spin 3
joint spin 4
joint spin 5

driver normal 0 
driver normal 1
driver normal 2
driver normal 3
driver normal 4
driver normal 5

servoP 5000 5000 5000 500 500 500
servoI 10 10 10 .5 .5 .5
servoD 100 100 100 10 10 10
viscousFriction 50 50 50 50 50 50
dryFriction 1 1 1 1 1 1
"""
	else:
		pattern += """joint floating 5 -1
"""

	import os
	f = open(tempfile,'w')
	f.write(pattern 
		% (robotfile,("as "+name if name else "")))
	f.close()
	res = world.loadElement(tempfile)
	if res < 0:
		raise IOError("Invalid robot file, use RobotTest to inspect "+tempfile)
	world.robot(world.numRobots()-1).setName("floating_"+(os.path.basename(robotfile).split(".")[0] if name == None else name))
	return world.robot(world.numRobots()-1)


def get_moving_base_xform(robot):
	"""For a moving base robot model, returns the current base rotation
	matrix R and translation t."""
	return robot.link(5).getTransform()

def set_moving_base_xform(robot,R,t):
	"""For a moving base robot model, set the current base rotation
	matrix R and translation t.  (Note: if you are controlling a robot
	during simulation, use send_moving_base_xform_command)
	"""
	q = robot.getConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	robot.setConfig(q)

def send_moving_base_xform_linear(controller,R,t,dt):
	"""For a moving base robot model, send a command to move to the
	rotation matrix R and translation t using linear interpolation
	over the duration dt.

	Note: can't currently set robot commands and linear base commands
	simultaneously.  If you want to do this, you'll have to wrap
	your own base trajectory controller.
	"""
	q = controller.getCommandedConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	controller.setLinear(q,dt)

def send_moving_base_xform_PID(controller,R,t):
	"""For a moving base robot model, send a command to move to the
	rotation matrix R and translation t by setting the PID setpoint

	Note: can't currently set robot commands and linear base commands
	simultaneously.  If you want to do this, you'll have to wrap
	your own base trajectory controller.
	"""
	q = controller.getCommandedConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	v = controller.getCommandedVelocity()
	controller.setPIDCommand(q,v)
