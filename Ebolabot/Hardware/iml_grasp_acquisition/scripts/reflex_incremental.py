#!/usr/bin/env python 
import rospy 
import roslib
roslib.load_manifest("reflex_sf_msgs")
from reflex_sf_msgs.msg import Command, JointState 
from iml_grasp_acquisition.srv import *
from threading import Lock

class reflex_close:

	def __init__(self):
		self.current_state = JointState() 
		self.pub = rospy.Publisher('reflex_hand1/command', Command)
		self.lock = Lock()

	def reflex_state_cb(self, data):
		self.lock.acquire()
		self.current_state = data
		self.lock.release()

	def close_until_force(self, req):
		self.lock.acquire()
		initial_position = self.current_state.goal_pos
		self.lock.release()

		while( all([abs(x) < 0.20 for x in self.current_state.load]) ):
			next_state = Command() 
			next_state.position = [x+.1 for x in initial_position[0:3] ] + [initial_position[3]]
			initial_position = next_state.position
			next_state.velocity = [0.1, 0.1 ,0.1, 0]
			self.pub.publish(next_state)
			rospy.sleep(0.1)

		self.lock.acquire()
		final_position = self.current_state
		self.lock.release()

		return IncrementalCloseResponse(final_position)

	def close(self, req):
		self.lock.acquire()
		initial_position = self.current_state.goal_pos
		self.lock.release()

		next_state = Command() 
		next_state.position = 3*[req.joint_position] + [initial_position[3]]
		next_state.velocity = [0.1, 0.1 ,0.1, 0.0]
		self.pub.publish(next_state)
		
		#might need a sleep and a spin_once\
		self.lock.acquire()
		final_position = self.current_state
		self.lock.release()

		return CloseResponse(final_position)

	def pinch(self, req):
		self.lock.acquire()
		initial_position = self.current_state.goal_pos
		self.lock.release()

		next_state = Command() 
		next_state.position = 2*[req.joint_position] + list(initial_position[2:4])
		print next_state.position
		next_state.velocity = [0.1, 0.1 ,0.1, 0.0]
		self.pub.publish(next_state)
		
		#might need a sleep and a spin_once\
		self.lock.acquire()
		final_position = self.current_state
		self.lock.release()

		return CloseResponse(final_position)

	def open(self, req):
		next_state = Command() 
		next_state.position = [0, 0, 0] + [self.current_state.goal_pos[3]]
		next_state.velocity = [0.1, 0.1 ,0.1, 0]
		self.pub.publish(next_state)

		#might need a sleep and a spin_once\
		self.lock.acquire()
		final_position = self.current_state
		self.lock.release()

		return OpenResponse(final_position)
		
	def preshape(self, req):
		self.lock.acquire()
		initial_position = self.current_state.goal_pos
		self.lock.release()

		next_state = Command() 
		next_state.position = list(initial_position[0:3]) + [req.preshape_position]
		next_state.velocity = [0.0, 0.0 ,0.0, 0.1]
		self.pub.publish(next_state)

		#might need a sleep and a spin_once
		self.lock.acquire()
		final_position = self.current_state
		self.lock.release()

		return PreshapeResponse(final_position)

if __name__ == '__main__':
	rf = reflex_close()
	rospy.init_node("reflex_incremental_close")
	sub = rospy.Subscriber('reflex_hand1/state', JointState, rf.reflex_state_cb )
	rospy.Service('reflex_hand1/incremental_close', IncrementalClose , rf.close_until_force)
	rospy.Service('reflex_hand1/close', Close, rf.close)
	rospy.Service('reflex_hand1/pinch', Close, rf.pinch)
	rospy.Service('reflex_hand1/open', Open, rf.open)
	rospy.Service('reflex_hand1/preshape', Preshape, rf.preshape)
	rospy.spin()
