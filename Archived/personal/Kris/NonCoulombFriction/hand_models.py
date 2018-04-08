import pkg_resources
pkg_resources.require("Klampt>=0.7.0")
from klampt import vis
from klampt.io import resource
from klampt.math import *
from klampt import *
from klampt.model import coordinates
import time

class ContactUnit:
	def __init__(self,type='Coulomb1',name=None,patchSize=0.02,localTransform=None):
		self.type = type
		self.name = name
		#2cm patch size
		self.patchSize = patchSize
		self.localTransform = localTransform if localTransform is not None else se3.identity()

class HandModel:
	def __init__(self):
		self.robot_file = None
		self.palm_name = None
		self.finger_names = []
		self.finger_link_names = []
		self.resource_directory = 'resources/empty_hand'
		#map from finger link -> map from phalange -> list of ContactUnits
		self.finger_units = {}
		#list of ContactUnits
		self.palm_units = []
		#list of (link,ContactUnit) pairs
		self.all_units = []

	def init_units(self,num_palm_units,num_finger_link_units,unit_type=None):
		"""Initializes all contact units with the given number of palm and finger units.
		Transforms and patch sizes are left as default.
		If unit_type is provided, initializes all of the units to the given type.
		"""
		self.palm_units = [ContactUnit() for i in xrange(num_palm_units)]
		for i in xrange(num_palm_units):
			self.palm_units[i].name = "palm_unit_%d"%(i+1,)
			if unit_type is not None:
				self.palm_units[i].type = unit_type
		self.finger_units = {}
		for finger in self.finger_names:
			self.finger_units[finger] = {}
			for link in self.finger_link_names:
				self.finger_units[finger][link] = [ContactUnit() for i in xrange(num_finger_link_units)]
				for i in xrange(num_finger_link_units):
					self.finger_units[finger][link][i].name = "%s_%s_unit_%d"%(finger,link,i+1)
					if unit_type is not None:
						self.finger_units[finger][link][i].type = unit_type

	def load_frames(self):
		"""Loads unit frames from disk"""
		resource.setDirectory(self.resource_directory)
		for i,u in enumerate(self.palm_units):
			u.localTransform = resource.get(u.name+".xform",type="RigidTransform",doedit=False)
			if u.localTransform == None:
				raise RuntimeError("Can't load palm frame "+str(i))
		for finger in self.finger_names:
			for link in self.finger_link_names:
				for i,u in enumerate(self.finger_units[finger][link]):
					u.localTransform = resource.get(u.name+".xform",type="RigidTransform",doedit=False)
					if u.localTransform == None:
						raise RuntimeError("Can't load finger link frame "+str(i))
		self.flatten_frames()

	def edit_frames(self):
		resource.setDirectory(self.resource_directory)
		w = WorldModel()
		w.loadElement(self.robot_file)
		robot = w.robot(0)

		num_finger_link_units = 0
		for f in self.finger_units:
			num_finger_link_units = len(self.finger_units[f])
		default_finger_unit_transforms = [se3.identity()]*num_finger_link_units
		default_palm_unit_transforms = [se3.identity()]*len(self.palm_units)

		for i in xrange(num_finger_link_units):
			default_finger_unit_transforms[i] = resource.get("default_finger_unit_%d.xform"%(i+1,),type="RigidTransform",default=default_finger_unit_transforms[i],world=w,frame=robot.link(self.finger_names[1]+":"+self.finger_link_names[i]),doedit=True)

		vis.add("gripper",robot)
		coordinates.setRobotModel(robot)
		print "Coordinates:"
		coordinates.listFrames(2)
		framenames = []
		for i,u in enumerate(self.palm_units):
			if i == 0:
				default = default_finger_unit_transforms[0]
			else:
				default = self.palm_units[i-1].localTransform
			self.palm_units[i].localTransform = resource.get(u.name+".xform",type="RigidTransform",default=default,world=w,frame=robot.link(self.palm_name),doedit=True)
			palmFrame = coordinates.frame(self.palm_name)
			coordinates.addFrame(name=u.name,parent=palmFrame,relativeCoordinates=u.localTransform)
			framenames.append(u.name)
		for finger in self.finger_names:
			for link in self.finger_link_names:
				for i,u in enumerate(self.finger_units[finger][link] ):
					u.localTransform = resource.get(u.name+".xform",type="RigidTransform",default=default_finger_unit_transforms[i],doedit=False)
					linkFrame = coordinates.frame(finger+":"+link)
					coordinates.addFrame(name=u.name,parent=linkFrame,relativeCoordinates=u.localTransform)
					framenames.append(u.name)

		vis.add("coordinates",coordinates.manager())
		vis.listItems()
		for n in framenames:
			vis.edit(("coordinates",n))

		#pop up the window for editing and wait until closed
		vis.show()
		while vis.shown():
			time.sleep(0.1)
		vis.kill()
		self.flatten_frames()

	def flatten_frames(self):
		"""Builds the all_unit_transforms list"""
		self.all_units= []
		for T in self.palm_units:
			self.all_units.append((self.palm_name,T))
		for fingername,finger in self.finger_units.iteritems():
			for link,unitlist in finger.iteritems():
				flatname = fingername+':'+link
				for unit in unitlist:
					self.all_units.append((flatname,unit))

	def save_frames(self):
		resource.setDirectory(self.resource_directory)
		for i,u in enumerate(self.palm_units):
			resource.set(u.name+".xform",value = u.localTransform,type="RigidTransform")
		for finger in self.finger_names:
			for link in self.finger_link_names:
				for i,u in enumerate(self.finger_units[finger][link]):
					resource.set(u.name+".xform",type="RigidTransform",value=u.localTransform)


def spiny_hand():
	#SpinyHand setup
	hand = HandModel()
	hand.robot_file = "robots/hand_default.rob"
	hand.palm_name = "Link 0"
	hand.finger_names = ["finger1","finger2","finger3","finger4"]
	hand.finger_link_names = ["Link 0","Link 1","Link 2"]
	hand.init_units(num_palm_units=8,num_finger_link_units=1,unit_type='StanfordMicroSpineUnit')
	hand.resource_directory = 'resources/spinyhand'
	hand.palm_units.append(ContactUnit("Coulomb1","hook_inner1",patchSize=0.01))
	hand.palm_units.append(ContactUnit("Coulomb1","hook_inner2",patchSize=0.01))
	hand.palm_units.append(ContactUnit("Coulomb1","hook_outer1",patchSize=0.01))
	hand.palm_units.append(ContactUnit("Coulomb1","hook_outer2",patchSize=0.01))
	hand.palm_units.append(ContactUnit("Coulomb1","hook_tip1",patchSize=0.003))
	hand.palm_units.append(ContactUnit("Coulomb1","hook_tip2",patchSize=0.003))
	#hand.palm_units.append(ContactUnit("Coulomb1","hook_tip3",patchSize=0.003))
	#hand.palm_units.append(ContactUnit("Coulomb1","hook_tip4",patchSize=0.003))
	return hand

def spiny_twofinger():
	#two-finger passive gripper setup
	hand = HandModel()
	hand.robot_file = "robots/twofinger_hand.rob"
	hand.palm_name = "Link 0"
	hand.finger_names = ["finger1","finger2"]
	hand.finger_link_names = ["Link 0","Link 1","Link 2"]
	hand.init_units(num_palm_units=0,num_finger_link_units=1,unit_type='StanfordMicroSpineUnit')
	hand.resource_directory = 'resources/twofinger'
	return hand

def spiny_opposing():
	#two opposing links
	hand = HandModel()
	hand.robot_file = "robots/opposing.rob"
	hand.palm_name = "Link 0"
	hand.finger_names = ["finger1","finger2"]
	hand.finger_link_names = ["Link"]
	hand.init_units(num_palm_units=0,num_finger_link_units=1,unit_type='StanfordMicroSpineUnit')
	hand.resource_directory = 'resources/opposing'
	return hand

hands = {'spiny_hand':spiny_hand(),
		'spiny_twofinger':spiny_twofinger(),
		'spiny_opposing':spiny_opposing()
		}

if __name__ == '__main__':
	import sys
	h = spiny_hand()
	#h = spiny_opposing()
	if len(sys.argv) > 1:
		if sys.argv[1] not in hands.keys():
			print "Available hands:",hands.keys()
			exit(1)
		h = hands[sys.argv[1]]
	h.edit_frames()
	while True:
		response = raw_input("Save to disk? y/n >")
		if response[0]=='y':
			h.save_frames()
			break
		elif response[0]=='n':
			break
		else:
			print "Please enter y/n"

