import sys
ebolabot_root = "/home/motion/iml-internal/Ebolabot/"
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config
from klampt import se3, so3
from time import sleep

# Connect to Baxter
config.mode = "physical"
print "Loading Motion Module model",config.klampt_model
robot = motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
res = robot.startup()
if not robot.isStarted():
    res = robot.startup()
    if not res:
        raise RuntimeError("Ebolabot Motion could not be started")
else:
    print "Robot started by another process"

def roundEachAndPrint(myList,decimals):
    code = '%.'+str(decimals)+'f'
    print [ code % elem for elem in myList ]

print('Script Start')

'''Reference points'''
frwrd = [0,0,-1,0,1,0,1,0,0] # rotation relative to global/baxter fram to point hands/z forward
ext = [0.91,-0.41,1.46]

'''Transforms'''
T_target = [frwrd,ext]
T_EEtoPPU = [[-1,0,0,0,0,1,0,1,0], [0,0,0]]

# '''Correction loop'''
# while True:
# 	T_EE = robot.right_ee.sensedTransform()
# 	R_PPU = so3.mul(T_EEtoPPU[0],T_EE[0])
# 	roundEachAndPrint(R_PPU,3)

while True:
	T_bestEE = se3.mul(se3.inv(T_EEtoPPU),T_target)
	T_EEcmd = T_bestEE
	robot.right_ee.moveTo(T_EEcmd[0],T_EEcmd[1])

# while True:
# 	transform = robot.right_ee.sensedTransform()
# 	position = transform[1]
# 	rotation = transform[0]
# 	roundEachAndPrint(rotation,3)



















"""


def flatRto2D(R):
	return [R[0:3],R[3:6],R[6:]]
def R2DtoFlat(R):
	return R[0]+R[1]+R[2]



# print('saving position')
# transform = robot.right_ee.sensedTransform()
# R = transform[0]
# t = transform[1]
# while True:
# 	sleep(3)
# 	print('moving back')
# 	robot.right_ee.moveTo(R,t)

'''Target positions'''
frwrd = [0,0,-1,0,1,0,1,0,0]
up = [1,0,0,0,1,0,0,0,1]
loc = [0.91,-0.41,1.46]

'''Transforms'''
R_EEtoPPU = [-1,0,0,0,0,-1,0,-1,0]
t_EEtoPPU = [0.062,0.013,0.060]

'''Test Math'''
# R_BtoEE, t_BtoEE = robot.right_ee.sensedTransform()
print numpy.linalg.matmul(frwrd,up)

# while True:
# 	robot.right_ee.moveTo(R_BtoPPU,vectorops.sub(loc,t_EEtoPPU))

# while True:
# 	transform = robot.right_ee.sensedTransform()
# 	position = transform[1]
# 	rotation = transform[0]
# 	roundEachAndPrint(rotation,3)

print('Script End')
"""