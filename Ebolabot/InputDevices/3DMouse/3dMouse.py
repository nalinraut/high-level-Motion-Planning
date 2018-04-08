'''
3dMouse.py - 3D Mouse cartesian control of baxter robot
'''

import threading
import os
import time
# import sspp
from Motion import motion
from evdev import InputDevice, categorize, ecodes

# initializing vars
tx = 0  # ABS_X event code: 0
ty = 0  # ABS_Y event code: 1
tz = 0  # ABS_Z event code: 2
rx = 0  # ABS_RX event code: 3
ry = 0  # ABS_RY event code: 4
rz = 0  # ABS_RZ event code: 5

sf = 500.0  # scaling factor (range of each input [-500,500])

dev = InputDevice('/dev/input/event9')
print(dev)


# class MyService(sspp.Service):
#	def __init__(self):
#		Service.__init__(self)
#		self.open(('localhost',port), asServer=True)
#	def onUpdate(self):
#		self.sendMessage([tx,ty,tz,rx,ry,rz])

def my_event_reader_func():
    global tx, ty, tz, rx, ry, rz
    for event in dev.read_loop():
        if event.code == 0:  # x translation
            tx += event.value / sf
        elif event.code == 1:  # y translation
            ty += event.value / sf
        elif event.code == 2:  # z translation
            tz += event.value / sf
        elif event.code == 3:  # x rotation
            rx += event.value / sf
        elif event.code == 4:  # y rotation
            ry += event.value / sf
        elif event.code == 5:  # z rotation
            rz += event.value / sf
        else:
            print 'Unrecognized input'


def run_event_reader():
    myThread = threading.Thread(target=my_event_reader_func)
    myThread.start()


def main():
    global tx, ty, tz, rx, ry, rz
    klampt_model = os.path.join(os.path.expanduser("~"), "Klampt/data/robots/baxter_col.rob")
    mode = "physical"
    robot = motion.setup(mode=mode, klampt_model=klampt_model)
    res = robot.startup()
    run_event_reader()
    while (True):
        print [rx, ry, rz], [tx, ty, tz]
        robot.left_ee.velocityCommand([rx, ry, rz], [tx, ty, tz])
        print(robot.left_ee.commandedVelocity())
        tx = 0  # ABS_X event code: 0
        ty = 0  # ABS_Y event code: 1
        tz = 0  # ABS_Z event code: 2
        rx = 0  # ABS_RX event code: 3
        ry = 0  # ABS_RY event code: 4
        rz = 0  # ABS_RZ event code: 5
        time.sleep(0.05)  # set rate to 20Hz


main()
