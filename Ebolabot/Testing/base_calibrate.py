from klampt import *
import math
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config
import csv

trace_keys=['t','x','y','theta','vx','vy','vtheta','vxcmd','vycmd','vthetacmd']

trace = dict((k,[]) for k in trace_keys)

def update_trace():
    global trace
    trace['t'].append(time.time())
    pos = motion.robot.base.odometryPosition()
    vel = motion.robot.base.velocity()
    velcmd = motion.robot.base.commandedVelocity()
    if pos==False:
        pos=[0,0,0]
    if vel==False:
        vel=[0,0,0]
    if velcmd==False:
        velcmd=[0,0,0]
    trace['x'].append(pos[0])
    trace['y'].append(pos[1])
    trace['theta'].append(pos[2])
    trace['vx'].append(vel[0])
    trace['vy'].append(vel[1])
    trace['vtheta'].append(vel[2])
    trace['vxcmd'].append(velcmd[0])
    trace['vycmd'].append(velcmd[1])
    trace['vthetacmd'].append(velcmd[2])

def move_constant(cmd,duration):
    t0 = time.time()
    while time.time()-t0 < duration:
        motion.robot.base.moveVelocity(*cmd)
        update_trace()
        time.sleep(0.01)
    motion.robot.base.stop()

def move_trajectory(cmd,duration):
    t0 = time.time()
    while time.time()-t0 < duration:
        motion.robot.base.moveVelocity(*cmd(time.time()-t0))
        update_trace()
        time.sleep(0.01)
    motion.robot.base.stop()

def dosquare(speed,size):
    print "Moving at speed",speed,"along square of size",size
    side_duration = size / speed
    move_constant([speed,0,0],side_duration)
    move_constant([0,speed,0],side_duration)
    move_constant([-speed,0,0],side_duration)
    move_constant([0,-speed,0],side_duration)

def dowiggle(speed,amount,axis=2):
    print "Wiggling at speed",speed,"amount",amount,"on axis",axis
    side_duration = amount / speed
    cmd = [0,0,0]
    cmd[axis] = speed
    move_constant(cmd,side_duration)
    cmd[axis] = -speed
    move_constant(cmd,side_duration)
    cmd[axis] = -speed
    move_constant(cmd,side_duration)
    cmd[axis] = speed
    move_constant(cmd,side_duration)

if __name__ == "__main__":
    config.parse_args()
    print "base_calibrate.py: gets data about driving the base"
    print
    print "Loading Motion Module model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",)
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)
    if not motion.robot.base.enabled():
        print "Base is not enabled..."
        exit(1)

    t0 = time.time()
    """
    dosquare(0.05,0.3)
    move_constant([0,0,0],0.5)
    dosquare(0.1,0.3)
    move_constant([0,0,0],0.5)
    dosquare(0.2,0.3)
    move_constant([0,0,0],0.5)
    """
    speeds = [0.025,0.05,0.1,0.15,0.2,0.3,0.4]
    #wiggle x or y
    scale = 1.0
    axis = 1
    for s in speeds:
        dowiggle(s,s*scale,axis)
        move_constant([0,0,0],0.5)
    """
    #wiggle theta
    scale = 2.0
    axis = 2
    for s in speeds:
        dowiggle(s,s*scale,axis)
        move_constant([0,0,0],0.5)
    """
    global trace,trace_keys
    for i,t in enumerate(trace['t']):
        trace['t'][i] = t-t0
    print "Now saving trace to base_calibrate.csv"
    with open('base_calibrate.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(trace_keys)
        for i in range(len(trace['t'])):
            writer.writerow([trace[k][i] for k in trace_keys])
    print "Done."
    motion.robot.shutdown()
    print "Saved to base_calibrate.csv."
