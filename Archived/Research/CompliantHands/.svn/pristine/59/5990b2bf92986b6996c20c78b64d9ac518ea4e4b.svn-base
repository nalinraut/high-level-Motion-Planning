#!/usr/bin/env python

from klampt import se3,vectorops
from klampt import *
from klampt.glprogram import *
import numpy.random
import numpy.linalg
from simlog import *

has_moving_base = True

DRAW_COMMANDED = False

AUTOMATIC_MODE = True

# for all experiments, use low effort (3.3). For high effort experiments, scale that value by 3/2 (as in physical exps)
THRESHOLD_EFFORT = 3.3
HIGH_EFFORT = THRESHOLD_EFFORT*3.0/2.0 # if effort was not .2, it was .3
high_effort_experiments = ["heinz_P1", "heinz_P2", "heinz_P3", "hammer_P1", "hammer_P2", "hammer_P3"]

# by default, move the hand by 0.05m in a random direction. If the experiment is in the custom_pert dict, use that value
DEFAULT_SHAKING_PERT = 0.05
custom_pert = {}

#The hardware name
gripper_name = 'reflex'

#The Klamp't model file name
klampt_model_name = 'reflex_col_with_moving_base.rob'

#the number of Klamp't model DOFs
numDofs = 16

#The number of command dimensions
numCommandDims = 4

#The names of the command dimensions
commandNames = ['finger1','finger2','finger3','preshape']

#default postures
openCommand = [1,1,1,0]
closeCommand = [0,0,0,0]
pinchOpenCommand = [0,0,0,1]
pinchCloseCommand = [1,1,0,1]

#named preset list
presets = {'open':openCommand,
           'closed':closeCommand,
           'pinch-open':pinchOpenCommand,
           'pinch-close':pinchCloseCommand
           }

#range of postures
commandMinimum = [0,0,0,0]
commandMaximum = [1,1,1,1]

#range of valid command velocities
commandMinimumVelocity = [-1,-1,-1,-1]
commandMaximumVelocity = [1,1,1,1]

swivel_links = [2,7]
proximal_links = [3,8,12]
distal_links = [4,9,13]

class HandModel:
    """A kinematic model of the Reflex hand"""
    def __init__(self,robot,link_offset=0,driver_offset=0):
        global swivel_links,proximal_links,distal_links
        self.robot = robot
        self.link_offset = link_offset
        self.driver_offset = driver_offset
        qmin,qmax = self.robot.getJointLimits()
        self.preshape_driver = self.driver_offset
        self.swivel_links = [link_offset+i for i in swivel_links]
        self.proximal_links = [link_offset+i for i in proximal_links]
        self.distal_links = [link_offset+i for i in distal_links]
        self.proximal_drivers = [self.driver_offset+1,self.driver_offset+6,self.driver_offset+10]
        self.jointLimits = ([qmin[link_offset+proximal_links[0]],qmin[link_offset+proximal_links[1]],qmin[link_offset+proximal_links[2]],0],
                            [qmax[link_offset+proximal_links[0]],qmax[link_offset+proximal_links[1]],qmax[link_offset+proximal_links[2]],0])
    def numCommands(self):
        return 4
    def commandNames(self):
        global commandNames
        return commandNames
    def domain(self):
        #closure joints: 0=closed, 1=open
        #preshape joint 3: 0=power, 1=pinch
        return ([0,0,0,0],[1,1,1,1])
    def getCommand(self):
        preshape = self.robot.driver(self.preshape_driver).getValue()
        qrob = self.robot.getConfig()
        qmin,qmax = self.jointLimits
        fingers = [(qrob[self.proximal_links[i]]-qmax[i])/(qmin[i]-qmax[i]) for i in range(3)]
        return fingers+[preshape]
    def getVelocity(self):
        preshape = self.robot.driver(self.preshape_driver).getVelocity()
        vrob = self.robot.getVelocity()
        qmin,qmax = self.jointLimits
        fingers = [(vrob[self.proximal_links[i]])/(qmin[i]-qmax[i]) for i in range(3)]
        return fingers+[preshape]
    def setCommand(self,command):
        """Sets the configuration of self.robot given a hand config"""
        assert len(command)==4,"Reflex hand has 4 DOFS"
        [finger1,finger2,finger3,preshape] = command
        fingers = [finger1,finger2,finger3]
        self.robot.driver(self.preshape_driver).setValue(preshape)
        qrob = self.robot.getConfig()
        qmin,qmax = self.jointLimits
        for i in range(3):
            qrob[self.proximal_links[i]] = qmax[i]+fingers[i]*(qmin[i]-qmax[i])
            qrob[self.distal_links[i]] = fingers[i]*1.9
        self.robot.setConfig(qrob)
    def setVelocity(self,vel):
        """Sets the command velocity of self.robot"""
        assert len(config)==4,"Reflex hand has 4 DOFS"
        [finger1,finger2,finger3,preshape] = vel
        fingers = [finger1,finger2,finger3]
        self.robot.driver(self.preshape_driver).setVelocity(preshape)
        vrob = self.robot.getVelocity()
        qmin,qmax = self.jointLimits
        for i in range(3):
            vrob[self.proximal_links[i]] = vel[i]*(qmin[i]-qmax[i])
            vrob[self.distal_links[i]] = fingers[i]*1.9
        self.robot.setVelocity(vrob)

class HandSim:
    def __init__(self,sim,world,robotindex=0,link_offset=0,driver_offset=0):
        self.x = 6*[0.0] #floating base
        self.sim = sim
        self.logger = SimLogger(sim,"log_reflex_state.csv","log_reflex_contact.csv",saveheader=False)
        self.controller = self.sim.controller(robotindex)
        #rubber for pad
        pad = self.sim.body(world.robotLink(robotindex,link_offset+1))
        #s = pad.getSurface()
        #s.kFriction = 1.5
        #s.kStiffness = 600000
        #s.kDamping = 60000
        #pad.setCollisionPadding(0.005)
        #pad.setSurface(s)
        fingerpads = [link_offset+5,link_offset+6,link_offset+10,link_offset+11,link_offset+14,link_offset+15]
        for l in fingerpads:
            pad = self.sim.body(world.robotLink(robotindex,l))
            #s = pad.getSurface()
            #s.kFriction = 1.5
            #pad.setCollisionPadding(0.0025)
            #pad.setSurface(s)
        self.world = world
        self.model = HandModel(world.robot(robotindex),link_offset,driver_offset)
        self.setpoint = self.model.getCommand()
        self.endpoint = self.setpoint[:]
        self.force = [0,0,0,0]
        self.moving = [0,0,0,0]
        # should we apply forces on the promixal link of finger [0, 1, 2]?
        self.ext_forces = [False, False, False]
        self.EXT_F = [0.0, 0.0, -150.0]
        self.EXT_F_DISP = [0.025, 0.00, 0.0]
        self.base_setpoint = None

        self.update_tendon_lengths()
        print "Reflex Hand Simulation initialized"
        print "  Initial setpoint", self.setpoint
        print "  Rest tendon lengths:", self.tendon_lengths
        #attachment points of proximal / distal joints,
        #relative to center of mass frames
        self.tendon1_local = [0.035,0,0.015]
        self.tendon2_local = [-0.015,0,0.012]
        self.logger.saveHeader(['cmd_setpoint1','cmd_setpoint2','cmd_setpoint3','cmd_setpoint4','tendon_length1','tendon_length2','tendon_length3'])
        self.logger.saveContactHeader()

    def getCommand(self):
        return self.endpoint

    def setCommand(self,command):
        self.endpoint = [max(min(v,1),0) for v in command]

    def setFinger1(self,value):
        self.endpoint[0] = max(min(value,1),0)

    def setFinger2(self,value):
        self.endpoint[1] = max(min(value,1),0)

    def setFinger3(self,value):
        self.endpoint[2] = max(min(value,1),0)

    def setPreshape(self,value):
        self.endpoint[3] = max(min(value,1),0)

    def update_tendon_lengths(self):
        #drive system:
        #find deviation between commanded and actual on proximal joint, use
        #that to determine tendon lengths
        qcmd = self.controller.getCommandedConfig()
        qactual = self.controller.getSensedConfig()
        pulls = [qcmd[l] - qactual[l] for l in self.model.proximal_links]
        #print pulls
        pullscale = 0.5
        #pullscale = 0.75
        rubber_length = 0.0215
        self.tendon_lengths = [0,0,0]
        self.tendon_lengths[0] = max(0,1.0-pulls[0]*pullscale)*rubber_length
        self.tendon_lengths[1] = max(0,1.0-pulls[1]*pullscale)*rubber_length
        self.tendon_lengths[2] = max(0,1.0-pulls[2]*pullscale)*rubber_length

    def controlLoop(self,dt):
        for i in range(4):
            speed = 2
            if self.endpoint[i] < self.setpoint[i]:
                self.setpoint[i] = max(self.setpoint[i]-speed*dt,self.endpoint[i])
            elif self.endpoint[i] > self.setpoint[i]:
                self.setpoint[i] = min(self.setpoint[i]+speed*dt,self.endpoint[i])
		# clean the robot state -
        self.model.robot.setConfig([0]*self.model.robot.numLinks())
        self.model.setCommand(self.setpoint)
        q = self.model.robot.getConfig()
        qcmd = self.controller.getCommandedConfig()
        qcmd[self.model.swivel_links[0]] = q[self.model.swivel_links[0]]
        qcmd[self.model.swivel_links[1]] = q[self.model.swivel_links[1]]
        qcmd[self.model.proximal_links[0]] = q[self.model.proximal_links[0]]
        qcmd[self.model.proximal_links[1]] = q[self.model.proximal_links[1]]
        qcmd[self.model.proximal_links[2]] = q[self.model.proximal_links[2]]

        #if has_moving_base:
        if has_moving_base and self.base_setpoint != None:
            for i in xrange(5):
                qcmd[i] = self.base_setpoint[i]

        vcmd = self.controller.getCommandedVelocity()
        self.controller.setPIDCommand(qcmd,vcmd)
        
        self.logger.saveStep(self.setpoint+self.tendon_lengths)

    def simLoop(self,dt):
        #apply forces associated with tendon
        self.model.setCommand(self.setpoint)
        self.update_tendon_lengths()
        for i in range(3):
            if self.ext_forces[i]:
                self.apply_external_forces(self.model.proximal_links[i])
            self.apply_tendon_forces(self.model.proximal_links[i],self.model.distal_links[i],self.tendon_lengths[i])

    def apply_external_forces(self, link_index):
        b = self.sim.body(self.model.robot.link(link_index))
        b.applyForceAtLocalPoint(self.EXT_F, self.EXT_F_DISP)

    def apply_tendon_forces(self,link1,link2,rest_length):
        tendon_c2 = 30000.0
        tendon_c1 = 100000.0
        b1 = self.sim.body(self.model.robot.link(link1))
        b2 = self.sim.body(self.model.robot.link(link2))
        p1w = se3.apply(b1.getTransform(),self.tendon1_local)
        p2w = se3.apply(b2.getTransform(),self.tendon2_local)

        d = vectorops.distance(p1w,p2w)
        if d > rest_length:
            #apply tendon force
            direction = vectorops.unit(vectorops.sub(p2w,p1w))
            f = tendon_c2*(d - rest_length)**2+tendon_c1*(d - rest_length)
            #print "d",d,"rest length",rest_length,"force",f
            b1.applyForceAtPoint(vectorops.mul(direction,f),p1w)
            b2.applyForceAtPoint(vectorops.mul(direction,-f),p2w)
        else:
            #print "d",d,"rest length",rest_length
            pass

class HandSimGLViewer(GLRealtimeProgram):
    def __init__(self, handsim, world_name = None):
        GLRealtimeProgram.__init__(self,"Reflex simulation program")
        self.handsim = handsim
        self.sim = handsim.sim

        self.drivers = [self.handsim.model.robot.driver(d) for d in xrange(self.handsim.model.robot.numDrivers())]

        self.world = handsim.world
        self.world_name = world_name

        self.simulate = AUTOMATIC_MODE
        self.auto_close = AUTOMATIC_MODE
        self.auto_close_idle_duration = 0.5
        self.auto_close_idle = None


        self.shaking = False
        self.num_shakes = 6 # 6 shakes, one every 0.5 secs, for 3.0 secs total
        self.shaking_interval = 0.5
        self.next_shake = None
        self.remaining_shakes = None

        self.lifting = False
        self.lifting_duration = 5.0 + self.auto_close_idle_duration # wait for auto_close_idle before lifting
        self.lifting_timeout = None

        self.settling_time = 1.0
        self.auto_start_timeout = None

        self.threshold_effort = THRESHOLD_EFFORT

        self.stats = {"S0":None,"S1": None, "S2": None, "S3": None}

        if self.world_name is not None:
            if self.world_name.split(".") in high_effort_experiments:
                self.threshold_effort = HIGH_EFFORT

        self.object = None
        self.initial_obj_com = None
        if world.numRigidObjects() > 0:
            self.object = world.rigidObject(0)
            self.initial_obj_com = self.getObjectGlobalCom()

        self.control_dt = 0.01
        self.sim_substeps = 10

        self.handsim.base_setpoint = 6*[0.0]

        #press 'c' to toggle display contact points / forces
        self.drawContacts = False

    def getObjectGlobalCom(self):
        return se3.apply(self.object.getTransform(), self.object.getMass().getCom())

    def shakeObj(self):
        print "shaking object"
        shake_pert = DEFAULT_SHAKING_PERT
        if self.world_name.split("_") in custom_pert.keys():
            shake_pert = DEFAULT_SHAKING_PERT

        random_dist = (numpy.random.random_sample(3) - 0.5)
        random_dist /= numpy.linalg.norm(random_dist)
        random_dist *= shake_pert

        for i in xrange(3):
            self.x_des[i] += random_dist[i]

    def saveExperimentStatistics(self):
        if self.world_name is not None:
            import pickle
            self.stats["S0"] = self.checkObjectIsGrasped()
            f = file(self.world_name.split(".")[0]+'.pickle',"w+")
            pickle.dump(self.stats, f)

    def checkObjectIsGrasped(self):
        if self.initial_obj_com is not None:
            return self.getObjectGlobalCom()[2] > self.initial_obj_com[2]
        return False

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world in grey and the
        #commanded configurations in transparent green
        self.sim.updateWorld()
        self.world.drawGL()

        #draw tendons and forces
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glLineWidth(4.0)
        glBegin(GL_LINES)
        for i in range(3):
            glColor3f(0,1,1)
            b1 = self.sim.body(self.handsim.model.robot.link(self.handsim.model.proximal_links[i]))
            b2 = self.sim.body(self.handsim.model.robot.link(self.handsim.model.distal_links[i]))
            glVertex3f(*se3.apply(b1.getTransform(),self.handsim.tendon1_local))
            glVertex3f(*se3.apply(b2.getTransform(),self.handsim.tendon2_local))
            if self.handsim.ext_forces[i]:
                glColor3f(0,1,0)
                b = self.sim.body(self.handsim.model.robot.link(self.handsim.model.proximal_links[i]))
                glVertex3f(*se3.apply(b.getTransform(),self.handsim.EXT_F_DISP))
                glVertex3f(*se3.apply(b.getTransform(),[self.handsim.EXT_F_DISP[i] -f/25.0 for i,f in enumerate(self.handsim.EXT_F)]))
        glEnd()
        glLineWidth(1)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        #draw commanded configurations
        if DRAW_COMMANDED:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
            for i in xrange(self.world.numRobots()):
                r = self.world.robot(i)
                q = self.sim.controller(i).getCommandedConfig()
                r.setConfig(q)
                r.drawGL(False)
            glDisable(GL_BLEND)

        #draw contacts, if enabled
        if self.drawContacts:
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            glEnable(GL_POINT_SMOOTH)
            glColor3f(1,1,0)
            glLineWidth(1.0)
            glPointSize(5.0)
            forceLen = 0.5  #scale of forces
            maxid = self.world.numIDs()
            for i in xrange(maxid):
                for j in xrange(i+1,maxid):
                    points = self.sim.getContacts(i,j)
                    if len(points) > 0:
                        forces = self.sim.getContactForces(i,j)
                        glBegin(GL_POINTS)
                        for p in points:
                            glVertex3f(*p[0:3])
                        glEnd()
                        glBegin(GL_LINES)
                        for p,f in zip(points,forces):
                            glVertex3f(*p[0:3])
                            glVertex3f(*vectorops.madd(p[0:3],f,forceLen))
                        glEnd()
            glEnable(GL_DEPTH_TEST)

    def control_loop(self):
        global AUTOMATIC_MODE

        #external control loop
        #print "Time",self.sim.getTime()

        tau_proximal = [t for i,t in enumerate(self.handsim.sim.getActualTorques(0)) if self.drivers[i].getAffectedLink() in self.handsim.model.proximal_links]
        #tau_distal = [t for i,t in enumerate(self.handsim.sim.getActualTorques(0)) if drivers[i].getAffectedLink() in self.handsim.model.distal_links]
        #d_n = self.handsim.model.robot.driver(self.handsim.model.robot.link(self.handsim.model.proximal_links[0]).getName())
        #assert(d_n.getAffectedLink() is self.handsim.model.robot.link(self.handsim.model.proximal_links[0]).getIndex())

        #print tau_proximal
        #print tau_distal

        if AUTOMATIC_MODE:
            self.auto_start_timeout = self.ttotal + self.settling_time

            if self.auto_start_timeout <= self.ttotal:
                self.auto_close = True
                AUTOMATIC_MODE = False

        if self.auto_close:
            if all(tau < self.threshold_effort for tau in tau_proximal):
                u = self.handsim.getCommand()
                u = vectorops.sub(u, 0.01)
                self.handsim.setCommand(u)
            else:
                self.auto_close = False
                print "Automatic Closing:",self.auto_close

                if AUTOMATIC_MODE:
                    self.lifting = True


        if has_moving_base:
            q_rob = self.handsim.model.robot.getConfig()

            for i in xrange(6):
                q_rob[i] = self.handsim.base_setpoint[i]

            self.handsim.model.robot.setConfig(q_rob)

        if self.lifting:
            if self.lifting_timeout is None:
                self.lifting_timeout = self.ttotal + self.lifting_duration
                self.auto_close_idle = self.ttotal + self.auto_close_idle_duration

            if self.auto_close_idle <= self.ttotal:
                w_T_base = self.handsim.model.robot.link(0).getTransform()
                world_up = [0,0,0.2]
                base_up = se3.apply_rotation(se3.inv(w_T_base), world_up)
                for i in xrange(3):
                    self.handsim.base_setpoint[i] = base_up[i]*(self.ttotal-self.auto_close_idle)/(self.lifting_duration-self.auto_close_idle_duration)

            if self.lifting_timeout <= self.ttotal:
                self.lifting_timeout = None
                self.lifting = False
                print "Lifting:", self.lifting

                if AUTOMATIC_MODE:
                    print "Object grasped:", self.checkObjectIsGrasped()
                    self.saveExperimentStatistics()
                    sys.exit(0)

        if self.shaking:
            if self.remaining_shakes is None:
                self.remaining_shakes = self.num_shakes

            if self.remaining_shakes > 0:
                if self.next_shake is None:
                    self.next_shake = self.ttotal + self.shaking_interval

                if self.next_shake <= self.ttotal:
                    self.next_shake = None
                    self.shakeObj()
                    self.remaining_shakes -= 1
            else:
                self.remaining_shakes = None
                self.shaking = not self.shaking
                print "Shaking:", self.shaking

        return

    def idle(self):
        if self.simulate:
            t = 0
            while t < self.dt:
                control_step = min(self.control_dt,self.dt-t)
                self.control_loop()
                self.handsim.controlLoop(control_step)
                for x in range(self.sim_substeps):
                    self.handsim.simLoop(control_step/self.sim_substeps)
                    self.sim.simulate(control_step/self.sim_substeps)
                t += self.control_dt
            glutPostRedisplay()

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        if c=='a':
            self.auto_close = True
            print "Automatic Closing:", self.auto_close
            print "Threshold effort:",  self.threshold_effort
        if c=='d' and has_moving_base:
            self.lifting = True
            print "Lifting:", self.lifting
        if c=='w':
            self.shaking = not self.shaking
            print "Shaking:", self.shaking
        if c=='y':
            u = self.handsim.getCommand()
            u[0] += 0.1
            self.handsim.setCommand(u)
        elif c=='h':
            u = self.handsim.getCommand()
            u[0] -= 0.1
            self.handsim.setCommand(u)
        elif c=='u':
            u = self.handsim.getCommand()
            u[1] += 0.1
            self.handsim.setCommand(u)
        elif c=='j':
            u = self.handsim.getCommand()
            u[1] -= 0.1
            self.handsim.setCommand(u)
        elif c=='i':
            u = self.handsim.getCommand()
            u[2] += 0.1
            self.handsim.setCommand(u)
        elif c=='k':
            u = self.handsim.getCommand()
            u[2] -= 0.1
            self.handsim.setCommand(u)
        elif c=='o':
            u = self.handsim.getCommand()
            u[3] += 0.1
            self.handsim.setCommand(u)
        elif c=='l':
            u = self.handsim.getCommand()
            u[3] -= 0.1
            self.handsim.setCommand(u)
        elif c == 'n':
            self.handsim.ext_forces[0] = not self.handsim.ext_forces[0]
        elif c == 'm':
            self.handsim.ext_forces[1] = not self.handsim.ext_forces[1]
        elif c == ',':
            self.handsim.ext_forces[2] = not self.handsim.ext_forces[2]
        elif c == 'c':
            self.drawContacts = not self.drawContacts
            if self.drawContacts:
                self.sim.enableContactFeedbackAll()

        glutPostRedisplay()


if __name__=='__main__':
    world = WorldModel()
    import sys
    if len(sys.argv) > 1:
        world_file = sys.argv[1]
    else:
        import os
        klampt_model_name_abs = os.path.normpath(
            os.path.join(os.path.dirname(__file__),
                         klampt_model_name))
        world_file = klampt_model_name_abs

    if not world.readFile(world_file):
        print "Could not load Reflex hand from", world_file
        exit(1)
    sim = Simulator(world)

    if has_moving_base:
        handsim = HandSim(sim, world, 0, 6, 6)
    else:
        handsim = HandSim(sim, world)

    viewer = HandSimGLViewer(handsim, world_name=world_file)
    viewer.run()

    
