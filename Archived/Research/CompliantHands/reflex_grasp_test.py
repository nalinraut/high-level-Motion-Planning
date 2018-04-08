from reflex_col import *
import numpy

has_moving_base = True
klampt_model_name = 'data/reflex_col_with_moving_base.rob'

AUTOMATIC_MODE = True

DISABLE_BLEM = False

# for all experiments, use low effort. For high effort experiments, scale that value by 3/2 (as in physical exps)
THRESHOLD_EFFORT = 2.3
HIGH_EFFORT = THRESHOLD_EFFORT*3.0/2.0 # if effort was not .2, it was .3
high_effort_experiments = ["heinz_P1", "heinz_P2", "heinz_P3", "hammer_P1", "hammer_P2", "hammer_P3"]

# by default, do not perturb the x-y position of the object
DEFAULT_POSE_PERT = 0.0
#DEFAULT_POSE_PERT = 0.01

# by default, move the hand by 0.05m in a random direction. If the experiment is in the custom_pert dict, use that value
DEFAULT_SHAKING_PERT = 0.05
custom_pert = {}


class ReflexGraspSimViewer(HandSimGLViewer):
    def __init__(self, world, log_prefix = None):
        if has_moving_base:
            HandSimGLViewer.__init__(self,world,6,6)
        else:
            HandSimGLViewer.__init__(self,world)
        if DISABLE_BLEM:
            sim.setSetting("boundaryLayerCollisions","0")
        
     	#defined in GLSimulationProgram
        self.simulate = AUTOMATIC_MODE
        self.log_prefix = log_prefix
        self.sim.log_state_fn = log_prefix+'.log'
        self.sim.log_contacts_fn = log_prefix+'_contacts.log'

        self.auto_close = AUTOMATIC_MODE
        self.auto_close_idle_duration = 0.5
        self.auto_close_idle = None

        self.stats = {}

        self.shaking = False
        self.num_shakes = 4 # 4 shakes, one every 0.5 secs, for 1.2 secs total
        self.shaking_interval = 0.3
        self.next_shake = None
        self.remaining_shakes = None
        self.dropping = None
        self.dropping_timeout = None

        self.lifting = False
        self.lifting_duration = 3.0 + self.auto_close_idle_duration # wait for auto_close_idle before lifting
        self.lifting_timeout = None

        self.settling_time = 1.0
        self.auto_start_timeout = None

        self.threshold_effort = THRESHOLD_EFFORT

        self.object = None
        self.initial_obj_com = None
        if world.numRigidObjects() > 0:
            self.object = world.rigidObject(0)
            self.initial_obj_com = self.getObjectGlobalCom()
        self.drivers = [world.robot(0).driver(i) for i in range(world.robot(0).numDrivers())]
        self.controller = self.sim.controller(0)

        self.base_setpoint = 6*[0.0]

    def getObjectGlobalCom(self):
        return se3.apply(self.object.getTransform(), self.object.getMass().getCom())

    def shakeObj(self):
        print "shaking object"
        shake_pert = DEFAULT_SHAKING_PERT
        
        random_dist = (numpy.random.random_sample(3) - 0.5)
        random_dist /= numpy.linalg.norm(random_dist)
        random_dist *= shake_pert

        for i in xrange(3):
            self.base_setpoint[i] += random_dist[i]

    def saveExperimentStatistics(self):
        if self.log_prefix is not None:
            import pickle
            fn = self.log_prefix+'.pickle'
            f = file(fn,"w+")
            pickle.dump(self.stats, f)

    def checkObjectIsGrasped(self):
        if self.initial_obj_com is not None:
            return self.getObjectGlobalCom()[2] > self.initial_obj_com[2]+0.02
        return False

    def control_loop(self):
        global AUTOMATIC_MODE

        #external control loop
        #print "Time",self.sim.getTime()

        tau_proximal = [t for i,t in enumerate(self.handsim.sim.getActualTorques(0)) if self.drivers[i].getAffectedLink() in self.handsim.model.proximal_links]
        tau_distal = [t for i,t in enumerate(self.handsim.sim.getActualTorques(0)) if self.drivers[i].getAffectedLink() in self.handsim.model.distal_links]
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
            if all((0.75*abs(tau_p) + abs(tau_d)*2.0< self.threshold_effort) for tau_p,tau_d in zip(tau_proximal,tau_distal)):
                u = self.handsim.getCommand()
                u[0] -= 0.01
                u[1] -= 0.01
                u[2] -= 0.01
                self.handsim.setCommand(u)
                if u[0] < 0.1:
                    self.auto_close = False
                    print "Automatic Closing:",self.auto_close

                    if AUTOMATIC_MODE:
                        self.lifting = True

            else:
                self.auto_close = False
                print "Automatic Closing:",self.auto_close

                if AUTOMATIC_MODE:
                    self.lifting = True


        if has_moving_base:
            q_rob = self.handsim.model.robot.getConfig()

            for i in xrange(6):
                q_rob[i] = self.base_setpoint[i]

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
                    self.base_setpoint[i] = base_up[i]*(self.ttotal-self.auto_close_idle)/(self.lifting_duration-self.auto_close_idle_duration)

            if self.lifting_timeout <= self.ttotal:
                self.lifting_timeout = None
                self.lifting = False
                print "Lifting:", self.lifting

                if AUTOMATIC_MODE:
                    print "Object grasped:", self.checkObjectIsGrasped()
                    self.stats["Object grasped and lifted"] = self.checkObjectIsGrasped()
                    self.saveExperimentStatistics()
                    self.shaking = True
                    print "Stats:",self.stats
                    if not self.stats["Object grasped and lifted"]:
                        exit(0)

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

                self.stats["Grasped After Shaking"] = self.checkObjectIsGrasped()
                self.saveExperimentStatistics()
                if not self.stats["Grasped After Shaking"]:
                    print "Stats:",self.stats
                    exit(0)
                self.dropping = True
        if self.dropping:
            self.handsim.setCommand([1,1,1,0])
            if self.dropping_timeout == None:
                print "Dropping"
                self.dropping_timeout = self.ttotal + 2.0
            if self.ttotal > self.dropping_timeout:
                self.stats["Dropped After Opening"] = not self.checkObjectIsGrasped()
                self.saveExperimentStatistics()
                print "Stats:",self.stats
                sys.exit(0)
        
        if has_moving_base and self.base_setpoint != None:
            qcmd = self.controller.getCommandedConfig()
            vcmd = [0.0]*len(qcmd)
            for i in xrange(5):
                qcmd[i] = self.base_setpoint[i]
            self.controller.setPIDCommand(qcmd,vcmd)
        return


    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c=='a':
            self.auto_close = True
            print "Automatic Closing:", self.auto_close
            print "Threshold effort:",  self.threshold_effort
        elif c=='d' and has_moving_base:
            self.lifting = True
            print "Lifting:", self.lifting
        elif c=='w':
            self.shaking = not self.shaking
            print "Shaking:", self.shaking
        else:
            HandSimGLViewer.keyboardfunc(self,c,x,y)
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
    if DEFAULT_POSE_PERT > 0:
        print "Perturbing object by +/-",DEFAULT_POSE_PERT*0.5
        obj = world.rigidObject(0)
        R,t = obj.getTransform()
        obj.setTransform(R,vectorops.add(t,[random.uniform(-DEFAULT_POSE_PERT*0.5,DEFAULT_POSE_PERT*0.5),random.uniform(-DEFAULT_POSE_PERT*0.5,DEFAULT_POSE_PERT*0.5),0]))

    if DISABLE_BLEM:
        log_prefix = world_file.split(".")[0]+'_opcode'
    else:
        log_prefix = world_file.split(".")[0]

    viewer = ReflexGraspSimViewer(world, log_prefix=log_prefix)
    if any(world_file.split(".")[0].endswith(v) for v in high_effort_experiments):
        viewer.threshold_effort = HIGH_EFFORT
    viewer.run()

    


