from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt import vectorops
from klampt.glprogram import *
from klampt.glcommon import GLWidgetPlugin
from klampt.robotcollide import WorldCollider
from klampt.robotsim import Geometry3D
from klampt.robotsim import PointCloud
from klampt.robotsim import IKSolver
from klampt.robotsim import IKObjective
import math
import time
import os
import sys
import random
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from roslib import message
import struct
from collections import defaultdict
#import pcl
#from sklearn.neighbors import KDTree
import numpy as np
from klampt import ik
from klampt import robotcspace
from klampt import cspace
from klampt.cspace import CSpace, MotionPlan
from klampt import robotplanning




def inCollision(collider, robot, q):
    """checks if a configuration collides with the environment"""
    robot.setConfig(q)
    collisions = collider.robotTerrainCollisions(robot.index)
    if any(collisions):
        return True
    return False


def selfcollision(robot,qstart,qgoal):
    """returns whether a self collision is predicted along the route from qstart to qgoal"""
    #return False
    distance = vectorops.distance(qstart,qgoal)
    epsilon = math.radians(2)
    numsteps = int(math.ceil(distance/epsilon))
    for i in xrange(numsteps+1):
        u = float(i)/numsteps
        q = robot.interpolate(qstart,qgoal,u)
        robot.setConfig(q)
        if robot.selfCollides():
            print "******************* SELF COLLIDES"
            return True
    return False

def obstaclecollision(collider,robot,qstart,qgoal):
    """returns whether a obstacle collision is predicted along the route from qstart to qgoal"""
    #return None
    distance = vectorops.distance(qstart,qgoal)
    epsilon = math.radians(2)
    numsteps = int(math.ceil(distance/epsilon))
    print "numTerrains:", collider.world.numTerrains()
    for i in xrange(numsteps+1):
        u = float(i)/numsteps
        q = robot.interpolate(qstart,qgoal,u)
        robot.setConfig(q)
        collisions = collider.robotTerrainCollisions(robot.index)
        if any(collisions):
            return collisions
    return None


# class ObstacleCSpace(CSpace):
#     def __init__(self, collider, robot):
#         CSpace.__init__(self)
#         qmin,qmax = robot.getJointLimits()
#         self.bound = []
#         for i in range(len(qmin)):
#             self.bound.append((qmin[i],qmax[i]))
#         self.eps = 1e-3 
#         self.collider = collider
#         self.robot = robot

#     def feasible(self, q):
#         #print "checking feasible"
#         isfeasible = not inCollision(self.collider, self.robot, q) and not self.robot.selfCollides()
#         print CSpace.feasible(self,q), q
#         if not CSpace.feasible(self,q): return False
#         return not inCollision(self.collider, self.robot, q) and not self.robot.selfCollides()



class GLWidgetProgram(GLPluginProgram):
    """A program that uses a widget plugin"""
    def __init__(self,name):
        GLPluginProgram.__init__(self,"Manual poser")
        self.widgetPlugin = GLWidgetPlugin()
    def initialize(self):
        GLPluginProgram.initialize(self)
        self.setPlugin(self.widgetPlugin)
    def addWidget(self,widget):
        self.widgetPlugin.addWidget(widget)

class MyGLViewer(GLWidgetProgram):
    def __init__(self,world):
        GLWidgetProgram.__init__(self,"Manual poser")

        #start up the poser in the currently commanded configuration
        q = motion.robot.getKlamptCommandedPosition()
        world.robot(0).setConfig(q)

        self.world = world
        self.robot = world.robot(0)
        self.robotPoser = RobotPoser(world.robot(0))
        self.addWidget(self.robotPoser)
        self.roadMap=([],[])

        robot = world.robot(0)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        self.left_arm_link_names = left_arm_link_names
        self.right_arm_link_names = right_arm_link_names
        missing_left_arm_names = ['left_upper_forearm_visual', 'left_upper_elbow_visual']
        missing_right_arm_names = ['right_upper_forearm_visual', 'right_upper_elbow_visual']
        #self.left_arm_link_names+=missing_left_arm_names
        #self.right_arm_link_names+=missing_right_arm_names
        self.left_arm_link_names=['left_upper_forearm','left_lower_forearm','left_wrist', 'left_upper_forearm_visual', 'left_gripper:base']
        self.right_arm_link_names=['right_upper_forearm','right_lower_forearm','right_wrist', 'right_upper_forearm_visual', 'right_gripper:base']
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]
        self.firstTimeHack = True
        self.world.makeTerrain('terrain')
       
        self.subscribe()
        self.newPC = None

        self.collider = WorldCollider(self.world)

    def display(self):
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green

        #this line with draw the world
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL()
        self.world.terrain(0).drawGL()
        GLWidgetProgram.display(self)

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL(False)
        glDisable(GL_BLEND)
        V,E = self.roadMap
        positions = []
        glDisable(GL_LIGHTING)
        glColor3f(0,1,0)
        glLineWidth(5.0)
        glBegin(GL_LINE_STRIP)
        #print len(V)
        #P = []
        #for j in range(len(V)/2):
        #    P.append(V[j])
        #for v in V:
        #    self.robot.setConfig(v)
        #    positions.append(self.robot.link("left_gripper:base").getTransform()[1])
            #print "endEffector:", self.robot.getLink("left_gripper:base").getTransform()[1]
        #    glVertex3f(*self.robot.getLink("left_gripper:base").getTransform()[1])
        #for (i,j) in E:
        #    glVertex3f(*positions[i])
        #    glVertex3f(*positions[j])
        #glEnd()
        #glEnable(GL_LIGHTING)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print '[space]: send the current posed milestone'
            print 'q: clean quit'
        elif c == ' ':
            q = self.robotPoser.get()
            #print "space q:", q
            q0 = motion.robot.getKlamptCommandedPosition()
            #print q0
            #collider = WorldCollider(self.world)
          
            collisions = obstaclecollision(WorldCollider(self.world),self.world.robot(0),q0,q)
          
            for i in range(self.world.robot(0).numLinks()):
                self.world.robot(0).link(i).appearance().setColor(0.5,0.5,0.5,1)
            if not self.firstTimeHack and selfcollision(self.world.robot(0),q0,q):
                print "Invalid configuration, it self-collides"
            
            elif not self.firstTimeHack and collisions!=None:
                #clear all links to gray
                for pairs in collisions:
                    print "Link "+str(pairs[0].getIndex())+" collides with obstacle"
                    self.world.robot(0).link(pairs[0].getIndex()).appearance().setColor(1,0,0,1)
            else:
                self.firstTimeHack = False
                robot = motion.robot
                robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
                robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
                print
                print
                print "Moving",q0,"->",q
                print
                print
                qlg = robot.left_gripper.configFromKlampt(q)
                qrg = robot.right_gripper.configFromKlampt(q)
                print "space prg:", qrg
                robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
                robot.right_gripper.command(qrg,[1]*len(qrg),[1]*len(qrg))
                #robot.left_mq.setRamp([q[i] for i in self.left_arm_link_indices])
                #robot.right_mq.setRamp([q[i] for i in self.right_arm_link_indices])
        elif c == 'q':
            motion.robot.shutdown()
            exit(0)
        elif c == 'p':
            """given a target object position, automatically moves the end effector close to the target object""" 
            #print "Joint limits:", self.robot.getJointLimits()
            start = self.world.robot(0).getConfig()
            #print "end effector original position:", self.robot.link('left_gripper:base').getWorldPosition((0,0,0))
            #print "start", start
            """target position currently fixed at (0.8,0.1,1) for testing"""
            target = self.getClosestConfig(self.world.robot(0), (0.8,0.1,1), 100, 0.1, 100)
            if target==None:
            #    print "Cannot solve IK"
                return
            self.robot.setConfig(target)
            #print "ik result:", target
            #print "end effector position:", self.robot.link('left_gripper:base').getWorldPosition((0,0,0))
            path = self.getCollisionFreePath(start, target, 10)
            #print "path:", path
           

            for q in path:
                robot = motion.robot
                robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
                robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
               
                qlg = robot.left_gripper.configFromKlampt(q)
                
                qrg = robot.right_gripper.configFromKlampt(q)
                
                if qlg:
                    robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
                if qrg:

                    robot.right_gripper.command(qrg,[1]*len(qrg),[1]*len(qrg))
            print "getKlamptCommandedPosition:", robot.getKlamptCommandedPosition()

        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()
    
    def subscribe(self):
        rospy.init_node('subscribe', anonymous=True)
        rospy.Subscriber("/kinect2/sd/points", PointCloud2, self.callback)

    def callback(self,data):
        if self.newPC == None:
            data_out = pc2.read_points(data,skip_nans=False)
            pc = PointCloud()
            pc.propertyNames.append('rgba')
            t0 = time.time()
            points = list(data_out)
            """ needs better external calibration"""
            q = (0.564775, 0.425383, 0.426796, 0.563848)
            R1 = so3.from_quaternion(q)
            t1 = (0.283078, 0.043398, 1.01-0.00182539)
            (Rinv, tinv) = se3.inv((R1,t1))
            denoised = self.voxelgridfilter(points, 0.1, 0.1, 0.1, 50)
            print "Noise filtered in time",time.time()-t0
            filtered = denoised
            print "Robot arms filtered in time",time.time()-t0
            counting = 0
            for p in filtered:
                if p == None:
                    counting = counting+1
                    continue
                coordinates = (p[0],p[1],p[2])
                index = pc.addPoint(coordinates)
                i = struct.unpack("i",struct.pack("f",p[3]))[0]
                pc.setProperty(index, 0, i)
            
            print "^^^^^^^^", counting
            print "Point cloud copied to Klampt format in time",time.time()-t0
            
            pc.transform(R1, t1)
            print "Point cloud transformed in time",time.time()-t0
            self.newPC = pc
        else:
            print "Not ready yet to receive new point cloud..."

    
    # def statisticalfilter(self,points,k,mul):
    #         pclCloud = pcl.PointCloud()
    #         pclCloud.from_list(points)
    #         fil = pclCloud.make_statistical_outlier_filter()
    #         fil.set_mean_k(k)
           
    #         fil.set_std_dev_mul_thresh(mul)
    #         filtered = fil.filter().to_list()
    #         return filtered
    
    # def radiusfilter(self,points,radius,num):
    #     filtered = []
    #     coordinates = []
    #     for p in points:
    #         coordinates.append([p[0],p[1],p[2]])
       
    #     tree = KDTree(np.array(coordinates))
        
    #     for i,p in enumerate(coordinates):
            
    #         if tree.query_radius(coordinates[i],r=radius,count_only=True)>=num:
    #             filtered.append(points[i])
    #     return points

    def voxelgridfilter(self,points,gridX,gridY,gridZ,num):
        """ a simple voxelgrid filter to get rid of noise in point cloud data"""
        dictionary = {}
        for i, p in enumerate(points):
            x = p[0]
            y = p[1]
            z = p[2]

            if math.isnan(x) or vectorops.norm((x,y,z))<0.31:
                points[i] = None
                continue
            xindex = int (math.floor(x/gridX))
            yindex = int (math.floor(y/gridY))
            zindex = int (math.floor(z/gridZ))
            key = (xindex,yindex,zindex)
            if key not in dictionary:
                dictionary[key]=[]
            dictionary[key].append(i)
        for key,value in dictionary.iteritems():
            if len(value)<num:
                for j in value:
                    points[j] = None
        return points

    def robotSelfFilter(self, points, Rinv, tinv):
        """ Get rid of points that belong to the robot's arms"""
        robot = self.world.robot(0)
        arm_link_names = self.left_arm_link_names+self.right_arm_link_names
        bbMargin = 0.1
        #upperThreshold = 0.1 # Need to tune
        #lowerThreshold = -0.1 # Need to tune
        armBBs = []
        transformedG = []
        maxArmDepth = 0
        for i in arm_link_names:
            currentT = robot.link(i).getTransform()
            (newR, newt) = se3.mul((Rinv, tinv), currentT)
            g = robot.link(i).geometry()
            g.setCurrentTransform(newR, newt)
            transformedG.append(g)
            myBB = g.getBB()
            if myBB[1][2] > maxArmDepth:
                maxArmDepth = myBB[1][2]
            expandedBB = ((myBB[0][0]-bbMargin, myBB[0][1]-bbMargin, myBB[0][2]-bbMargin), (myBB[1][0]+bbMargin, myBB[1][1]+bbMargin, myBB[1][2]+bbMargin))
            armBBs.append(expandedBB)
        allIndices = self.indicesOfInterest(armBBs)
        count = 0
        self.count2 = 0
        for j, indices in enumerate(allIndices):
            if indices == None:
                continue
            for i in indices:

                p = points[i]
                if p==None:
                    continue
                x = p[0]
                y = p[1]
                z = p[2]

                if vectorops.norm((x,y,z))>maxArmDepth+0.3:
                    continue
                points[i] = None
                continue
                #point = (x,y,z)
                #if(z<1.2) and self.insideGeometry(point,transformedG[j],upperThreshold,lowerThreshold):
                #    count = count + 1
                #    points[i] = None
        return points
    
    # def insideGeometryBB(self, point, bb, bbMargin):
  
       
    #     mins = vectorops.sub(bb[0],bbMargin)
    #     maxes = vectorops.sub(bb[1], -bbMargin)
      
    #     insideX = point[0]>=mins[0] and point[0]<=maxes[0]
    #     insideY = point[1]>=mins[1] and point[1]<=maxes[1]
    #     insideZ = point[2]>=mins[2] and point[2]<=maxes[2]
       
    #     return insideX and insideY and insideZ
 

    # def insideGeometry(self, point, g, upperThreshold, lowerThreshold):
        
    #     direction = vectorops.unit(point)
        
    #     (hit, pt) = g.rayCast((0,0,0), direction)
    #     depthPoint = vectorops.norm(point)
    #     depthHit = vectorops.norm(pt)
    #     diff = depthPoint - depthHit
       
    #     if hit:
            
    #         self.count2 = self.count2 + 1
    #         if diff < upperThreshold and diff > lowerThreshold: # If point lies inside robot body
    #             return True
    #     return False

    def indicesOfInterest(self, armBBs):
        """ Get indices of point cloud data that belong to robot's arms"""
        allIndices = []
        for i,bb in enumerate(armBBs):
           
            xs = (bb[0][0], bb[1][0])
            ys = (bb[0][1], bb[1][1])
            zs = (bb[0][2], bb[1][2])
            allIndices.append(self.indicesOfSingleBB(xs,ys,zs))
        
        return allIndices

    def indicesOfSingleBB(self, xs, ys, zs):
        """ Get indices of point cloud data that belong to the bounding box of a single arm link"""
        indices = []
        xminAngle = 180
        xmaxAngle = -180
        yminAngle = 180
        ymaxAngle = -180
        for x in xs:
            for z in zs:
                xangle = math.degrees(math.atan2(x,z))+35.3
                if xangle < xminAngle:
                    xminAngle = xangle
                if xangle > xmaxAngle:
                    xmaxAngle = xangle
       
        if xminAngle >70.6 or xmaxAngle < 0:
           
            return None
        if xminAngle < 0:
            xminAngle = 0
        if xmaxAngle > 70.6:
            xmaxAngle = 70.6

        for y in ys:
            for z in zs:
                yangle = math.degrees(math.atan2(y,z)) + 30
                if yangle < yminAngle:
                    yminAngle = yangle
                if yangle > ymaxAngle:
                    ymaxAngle = yangle
        
        if yminAngle > 60 or ymaxAngle < 0:
           
            return None
        if yminAngle < 0:
            yminAngle = 0
        if ymaxAngle > 60:
            ymaxAngle = 60
        
        xminPixel = int (math.floor(512*xminAngle/70.6))
        xmaxPixel = int (math.floor(512*xmaxAngle/70.6))
        yminPixel = int (math.floor(424*yminAngle/60))
        ymaxPixel = int (math.floor(424*ymaxAngle/60))
       
        for i in range(yminPixel, ymaxPixel):
            for j in range(xminPixel, xmaxPixel):
                indices.append(512*i+j)
        
        return indices


    def idle(self):
        if self.newPC != None:
            print "Setting the terrain geometry"
            self.world.terrain(0).geometry().setPointCloud(self.newPC)
            print "Setting the collision margin"
            self.world.terrain(0).geometry().setCollisionMargin(0.01)
            self.newPC = None
            print "Done"
            self.refresh()

    
   
    def getCollisionFreePath(self, start, goal, iterations):
        """ Given a start and a goal configuration, returns a collision-free path between the two configurations"""
        """ Currently takes forever to find a path... Needs more work"""
        #MotionPlan.setOptions(type="rrt", perturbationRadius=2.0, bidirectional=True)
        #MotionPlan.setOptions(type="prm", knn=10, connectionThreshold=0.25)
        MotionPlan.setOptions(type="sbl", perturbationRadius=2.0, connectionThreshold=0.5, bidirectional=True)
        #MotionPlan.setOptions(type="lazyrrg*")
        #space = ObstacleCSpace(self.collider, self.robot)
        #planner = MotionPlan(space)
        
        #planner = robotplanning.planToConfig(self.world, self.robot, goal, type="prm", knn=10, connectionThreshold=0.1)
        
        space = robotcspace.RobotCSpace(self.robot, WorldCollider(self.world))
        jointLimits = self.robot.getJointLimits()
        lower = jointLimits[0]
        higher = jointLimits[1]
        for i in range(12):
            lower[i]=0
            higher[i]=0
        newLimits = (lower,higher)
        space.bound = zip(*newLimits)
       
        planner = cspace.MotionPlan(space)
        
        planner.setEndpoints(start, goal)
        
        planner.planMore(iterations)
        
        V,E = planner.getRoadmap()
        self.roadMap = (V,E)
      
        return planner.getPath()

  


   

    def perturb(self, q, radius):
        newq = []
        for i in q:
            newq.append(i+random.uniform(-radius, radius))
        return newq

    def getClosestConfig(self, robot, target, iterations, c, numsteps):
        """ given a target object position, returns a configuration with end effector close to the target object but without colliding with it"""
        cost = 9999
        res = None
        start = robot.getConfig()
      
        s = IKSolver(robot)
        objective = IKObjective()
        objective.setFixedPoint(robot.link("left_gripper:base").getIndex(), (0,0,0), target)
        s.add(objective)
        s.setActiveDofs([12, 13, 14, 15, 16, 18, 19])
        
        for i in range(iterations):
            (ret, iters) = s.solve(100,1e-4)
            if ret:
                end = robot.getConfig()
                qmin, qmax = robot.getJointLimits()
                flag = False
                for k in range(len(end)):
                    if end[k]<qmin[k] or end[k]>qmax[k]:
                        flag = True
                        break
                if flag:
                    start = self.perturb(start, 0.1)
                    robot.setConfig(start)
                    continue
                for j in xrange(numsteps+1):
                    u = float(j)/numsteps
                    q = robot.interpolate(end, start, u)
                    if not inCollision(WorldCollider(self.world), robot, q):
                        newcost = vectorops.distance(q, end) + c*vectorops.distance(q, start) 
                        if newcost < cost:
                            res = q
                            cost = newcost
                        break
            start = self.perturb(start, 0.1)
            robot.setConfig(start)
        
        return res



if __name__ == "__main__":
    config.parse_args()
    print "widget_control.py: manually sends configurations to the Motion module"
    print "Press [space] to send milestones.  Press q to exit."
    print

    print "Loading APC Motion model",config.klampt_model    
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",server_addr=config.motion_server_addr)
    res = motion.robot.startup()
    if not res:
        print "Error starting up APC Motion"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load APC Motion model "+fn)
    
    

    viewer = MyGLViewer(world)
    viewer.run()

