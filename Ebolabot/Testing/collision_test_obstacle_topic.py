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


#from Common.system_config import EbolabotSystemConfig

#system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

def inCollision(collider, robot, q):
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


class ObstacleCSpace(CSpace):
    def __init__(self, collider, robot):
        CSpace.__init__(self)
        qmin,qmax = robot.getJointLimits()
        self.bound = []
        for i in range(len(qmin)):
            self.bound.append((qmin[i],qmax[i]))
        self.eps = 1e-3 
        self.collider = collider
        self.robot = robot

    def feasible(self, q):
        #print "checking feasible"
        isfeasible = not inCollision(self.collider, self.robot, q) and not self.robot.selfCollides()
        print CSpace.feasible(self,q), q
        if not CSpace.feasible(self,q): return False
        return not inCollision(self.collider, self.robot, q) and not self.robot.selfCollides()



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
        for v in V:
            self.robot.setConfig(v)
        #    positions.append(self.robot.link("left_gripper:base").getTransform()[1])
            #print "endEffector:", self.robot.getLink("left_gripper:base").getTransform()[1]
            glVertex3f(*self.robot.getLink("left_gripper:base").getTransform()[1])
        #for (i,j) in E:
        #    glVertex3f(*positions[i])
        #    glVertex3f(*positions[j])
        glEnd()
        glEnable(GL_LIGHTING)

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
            print "Joint limits:", self.robot.getJointLimits()
            start = self.world.robot(0).getConfig()
            print "end effector original position:", self.robot.link('left_gripper:base').getWorldPosition((0,0,0))
            print "start", start
            target = self.getClosestConfig(self.world.robot(0), (0.8,0.1,1), 100, 0.1, 100)
            if target==None:
                print "Cannot solve IK"
                return
            self.robot.setConfig(target)
            print "ik result:", target
            print "end effector position:", self.robot.link('left_gripper:base').getWorldPosition((0,0,0))
            path = self.getCollisionFreePath(start, target, 10)
            print "path:", path
            #q = path[1]
            #robot = motion.robot
            #robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
            #robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
            #     #print
            #     #print
            #     #print "Moving",q0,"->",q
            #     #print
            #     #print
            #qlg = robot.left_gripper.configFromKlampt(q)
            #     #print "q is:", q
            #qrg = robot.right_gripper.configFromKlampt(q)
            #     #print qrg
            #if qlg:
            #    robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
            #if qrg:

            #    robot.right_gripper.command(qrg,[1]*len(qrg),[1]*len(qrg))

            for q in path:
                robot = motion.robot
                robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
                robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
                #print
                #print
                #print "Moving",q0,"->",q
                #print
                #print
                qlg = robot.left_gripper.configFromKlampt(q)
                #print "q is:", q
                qrg = robot.right_gripper.configFromKlampt(q)
                #print qrg
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
            #data_out = pc2.read_points(data)
            #print data.width, data.height
            pc = PointCloud()
            pc.propertyNames.append('rgba')
            t0 = time.time()
            points = list(data_out)
            #print points
            #sys.exit(0)
            #print len(points)
            #print "converted to list in time", time.time()-t0
            q = (0.564775, 0.425383, 0.426796, 0.563848)
            R1 = so3.from_quaternion(q)
            ##t1 = (0.24539, 0.0893425, 1.00112145)
            #t1 = (0.24539, -0.05, 1.00112145)
            ####t1 = (0.273078, 0.0893398, 1.0-0.00182539)
            t1 = (0.283078, 0.043398, 1.01-0.00182539)
            #filtered = self.radiusfilter(points,0.1,0)
            (Rinv, tinv) = se3.inv((R1,t1))
            #for p in points:
            #    print p

                #tm = se3.apply((R1,t1), p)
                #t = se3.apply((Rinv, tinv), tm)
                #print t

            #self.xmin = 100
            #self.xmax = 0
            #self.ymin = 100
            #self.ymax = 0
           
            denoised = self.voxelgridfilter(points, 0.1, 0.1, 0.1, 50)
            print "Noise filtered in time",time.time()-t0
            #filtered = points
            filtered = denoised
            #print self.xmin, self.xmax, self.ymin, self.ymax

            #filtered = self.robotSelfFilter(denoised, Rinv, tinv)
            #print len(denoised)-len(filtered)
            print "Robot arms filtered in time",time.time()-t0
            #print len(filtered)
            

            
            counting = 0
            for p in filtered:
                if p == None:
                    counting = counting+1
                    continue
                coordinates = (p[0],p[1],p[2])
                #if vectorops.norm(coordinates)>0.31:
                index = pc.addPoint(coordinates)
                i = struct.unpack("i",struct.pack("f",p[3]))[0]
                pc.setProperty(index, 0, i)
            
            print "^^^^^^^^", counting
            print "Point cloud copied to Klampt format in time",time.time()-t0
            
            #q = (0.566222,0.423455,0.424871,0.5653)
            #q = (0.564775, 0.425383, 0.426796, 0.563848)
            #R1 = so3.from_quaternion(q)
            #t1 = (0.228665,0.0591513,0.0977748)
            #t1 = (0.24539, 0.0893425, 0.00112145)
            #t1 = (0.24539, 0.0893425, 1.00112145)
            #T1 = (R1, t1)
            #R2 = so3.rotation((0,0,1),math.pi/2)
            #t2 = (0,0,1)
            #T2 = (R2, t2)
            #T = se3.mul(T2,T1)
            #(R,t)=T
            #pc.transform(R,t)
            pc.transform(R1, t1)
            print "Point cloud transformed in time",time.time()-t0
            self.newPC = pc
            
        else:

            print "Not ready yet to receive new point cloud..."

    # Color data is lost
    def statisticalfilter(self,points,k,mul):
            pclCloud = pcl.PointCloud()
            pclCloud.from_list(points)
            fil = pclCloud.make_statistical_outlier_filter()
            fil.set_mean_k(k)
           
            fil.set_std_dev_mul_thresh(mul)
            filtered = fil.filter().to_list()
            return filtered
    
    def radiusfilter(self,points,radius,num):
        filtered = []
        coordinates = []
        for p in points:
            coordinates.append([p[0],p[1],p[2]])
        #print np.array(coordinates).reshape(-len(coordinates),3)
        tree = KDTree(np.array(coordinates))
        #tree = KDTree(np.random.random((10, 3)), leaf_size=2) 
        for i,p in enumerate(coordinates):
            #print tree.query_radius(p,r=radius,count_only=True)
            if tree.query_radius(coordinates[i],r=radius,count_only=True)>=num:
                filtered.append(points[i])
        return points

    def voxelgridfilter(self,points,gridX,gridY,gridZ,num):
        #filtered = []
        dictionary = {}
        for i, p in enumerate(points):
            
            x = p[0]
            y = p[1]
            z = p[2]

            #xangle = math.degrees(math.atan2(x,z))+35.3
            #yangle = math.degrees(math.atan2(y,z)) + 30
            #if xangle>self.xmax:
            # 	self.xmax = xangle
            #if xangle<self.xmin:
            #	self.xmin = xangle
            #if yangle>self.ymax:
            #	self.ymax = yangle
            #if yangle<self.ymin:
            #	self.ymin = yangle


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
            #print value
            if len(value)<num:
                for j in value:
                    points[j] = None
                    #print points[j]
        return points

    def robotSelfFilter(self, points, Rinv, tinv):
        #t1 = time.time()
        #filtered = []
        robot = self.world.robot(0)
        #camPos = (0.24539, 0.0893425, 0.00112145) # Not needed anymore in camera frame
        arm_link_names = self.left_arm_link_names+self.right_arm_link_names
        bbMargin = 0.1
        upperThreshold = 0.1 # Need to tune
        lowerThreshold = -0.1 # Need to tune
        armBBs = []
        transformedG = []
        #testPoint = (0.26839444608864566, -0.6676082722078356, 1.2409759988857147)
        #testPoint = (0.6351629925287235, -0.8301655827368236, 1.2909760000026105)
        #print se3.apply((Rinv, tinv), testPoint)
        maxArmDepth = 0
        for i in arm_link_names:
            #robot.link(i).appearance().setColor(0,0,1,1)
            currentT = robot.link(i).getTransform()
            #pt = se3.apply(currentT, (0,0,0.1))
            #print se3.apply((Rinv, tinv), pt)
            #print currentT
            (newR, newt) = se3.mul((Rinv, tinv), currentT)
            #print se3.apply((newR,newt), (0,0,0.1))
            g = robot.link(i).geometry()
            g.setCurrentTransform(newR, newt)
            #g.transform(Rinv, tinv)
            transformedG.append(g)
            myBB = g.getBB()
            if myBB[1][2] > maxArmDepth:
            	maxArmDepth = myBB[1][2]
            expandedBB = ((myBB[0][0]-bbMargin, myBB[0][1]-bbMargin, myBB[0][2]-bbMargin), (myBB[1][0]+bbMargin, myBB[1][1]+bbMargin, myBB[1][2]+bbMargin))
            armBBs.append(expandedBB)
        #robot.link(arm_link_names[10]).appearance().setColor(0,0,1,1)
        #print armBBs[10]   
        #print "Arm geometry transformed in time: ",time.time()-t1
        allIndices = self.indicesOfInterest(armBBs)

        #for i, p in enumerate(points):
        #print points[15000]
        count = 0
        self.count2 = 0
        for j, indices in enumerate(allIndices):
            #print indices[len(indices)-1]
            
            if indices == None:
                continue
            #print len(indices)
            for i in indices:
                
                
                
                #print points[i][3]
            	#if points[i] !=None:
            		
            	#	newpoint = (points[i][0], points[i][1], points[i][2], -1.7014118346e+38)
            		
            	#	points[i] = newpoint

            	#continue


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
            
                point = (x,y,z)
                #discard = False
                #for j,bb in enumerate(armBBs):
                #if(z<1.2):
                
                #g = robot.link(j).geometry()
                #g.transform(Rinv, tinv)    # geometry transformed to camera frame
                #if(self.insideGeometryBB(point,bb,bbMargin)):
                #    if(self.insideGeometry(point,transformedG[j],upperThreshold,lowerThreshold)):
                #        discard = True
                        #break
                    #pass
                #if discard:
                #    points[i] = None
                #print "**********************"
           #     count2 = count2+1
                if(z<1.2) and self.insideGeometry(point,transformedG[j],upperThreshold,lowerThreshold):
                    #print "detected"
                    count = count + 1
                    points[i] = None
        print "detected: ", count
        print "hits: ", self.count2            
        return points
    
    def insideGeometryBB(self, point, bb, bbMargin):
        #t1 = time.time()
       
        mins = vectorops.sub(bb[0],bbMargin)
        maxes = vectorops.sub(bb[1], -bbMargin)
        #print "bounding box checked first two lines in time: ",time.time()-t1
        insideX = point[0]>=mins[0] and point[0]<=maxes[0]
        insideY = point[1]>=mins[1] and point[1]<=maxes[1]
        insideZ = point[2]>=mins[2] and point[2]<=maxes[2]
        #print "bounding box checked in time: ",time.time()-t1
        #print maxes[0], maxes[1]
        #print "point:", point[0], point[1]
        return insideX and insideY and insideZ
 

    def insideGeometry(self, point, g, upperThreshold, lowerThreshold):
        #t1 = time.time()
       #print "using rayCast"
        direction = vectorops.unit(point)
        #print g.getBB()[0][0]
        (hit, pt) = g.rayCast((0,0,0), direction)
        depthPoint = vectorops.norm(point)
        depthHit = vectorops.norm(pt)
        diff = depthPoint - depthHit
        #print "rayCast done in: ",time.time()-t1
        #self.count2 = self.count2 + 1
        if hit:
            #print "hit!!!!!!"
            self.count2 = self.count2 + 1
            if diff < upperThreshold and diff > lowerThreshold: # If point lies inside robot body
                return True
        return False

    def indicesOfInterest(self, armBBs):
        allIndices = []
        for i,bb in enumerate(armBBs):
            #if i<len(self.left_arm_link_names):
            #	print self.left_arm_link_names[i]
            xs = (bb[0][0], bb[1][0])
            ys = (bb[0][1], bb[1][1])
            zs = (bb[0][2], bb[1][2])
            allIndices.append(self.indicesOfSingleBB(xs,ys,zs))
        #print allIndices
        return allIndices

    def indicesOfSingleBB(self, xs, ys, zs):
        #print xs, ys, zs
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
        #print "*********x: ", xminAngle, xmaxAngle
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
        #print "*********y: ", yminAngle, ymaxAngle
        #print "*********y coordiantes: ", ys
        if yminAngle > 60 or ymaxAngle < 0:
           
            return None
        if yminAngle < 0:
            yminAngle = 0
        if ymaxAngle > 60:
            ymaxAngle = 60

        #print "*********",xminAngle, xmaxAngle, yminAngle, ymaxAngle
        #print "******* lalalala"
        xminPixel = int (math.floor(512*xminAngle/70.6))
        xmaxPixel = int (math.floor(512*xmaxAngle/70.6))
        yminPixel = int (math.floor(424*yminAngle/60))
        ymaxPixel = int (math.floor(424*ymaxAngle/60))
        #print "*********", xminPixel, xmaxPixel, yminPixel, ymaxPixel
        for i in range(yminPixel, ymaxPixel):
            for j in range(xminPixel, xmaxPixel):
                indices.append(512*i+j)
        #print "******** indices: ", indices[len(indices)-1]
        return indices


            

    #def countNeighbors(self, points, radius):
    #    dictionary = defaultdict(int)
    #    for i in range(len(points)-1):
    #        for j in range(i+1,len(points)):
    #            pointA = points[i]
    #            pointB = points[j]
    #            if(vectorops.distance((pointA[0],pointA[1],pointA[2]),(pointB[0],pointB[1],pointB[2]))<radius):
    #                dictionary[i]+=1
    #                dictionary[j]+=1
    #    return dictionary


    def idle(self):
        if self.newPC != None:
            print "Setting the terrain geometry"
            self.world.terrain(0).geometry().setPointCloud(self.newPC)
            print "Setting the collision margin"
            self.world.terrain(0).geometry().setCollisionMargin(0.01)
            self.newPC = None
            print "Done"
            self.refresh()

    
    #def calibrate(self, point):
    #    q = (0.566222,0.423455,0.424871,0.5653)
    #    R = so3.from_quaternion(q)
    #    t = (0.228665,0.0591513,0.0977748)
    #    T = (R, t)
    #    newPoint = se3.apply(T,point)
    #    x = newPoint[0]
    #    y = newPoint[1]
    #    z = newPoint[2]
    #    return (-y,x,z)
    def getCollisionFreePath(self, start, goal, iterations):
        #MotionPlan.setOptions(type="rrt", perturbationRadius=2.0, bidirectional=True)
        #MotionPlan.setOptions(type="prm", knn=10, connectionThreshold=0.25)
        MotionPlan.setOptions(type="sbl", perturbationRadius=2.0, connectionThreshold=0.5, bidirectional=True)
        #MotionPlan.setOptions(type="lazyrrg*")
        #space = ObstacleCSpace(self.collider, self.robot)
        #planner = MotionPlan(space)
        
        #planner = robotplanning.planToConfig(self.world, self.robot, goal, type="prm", knn=10, connectionThreshold=0.1)
        print "milestone 1"
        space = robotcspace.RobotCSpace(self.robot, WorldCollider(self.world))
        jointLimits = self.robot.getJointLimits()
        lower = jointLimits[0]
        higher = jointLimits[1]
        for i in range(12):
            lower[i]=0
            higher[i]=0
        newLimits = (lower,higher)
        space.bound = zip(*newLimits)
        print "milestone 2"
        planner = cspace.MotionPlan(space)
        print "milestone 3"
        planner.setEndpoints(start, goal)
        print "before planning"
        planner.planMore(iterations)
        print "after planning"
        V,E = planner.getRoadmap()
        self.roadMap = (V,E)
        print "No. of vertices:", len(V)
        print "Vertices:",V
        print "Edges:", E
       
        return planner.getPath()

  


   

    def perturb(self, q, radius):
        newq = []
        for i in q:
            newq.append(i+random.uniform(-radius, radius))
        return newq

    def getClosestConfig(self, robot, target, iterations, c, numsteps):
        cost = 9999
        res = None
        start = robot.getConfig()
        #goal = ik.objective(robot.link("left_gripper:base"), local = [(0,0,0)], world = [target])
        
        #goal = ik.objective(robot.link("left_wrist"), local = [(0,0,0)], world = [target])
        s = IKSolver(robot)
        objective = IKObjective()
        objective.setFixedPoint(robot.link("left_gripper:base").getIndex(), (0,0,0), target)
        s.add(objective)
        s.setActiveDofs([12, 13, 14, 15, 16, 18, 19])
        print "Active DOFs:", s.getActiveDofs()
        for i in range(iterations):
            #if ik.solve(goal):
            (ret, iters) = s.solve(100,1e-4)
            if ret:
                end = robot.getConfig()
                print "*****************************",end
                qmin, qmax = robot.getJointLimits()
                #print qmin
                #print qmax
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
                    print "u is:", u
                    q = robot.interpolate(end, start, u)
                    #q = end
                    print "interpolated q is:", q
                    if not inCollision(WorldCollider(self.world), robot, q):
                        newcost = vectorops.distance(q, end) + c*vectorops.distance(q, start) 
                        if newcost < cost:
                            res = q
                            cost = newcost
                        break
            start = self.perturb(start, 0.1)
            robot.setConfig(start)
        print "res is:", res
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
