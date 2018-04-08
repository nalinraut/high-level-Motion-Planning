from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt import vectorops
from klampt.glprogram import *
from klampt.glcommon import GLWidgetPlugin
from klampt.robotcollide import WorldCollider
from klampt.robotsim import Geometry3D
from klampt.robotsim import PointCloud
import math
import time
import os
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
import pcl
from sklearn.neighbors import KDTree
import numpy as np

def selfcollision(robot,qstart,qgoal):
    """returns whether a self collision is predicted along the route from qstart to qgoal"""
    distance = vectorops.distance(qstart,qgoal)
    epsilon = math.radians(2)
    numsteps = int(math.ceil(distance/epsilon))
    for i in xrange(numsteps+1):
        u = float(i)/numsteps
        q = robot.interpolate(qstart,qgoal,u)
        robot.setConfig(q)
        if robot.selfCollides():
            return True
    return False

def obstaclecollision(collider,robot,qstart,qgoal):
    """returns whether a obstacle collision is predicted along the route from qstart to qgoal"""
    #t0 = time.time()
    distance = vectorops.distance(qstart,qgoal)
    epsilon = math.radians(2)
    numsteps = int(math.ceil(distance/epsilon))

    for i in xrange(numsteps+1):
        u = float(i)/numsteps
        q = robot.interpolate(qstart,qgoal,u)
        robot.setConfig(q)
        collisions = collider.robotTerrainCollisions(robot.index)
        
        if any(collisions):
            return collisions
    
    
    #print "Obstacle collision detection done in time", + time.time()-t0
    
   
    return None

def bisection(collider,robot,qstart,qgoal):

    distance = vectorops.distance(qstart,qgoal)
    epsilon = math.radians(2)
    numsteps = int(math.ceil(distance/epsilon))
    
    return bisectionhelper(collider,robot,qstart,qgoal,0,numsteps,numsteps)
    

def bisectionhelper(collider,robot,qstart,qgoal,left,right,numsteps):
   
    mid = (left+right)/2
    u = float(mid)/numsteps
    q = robot.interpolate(qstart,qgoal,u)
    robot.setConfig(q)
    collisions = collider.robotTerrainCollisions(robot.index)
    if any(collisions):
        return True
    if left>=right:
        return False

    return bisectionhelper(collider,robot,qstart,qgoal,left,mid-1,numsteps) or bisectionhelper(collider,robot,qstart,qgoal,mid+1,right,numsteps)






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

        self.world = world
        self.robotPoser = RobotPoser(world.robot(0))
        self.addWidget(self.robotPoser)

        robot = world.robot(0)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]
        self.firstTimeHack = True
        self.world.makeTerrain('terrain')
       
        self.subscribe()
        self.newPC = None

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

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print '[space]: send the current posed milestone'
            print 'q: clean quit'
        elif c == ' ':
            q = self.robotPoser.get()
            q0 = motion.robot.getKlamptCommandedPosition()
            t1 = time.time()
            #collisions = obstaclecollision(WorldCollider(self.world),self.world.robot(0),q0,q)
            collides = bisection(WorldCollider(self.world),self.world.robot(0),q0,q)
            print "Obstacle collision detection done in time", + time.time()-t1
            exit(0)
            for i in range(self.world.robot(0).numLinks()):
                self.world.robot(0).link(i).appearance().setColor(0.5,0.5,0.5,1)
            #if not self.firstTimeHack and selfcollision(self.world.robot(0),q0,q):
            if collides:
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
                qlg = robot.left_gripper.configFromKlampt(q)
                qrg = robot.right_gripper.configFromKlampt(q)
                robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
                robot.right_gripper.command(qrg,[1]*len(qrg),[1]*len(qrg))
                #robot.left_mq.setRamp([q[i] for i in self.left_arm_link_indices])
                #robot.right_mq.setRamp([q[i] for i in self.right_arm_link_indices])
        elif c == 'q':
            motion.robot.shutdown()
            exit(0)
        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()
    
    def subscribe(self):
        rospy.init_node('subscribe', anonymous=True)
        rospy.Subscriber("/kinect2/sd/points", PointCloud2, self.callback)

    def callback(self,data):
        if self.newPC == None:
            data_out = pc2.read_points(data,skip_nans=True)
            #data_out = pc2.read_points(data)
            pc = PointCloud()
            pc.propertyNames.append('rgba')
            t0 = time.time()
            points = list(data_out)
          
            #filtered = self.radiusfilter(points,0.1,0)
            filtered = self.voxelgridfilter(points, 0.1, 0.1, 0.1, 10)
            print "Noise cleaned up in time", time.time()-t0
            #filtered = points
            for p in filtered:
                coordinates = (p[0],p[1],p[2])
                if vectorops.norm(coordinates)>0.4:
                    index = pc.addPoint(coordinates)
                    i = struct.unpack("i",struct.pack("f",p[3]))[0]
                    pc.setProperty(index, 0, i)
            #print "Point cloud copied to Klampt format in time",time.time()-t0
            
            q = (0.566222,0.423455,0.424871,0.5653)
            R1 = so3.from_quaternion(q)
            t1 = (0.228665,0.0591513,0.0977748)
            T1 = (R1, t1)
            R2 = so3.rotation((0,0,1),math.pi/2)
            t2 = (0,0,1)
            T2 = (R2, t2)
            T = se3.mul(T2,T1)
            (R,t)=T
            pc.transform(R,t)
            #print "Point cloud transformed in time",time.time()-t0
            self.newPC = pc
        else:

            print "Not ready yet to receive new point cloud..."

    # Color data will be lost
    def statisticalfilter(self,points,k,mul):
            pclCloud = pcl.PointCloud()
            pclCloud.from_list(points)
            fil = pclCloud.make_statistical_outlier_filter()
            fil.set_mean_k(k)
           
            fil.set_std_dev_mul_thresh(mul)
            filtered = fil.filter().to_list()
            return filtered
    # Under construction
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
        filtered = []
        dictionary = {}
        for i, p in enumerate(points):
            x = p[0]
            y = p[1]
            z = p[2]
            xindex = int (math.floor(x/gridX))
            yindex = int (math.floor(y/gridY))
            zindex = int (math.floor(z/gridZ))
            key = (xindex,yindex,zindex)
            if key not in dictionary:
                dictionary[key]=[]
            else:
                dictionary[key].append(i)
        for key,value in dictionary.iteritems():
            if len(value)>=num:
                for j in value:
                    filtered.append(points[j])
        return filtered

    
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
            #print "Setting the terrain geometry"
            self.world.terrain(0).geometry().setPointCloud(self.newPC)
            #print "Setting the collision margin"
            self.world.terrain(0).geometry().setCollisionMargin(0.01)
            self.newPC = None
            #print "Done"
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



if __name__ == "__main__":
    config.parse_args()
    print "widget_control.py: manually sends configurations to the Motion module"
    print "Press [space] to send milestones.  Press q to exit."
    print

    print "Loading APC Motion model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",)
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

