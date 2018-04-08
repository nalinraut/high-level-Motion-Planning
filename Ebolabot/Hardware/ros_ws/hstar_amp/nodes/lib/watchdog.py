import roslib
roslib.load_manifest( 'hstar_amp' )
import rospy, sys
import tf
import math

# ros common msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#=======================================================#
#   
#=======================================================#
class Watchdog:

  def __init__( self ):
    self.sig = -1
    self.sig_timeout = 0.2
    self.sig_count = 0
    self.sig_count_min = 1
    self.valid = False
    self.valid_last = False
    self.update()

  def setValidCountMin( self, val ):
    self.sig_count_min = val
 
  def isValid( self ):
    #return self.valid 
    return self.check() 

  def setTimeout( self, val ):
    self.sig_timeout = val

  def update( self ):
    self.sig = rospy.Time.now()
      
  def updateWithTime( self, val ):
    self.sig = val 

  #def check( self, val ):
  def check( self ):
    if ( self.sig == -1 ):
      valid = False
    else:
      self.valid_last = self.valid
      dt = ( rospy.Time.now() - self.sig ).to_sec()
      #dt = ( val - self.sig ).to_sec()
      if ( dt < self.sig_timeout ):
        self.sig_count = self.sig_count + 1 
      else:
        self.sig_count = 0
      self.valid = self.sig_count > self.sig_count_min
    return self.valid
