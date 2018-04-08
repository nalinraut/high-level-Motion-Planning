#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    display on serial LCD
            - battery:
              - voltage
              - s.o.c. 
            - ip addr: 
              - change in yaml file

  author:  ks
  created:

---------------------------------------------------------------------------'''

import roslib
import rospy, sys
from std_msgs.msg import String, Float32

import os
import sys

import serial, time, sys, os

NODE_NAME='lcd'

ip = os.getenv( "ROS_IP", 'N/A' )
print 'get ip address: ' + str( ip )

#print "Content-Type: text/plain\n\n"
#for key in os.environ.keys():
    #print "%30s %s \n" % (key,os.environ[key])

from lib.lcd_functions import SerialLcd

#-----------------------------------------------------#
#  
#-----------------------------------------------------#
class ROSNode:

  #  constructor 
  #-----------------------------------------------------#
  def __init__(self):
    rospy.init_node( NODE_NAME )

    # params
    self.port = rospy.get_param( 'serial_port', '/dev/ros/lcd' )
    #self.display_type = rospy.get_param( 'display_type', 'battery' )
    self.display_type = rospy.get_param( '~display_type', 'ip addr' )
    self.dim_timeout = rospy.get_param( '~dim_timeout', 3.0 )
    self.timer_rate = rospy.get_param( '~lcd_refresh_rate', 1.0 ) 
    self.enable_dimming = rospy.get_param( '~enable_dimming', False ) 
    self.init_display_time = rospy.get_param( '~init_display_time', 3.0 ) 
    self.use_rosserial = rospy.get_param( '~use_rosserial', True ) 

    # init lcd
    self.lcd_text = 'ros robot test'
    self.lcd = SerialLcd( self, self.use_rosserial )

    if self.display_type == 'ip addr':
      ip = self.getIp()
      self.lcd.sendText( 'IP: '+ip )

    # sub
    self.sub_lcd = rospy.Subscriber( 'lcd', String, self.lcdCallback)
    #self.sub_battery = rospy.Subscriber( 'mcu/battery_voltage', Float32, self.voltageCallback)

    self.initTimer( 0.1 )

    print 'Initialized '+NODE_NAME+'.'

  #  timer
  #-----------------------------------------------------#
  def initTimer( self, rate ):
    rospy.Timer(rospy.Duration( 1.0 / rate ), self.loop )

  def loop(self, e):
    if self.display_type == 'ip addr':
      self.lcd.sendText( 'IP: '+self.getIp() )

  #   
  #-----------------------------------------------------#
  def getIp( self ):
    return os.getenv( "ROS_IP", 'N/A' )

  def makeBatteryString( self, volts ):
    soc = volts + 10
    return 'state: '+'%.1f'%soc+' V: '+'%.1f'%volts

  #  sub callback 
  #-----------------------------------------------------#
  def lcdCallback(self, msg):
    data = msg.data
    print 'lcd callback: ['+data+']'
    self.lcd_text = data
    self.lcd.sendText( data )

  def voltageCallback( self, msg ):
    dt = ( rospy.Time.now() - self.start_time ).to_sec()
    if dt > self.init_display_time:
    #if ( self.display_type == 'battery' ):
      data = msg.data
      print 'got battery voltage: ['+str( data )+']'
      self.lcd_text = self.makeBatteryString( data ) 
      self.lcd.sendText( self.lcd_text )

  #  shutdown 
  #----------------------------------------------------------------#
  def shutdown(self):
    self.lcd.shutdown()


def main(args):
  
  print 'Starting ['+NODE_NAME+']'
  node = ROSNode() 
  rospy.spin()
  node.shutdown()
  print 'Stopping ['+NODE_NAME+']'


if __name__ == '__main__':
  main(sys.argv)
