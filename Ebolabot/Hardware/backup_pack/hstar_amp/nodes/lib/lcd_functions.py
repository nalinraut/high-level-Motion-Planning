#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input:  
           output: 
           
  author:  ks
  created: 12/05/14

---------------------------------------------------------------------------'''

import roslib
import rospy, sys
from std_msgs.msg import String, Float32

import os
import sys

import serial, time, sys, os

#-----------------------------------------------------#
#  
#-----------------------------------------------------#
class SerialLcd:

  #  constructor 
  #-----------------------------------------------------#
  def __init__( self, parent, use_ros=True ):
    self.app = parent
    self.use_ros = use_ros

    #self.initDisplayOptions()
    if not use_ros:
      self.connect()
      self.init()
    else:
      self.pub = rospy.Publisher( 'mcu/lcd', String )

    print '\t- initialized lcd instance.'

  def connect( self ):
    self.ser = serial.Serial( self.app.port, 9600 )
    rospy.sleep( 0.1 )
    self.clearLcd()
    
  def init( self ):
    self.turnBacklightOff()
    self.start_time = rospy.Time.now()
    self.last_write_time = rospy.Time.now()
    self.loop_count = 0
    self.initTimer()

    # init text
    self.strings = {}
    #self.strings['clear'] = 254
    #self.initDisplayOptions()

  #  lcd functions 
  #-----------------------------------------------------#
  def clearLcd( self ):
    self.ser.write( '\xFE\x01' ) 

  def resetLcd( self ):
    self.ser.write( '\xFE\x80' ) 
    self.clearLcd()

  def sendRosText( self, data ):
    self.pub.publish( data )

  def sendText( self, data ):
    if self.use_ros:
      self.sendRosText( data )
    else:
      self.resetLcd()
      if self.dim:
        self.turnBacklightOn()
      print 'sending text ['+data+']'
      for letter in data:
        #self.ser.write( letter )
        self.writeToLcd( letter )

  def writeToLcd( self, data ):
    self.ser.write( data )
    print 'writing: ' + data
    self.last_write_time = rospy.Time.now()

  def turnBacklightOff( self ):
    print 'backlight off'
    self.ser.write( '\x7C\x80' )
    self.dim = True
    rospy.sleep( 0.10 )

  def turnBacklightOn( self ):
    print 'backlight on'
    self.ser.write( '\x7C\x9d' )
    self.dim = False
    rospy.sleep( 0.10 )

  def checkBackLight( self ):
    if not self.dim:
      dt = ( rospy.Time.now() - self.last_write_time ).to_sec()
      if dt > self.dim_timeout:
        self.turnBacklightOff()

  #  timer
  #-----------------------------------------------------#
  def initTimer( self ):
    rospy.Timer(rospy.Duration( 1.0 / self.app.timer_rate ), self.loop)
    self.loop_count

  def loop(self, e):
    #self.sendText( self.lcd_text )
    if self.app.enable_dimming:
      self.checkBackLight()
    self.loop_count = self.loop_count + 1

  #  shutdown 
  #----------------------------------------------------------------#
  def shutdown(self):
    if not self.use_ros:
      self.ser.close()
      print 'close serial port'  

