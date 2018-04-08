=======================================================================================
OpenHaptics -> SSPP Haptic Service

Kris Hauser
11/24/2014

=======================================================================================

These files contain SSPP Services for reading data from / sending commands to a
OpenHaptics device.

==============================================
JSON Schema
==============================================

========
State
========
{
  "type" : "MultiHapticState",
  "devices" : [
     {
        "time" : float,        //time of reading relative to service start
        "position" : [x,y,z],  //position in meters
        "velocity" : [vx,vy,vz],  //velocity in meters / second
        "rotationMoment" : [mx,my,mz],   //moment representation of rotation
        "angularVelocity" : [wx,wy,wz],  //angular velocity in rads / second
        "force" : [fx,fy,fz],       //rendered force at tooltip (units unknown... Newtons?)
        "button1" : 0 or 1,       //button 1 currently pressed
        "button2" : 0 or 1,       //button 2 currently pressed
     }
     ...
  ]
  "events" : [
     {
        "type" : "b1_down", "b1_up", "b2_down", "b2_up",  //type of event
        "device" : 0 or 1,   //which device had the event
     }
  ]
}

=======
Command
=======
{
  "type" : "HapticForceCommand",
  "device" : int,    //device index
  "enabled" : 0 or 1,   //set to 1 to enable forces
  *"overrideContinuousForceLimits" : 0 or 1, //set to 1 to use maximum forces (may raise motor temp)
  *"center" : [cx,cy,cz],   //center of spring
  *"constant" : [fx,fy,fz],   //spring force offset
  *"linear" : [k] or [kx,ky,kz] or [kxx,kxy,kxz,kyx,kyy,kyz,kzx,kzy,kzz],   //gain matrix (should be negative-definite)
  *"damping" : [k] or [kx,ky,kz] or [kxx,kxy,kxz,kyx,kyy,kyz,kzx,kzy,kzz],   //damping matrix (should be negative-definite)
  *"forceCenter": [vx,vy,vz]    //center of damping
}

* indicates optional item
