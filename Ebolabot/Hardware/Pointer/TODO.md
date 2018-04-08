## High Level (Computer side)
#### Short term
  * Create a tuck command
  * Create a way to que up commands to send to PPU device that need to be run locally there
  * Have a clean assebly of PPU system that clearly communicates all coordinate systems

#### Long Term
  * Implement press function (waiting on filtered pos/vel/accel feedback)
  * Unify measurement system. Currently use degrees, rads, inches, stroke percent, raw. Pair down to raw, rad, and meters.
     * Solve IK and FK in radians and remove need for commandconversion function.
  * Add error checking to PPU client
  
## Low Level (Device side)
#### Short term
  * Only write to dynamixels if a new position is being written
  * Remove NULL ignore for pan/tilt because it prevents position 0 from being sent
  * Prevent out-of-bounds feedback from being sent for linac feedback
  * Motors rotate in opposite directions with increasing/decreasing values
  * Create a relax command for turning off the motors torqe
  * Make wiring safer on DC input

#### Long term
  * Replace hacky position feedback and add velocity feedback with Kalman filter to account for analog noise
    - Then implement veloity and overload feedback
    - Then implement force-press function
  * Replace linear actuator. Get one a new one we can program with custom control, buy a control board, or figure out how to use a Dynamixel as a linear actuator
