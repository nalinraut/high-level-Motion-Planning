To build, run:
cmake . -DKLAMPT_ROOT="path/to/Klampt" -DROS_WS="path/to/baxter_sdk" 
make

RUNNING THE TASK SERVER, SIMULATED ROBOT:

On the server machine
1. Change directories into this folder and run
      python Ebolabot/Common/system_state_service.py
   (check the printout to make sure it's set up on [MY_IP_ADDRESS]:4568).
3. Open a new console window, change directories into this folder and run 
      ./MotionServer_physical
4. Open a third console window, change directories into this folder and run
      ./ControllerDispatcher
   (optionally with the -v flag can be used to perform visual debugging)

On the client machine (this can be just the same machine.)
5. Run your UI to start serving tasks.  Make sure it is set up so the state
   service is [MY_IP_ADDRESS]:4568.

   Example: python /UI/haptic_widget.py.


RUNNING THE TASK SERVER, PHYSICAL ROBOT:

1. Start up the robot.

On the Motion laptop
2. Open a console window and run 
      cd ~/ros_ws/; . baxter.sh
   Change directories back into this folder and run
      python Ebolabot/Common/system_state_service.py
   (check the printout to make sure it's set up on [MOTION_IP_ADDRESS]:4568).
3. Open a new console window and run
      cd ~/ros_ws/; . baxter.sh
   Change directories back into this folder and run 
      ./MotionServer_physical
4. Open a third console window, change directories into this folder and run
      ./ControllerDispatcher
   (optionally with the -v flag can be used to perform visual debugging)

On the Console computer (for debugging this can be just the Motion laptop.)
5. Run your UI to start serving tasks.  Make sure it is set up so the state
   service is [MOTION_IP_ADDRESS]:4568.

   Example: python /UI/haptic_widget.py.
