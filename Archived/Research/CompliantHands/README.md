= Righthand ReFlex Gripper simulation =
== Alessio Rocchi, Kris Hauser ==
== 5/20/16 ==

This package accompanies the paper:

> A. Rocchi, J. Li, B. Ames, and K. Hauser. Robust Simulation of Underactuated Compliant Hands.  IEEE Int'l. Conf. on Robotics and Automation, Stockholm, Sweden, 2016.

It simulates the Righthand ReFlex gripper grabbing and releasing a variety of different scanned objects.


=== Dependencies ===

This package requires Python 2.X and the Python Klamp't API, available at http://klampt.org.


=== Running ===

To run a single grasp experiment, call 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
python reflex_grasp_test.py worlds/hammer_P1.xml
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To run all available experiments in the worlds/ folder, call:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
python batch_test.py 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The results will be logged to [world file name]_state_log.csv, [world file name]_contact_log.csv, and [world file name].pickle.

The pickle file contains all the overall grasp results.  To output the results in csv format, run the pickle_to_csv.py script.


To calculate contact force, point, and normal variation metrics, call:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
python parse_contact_log.py [CONTACT LOG FILE] tmin tmax
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For the auto-grasp experiments, we used tmin = 1, tmax = 2 to get the contact variation over the lifting phase.


=== Changing the calibration and experimental conditions ===

To disable Klampt's BLEM contact detection technique and use your default ODE implementation, turn DISABLE_BLEM = True
at the top of reflex_grasp_test.py.

To tune the ReFlex tendon constants, change the values of tendon_c1 and tendon_c2 in reflex_col.py.

To tune the Reflex finger pad simulation parameters, you can change the following lines in reflex_col.py:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
s.kFriction = 1.5
s.kStiffness = 20000
s.kDamping = 20000
pad.setCollisionPadding(0.0025)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

