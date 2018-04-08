The RoboSlider provides for reading and commanding the position of audio sliders
and buttons, and is specialized for the control of the joint angles any 7 DOF robot.
	*  slider.py -- communicates with RoboSlider devices, allowing for reading and
	   writing slider positions and button states. Also provides easy methods for
	   mapping input and output.
	*  /slider_driver -- contains the Arduino code (and dependent files) for the low
	   level controls of the slider, and for messaging with the computer.
	*  /Hardware -- contains the PCB and enclosure designs for the most recent slider
	   design, and includes a detailed Bill of Materials with links to suppliers.
	*  baxter_slider.py -- utilized the slider module to control Baxter's joint angles
	   with absolute positioning between the slider and Baxter.