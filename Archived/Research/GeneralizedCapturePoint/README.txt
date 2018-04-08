Generalized Capture Point Demo Package

Kris Hauser

11/5/15 (updated 10/18/2016 to work with Klampt 0.6.2+)

************************************
The core C++ code is in gcp.h and gcp.cpp.  This will have no dependencies and can easily be added to your
project.

The demo programs can be built on Makefile-enabled systems using the commands:

  make main (default)
  make maingui
  make simgui

The latter two will require some external dependencies. By far the easiest way to get all the dependencies working is to just install Klamp't from the given tutorial:

   http://motion.pratt.duke.edu/klampt/tutorial_install.html

Finally, you should edit the Makefile variables KRISLIBRARY and KLAMPT to point to the correct paths.
By default they assume Klampt is installed in your home directory.

*************************************
maingui: a 2D GUI (maingui) that lets you test the 2D method by clicking to define a terrain and observing
the resulting capture points. 

Usage:
[space] toggles between modes 1) terrain definition / all CPs and paths, 2) find CP for a path, 3) find path for a CP.
r resets the terrain
[up/down/left/right] changes the initial velocity
Clicking with the left mouse button continues adding a polyline to the existing terrain, while the right mouse
button ends the existing polyline.

Requirements: KrisLibrary, which in turn requires a couple of dependencies. 

*************************************
simgui: a 3D GUI
Requirement: Klamp't and KrisLibrary and all dependencies.  KrisLibrary must also be patched with the two
experimental files given in this package.

It can be run "./simgui [meshfile]" where meshfile can be any file format that Assimp reads.  See the
data/ folder for examples, e.g., "./simgui data/fractal_terrain_2.tri"

Usage:
s: toggles simulation
p: toggles arrows controlling target point / perturbation mode
[up/down/left/right] changes the target / adds perturbations
