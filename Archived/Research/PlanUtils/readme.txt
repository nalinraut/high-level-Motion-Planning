##############################################################
#                      PlanUtils Readme                      #
##############################################################

Last updated: 5/18/2010

#####
About

Command line utilities for building, viewing, and
processing probabilistic roadmaps for motion planning.


#####
Prerequisites

- Built on Linux and Cygwin.

- KrisLibrary (available on SVN repository)
  Use 'make all' to build the source, then 'make KrisLibrary' to
  build the libKrisLibrary.a library.

- Tinyxml 2.6.3 or later. (available on SVN repository)
  Turn on STL support in the Makefile by setting TINYXML_USE_STL=YES.
  Run 'make', and then group all of the .o files into a shared
  library 'libtinyxml.a' using the commands:
    ar rcs libtinyxml.a *.o
    ranlib libtinyxml.a

- OpenGL and GLUT (optional, for roadmap and Cspace visualization)

- Graclus (optional, for graph clustering)

- GraphVis (optional, for graph visualization)

- Python (2.x or 3.x are ok)


#####
Programs

PRMBuild builds a PRM using a given plannera and a given state space.
PRMStats prints and computes information about a PRM.
PRMVis visualizes a PRM, and performs single-query planning.

Usage and command line options can be queried using the command
  '[Program name] -h'


#####
File formats

Roadmaps are stored in the Trivial Graph Format (TGF).  If nodes
contain configuration information, the configuration is written
as a line "#entries v1 v2 ... vn" where n is the number of entries
in the configuration vector.

C-spaces are stored in XML format.  Supported types of cspaces are
2d points among obstacles, rigid translating 2d objects, and rigid
translating and rotating 2d objects.  See the spaces/ subdirectory
for examples.

Planners are also stored in XML format.  Supported planners are
PRM, RRT, bidirectional RRT, SBL, and PRT.  See the planners/
subdirectory for examples.



