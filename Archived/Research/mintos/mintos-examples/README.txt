###########################################################

Mintos example files

Kris Hauser (hauserk@indiana.edu)

###########################################################

For use with the timeopt program in Mintos (http://www.iu.edu/~motion/mintos).

The trajopt_interp_[N].spline files gives bezier splines interpolated for an
N-joint robot with constrained endpoints.

The pi_limits_[N].txt files give velocity and acceleration limits of the range [-pi,pi]^N
(units are radians/s and radians/s^2 respectively).

Example usage:

  ./trajopt mintos-examples/trajopt_interp_10.spline mintos-examples/pi_limits_10.txt 1000 0.1

This optimizes a time-scaling for the 10-joint robot arm with 1000 grid points.  The resulting
time scaling is printed on a 0.1s grid.

