'''
Dhruv K Patel, Summer 2016
dhruv.patel@duke.edu

Purpose: to use 'BaseSensing' to simplify TRINA implementation into 'base_velocity.py'
'''

import sys, time, math

from BaseSensing import Base, Lidar

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl

'''
--------------
Base Constants
--------------
'''
base_h = 931   # (mm) Size of robot
base_w = 542
base_x = 0     # (mm) offset of robot [from center]
base_y = 0

'''
---------------
Lidar Constants
---------------
'''
lidar_offset = 14   # (mm) x & y axis offset of lidar [from robot corner]
lidar_protection = 75 # (mm) radius from lidar at which collision is assumed

'''
-----------------
Control Constants
-----------------
'''
vx_max = 0.13 # Actual max velocity: 0.1215 
vy_max = 0.08 # Actual max velocity: 0.07716
vth_max = 0.18 # Actual max velocity: 0.17911
cv_max = 0.2

class SafeBaseControl:
    def __init__(self):
        # initialize base
        self.base = Base(base_h, base_w, base_x, base_y)

        # add lidars
        self.lidars = []
        try:
            lidar1 = Lidar(1, base_x + (-base_w / 2) + lidar_offset, base_y + (base_h / 2) - lidar_offset, 180-45, True, lidar_protection)  # top left corner
            lidar2 = Lidar(2, base_x + (base_w / 2) - lidar_offset, base_y - (base_h / 2) + lidar_offset, -45, True, lidar_protection)  # bottom right corner
            self.isConnected = True
            self.enabled = True
            self.lidars = [lidar1, lidar2]
        except:
            self.isConnected = False
            self.enabled = False

        # initialize class properties
        self.cf_prev = 1.0
        self.cf = 1.0
        self.vx_prev = 0
        self.vy_prev = 0
        self.vth_prev = 0

        # time step for thread exit
        self.t_prev = time.time()

    def set_previous_velocity(self, previous_velocity):
        """
        Sets previous velocity values to be used when updating control factor
        """
        self.vx_prev = previous_velocity[0]
        self.vy_prev = previous_velocity[1]
        self.vth_prev = previous_velocity[2]


    def update_control_factor(self):
        """
        Uses previous velocity setting and previous control factor to determine new control factor
        """
        while 1:
            if self.enabled:
                vx = self.vx_prev
                vy = self.vy_prev
                vth = self.vth_prev

                # Convert velocity units
                vx *= 1000 * (vx_max/cv_max) # (m/s -> mm/s)
                vy *= 1000 * (vy_max/cv_max) # (m/s -> mm/s)
                vth /= 0.0174533 * (vth_max/cv_max) # (rad/s -> deg/s)

                # Update velocities
                self.base.set_vx(vx)
                self.base.set_vy(vy)
                self.base.set_vth(vth)

                # Update points
                for lidar in self.lidars:
                    lidar.get_data()
                # self.lidars = self.base.ignore_points(self.lidars)
                self.base.ignore_points(self.lidars)


                # Find time to collision
                self.t_to_collision, self.collision_point = self.base.time_to_collision(self.lidars)

                # Find control factor
                cf = self.base.get_control_factor(self.t_to_collision, self.collision_point)
                cf = (cf-0.2)*(1/(1-0.2)) # Map control factor because value of 0.2 yields stop at vmax
                self.cf = cf if cf > 0 else 0
            else:
                time.sleep(0.1)

                # self.cf = 1.0     
        # return cf

    def get_controlled_velocity(self, commanded_velocity, cf):
        """
        Uses commanded velocity and control factor to determine proper output velocity
        """
        cvx = commanded_velocity[0]
        cvy = commanded_velocity[1]
        cvth = commanded_velocity[2]

        vx = cvx * cf
        vy = cvy * cf
        vth = cvth * cf
        return (vx, vy, vth)

    def data_viewer(self):
        """
        For testing. Initializes plot of base, lidar, and data.
        * If using the motion computer, you must 'ssh -X'
        :return:
        """
        mpl.rcParams['toolbar'] = 'None'
        mpl.rcParams['toolbar'] = 'None'  # get rid of graph tools
        self.fig = plt.figure(1, figsize=(7, 7))
        self.fig.canvas.set_window_title('Mobile Base Collision Detection')
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        plt.axis('scaled')

        # Collect and plot data
        ani = animation.FuncAnimation(self.fig, self.collision_plot, interval = 50)
        plt.show()

    def collision_plot(self):
        """
        For testing. Updates plot of base, lidar, data, and collision trajectory using current velocities
        * If using the motion computer, you must 'ssh -X'
        """
        # Base
        self.base.plot_base(self.ax1)

        # Lidars
        for lidar in self.lidars:
            self.ax1.plot(lidar.x_pos, lidar.y_pos, 'y.', markersize=lidar.collision_radius, alpha=0.3)  # Location of Lidar

        # Points
        for lidar in self.lidars:
            self.ax1.plot(lidar.x, lidar.y, 'b.', markersize=2)

        # Collision course
        self.base.plot_closest_trajectory(self.t_to_collision, self.collision_point, self.ax1)


