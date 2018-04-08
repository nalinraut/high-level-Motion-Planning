from BaseSensing import Base, Lidar, _distance
from math import sqrt, sin, cos, radians
import sys, time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl

VX = 100 # mm/s
VY = 0 # mm/s
VTH = 0 # mm/s

'''
---------------------
Set up Base and Lidar
---------------------
'''
# Base:
base_h = 931   # Size of robot in mm
base_w = 542
base_x = 0     # offset of robot (from center)
base_y = 0
base = Base(base_h, base_w, base_x, base_y)

# Lidar:
lidar_offset = 17 # x/y axis offset of lidar (from robot corner)

try:
    laser1 = Lidar(1, base_x + (base_w / 2) + lidar_offset, base_y + (base_h / 2) + lidar_offset, 45, True, 35)           # top right
    # laser1 = Lidar(1, base_x + (base_w / 2) + lidar_offset, 0, 0, True, 35)                                               # middle right
    # laser1 = Lidar(1, base_x + (-base.w / 2) + lidar_offset, base_y + (base.h / 2) - lidar_offset, 180-45, True, 35)       # top left (actual)
    # laser1 = Lidar(1, base_x, base_y + (base.h/2) + lidar_offset, 90, False, 35)                                        # top middle
    # laser1 = Lidar(1, base_x + (-base.w / 2) - lidar_offset, base_y + (-base.h / 2) - lidar_offset, 180 + 45, True, 35)   # bottom left
    # laser1 = Lidar(1, base_x + (+base.w / 2) + lidar_offset, base_y + (-base.h / 2) - lidar_offset, -45, False, 35)        # bottom right

except:
    print 'Lidar 1 not connected'
    sys.exit()

'''
--------------------------------
Repeats for each Lidar iteration
--------------------------------
'''
# time code:
times = [0]*7
times0 = 0
time_counter = 0


def loop(i):
    global graphx
    global graphy
    # change velocities:
    base.set_vx(VX)
    base.set_vy(VY)
    base.set_vth(VTH)

    # time code:
    global times
    global times0
    global time_counter

    # time code:
    startTime0 = time.time()
    startTime = time.time()
    time_counter += 1

    # Collect data
    laser1.get_data()
    base.ignore_points([laser1])

    # time code:
    times[0] += time.time()-startTime
    print '\n', (str(time_counter) + ')'), ' vx:', base.get_vx(), '\tvy:', base.get_vy(), '\tvth:', base.get_vth()
    print '%f sec to %s' % (times[0]/time_counter, 'get data')
    startTime = time.time()

    # Plot robot base & lidars
    base.plot_base(ax1)
    ax1.plot(laser1.x_pos, laser1.y_pos, 'y.', markersize=laser1.collision_radius, alpha=0.3)  # Location of Lidar

    # time code:
    times[1] += time.time() - startTime
    print '%f sec to %s' % (times[1] / time_counter, 'plot base')
    startTime = time.time()

    # Plot data
    ax1.plot(laser1.x, laser1.y, 'b.', markersize=2) # All points

    # time code:
    times[2] += time.time() - startTime
    print '%f sec to %s' % (times[2] / time_counter, 'plot data')
    startTime = time.time()

    # Find collision Points
    t_to_collision, collision_point = base.time_to_collision([laser1], ax1)

    # time code:
    times[3] += time.time() - startTime
    print '%f sec to %s' % (times[3] / time_counter, 'search for collision')
    startTime = time.time()

    base.plot_closest_trajectory(t_to_collision, collision_point, ax1)

    # time code:
    times[4] += time.time() - startTime
    print '%f sec to %s' % (times[4] / time_counter, 'plot collision trajectory')
    startTime = time.time()

    # Find control factor and plot

    cf = base.get_control_factor(t_to_collision, collision_point)

    if len(graphy) <= max_graph_length:
        graphy.append(cf)
        graphx.append(len(graphy))
    else:
        graphx = []
        graphy = []
        ax2.cla()
    ax2.plot(graphx, graphy, 'k-')

    # time code:
    times[5] += time.time() - startTime
    print '%f sec to %s' % (times[5] / time_counter, 'find control factors')
    startTime = time.time()

    # Reconfigure axes and draw
    size = 1000
    ax1.axis([-size, size, -size, size])
    ax2.axis([0, max_graph_length, -0.1, 1.1])
    # plt.axis('scaled')
    plt.draw()
    ax1.grid(1)
    # ax1.axes.get_xaxis().set_visible(False)
    # ax1.axes.get_yaxis().set_visible(False)
    ax2.grid(1)

    # time code:
    times[6] += time.time() - startTime
    print '%f sec to %s' % (times[6] / time_counter, 'reconfigure axes')

    # time code:
    times0 += (time.time() - startTime0)
    print '%f sec TOTAL' % (times0 / time_counter)
    print '\nCollision in: %f sec' % t_to_collision
    print 'Control factor: %f' % cf

    ## Matlab code:
    # if len(collision_point) is not 0:
    #     print '%f, %f, %f, %f, %f' % (time.time(), t_to_collision, base.dist_to_base(collision_point[0], collision_point[1])[0], _distance(*collision_point), cf)
    #
    #     # Time, TTC, Closest d, Diagonal d, cf
'''
---------
Main Code
---------
'''

# Configure plot

# mpl.rcParams['toolbar'] = 'None' # get rid of graph tools
fig = plt.figure(1)
fig.canvas.set_window_title('Mobile Base Collision Detection')
# ax1 = fig.add_subplot(1,1,1)
ax1 = fig.add_subplot(1, 2, 1)
plt.axis('scaled')
ax2 = fig.add_subplot(1, 2, 2)
graphy = []
graphx = []
max_graph_length = 50

# Collect and plot data
ani = animation.FuncAnimation(fig, loop, interval = 50)
plt.show()