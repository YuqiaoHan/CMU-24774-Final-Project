import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
data = np.load('G6_data_no_obs.npz')
x_real = data['x']
y_real = data['y']
z_real = data['z']
x_goal = data['g_x']
y_goal = data['g_y']
z_goal = data['g_z']
z_goal = (z_goal >= 0)
t = data['t']

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x_real, y_real, z_real, label = "Real")
ax.plot3D(x_goal, y_goal, z_goal, label = "Goal")
ax.set_zlim(0, 1.5)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
plt.legend()
plt.title('Trajectory Tracking with No Obstacle')


fig = plt.figure()
plt.plot(x_real, y_real, label = 'Real')
plt.plot(x_goal, y_goal, label = 'Goal')
plt.ylabel('Y(m)')
plt.xlabel('X(m)')
plt.legend()

fig = plt.figure()
plt.plot(x_real, z_real, label = 'Real')
plt.plot(x_goal, z_goal, label = 'Goal')
plt.ylabel('Z(m)')
plt.xlabel('X(m)')
plt.ylim(0,1.5)
plt.legend()

fig = plt.figure()
plt.plot(y_real, z_real, label = 'Real')
plt.plot(y_goal, z_goal, label = 'Goal')
plt.ylabel('Z(m)')
plt.xlabel('Y(m)')
plt.ylim(0,1.5)
plt.legend()

plt.show()

############################################################################

x_err = data['e_x']
y_err = data['e_y']
z_err = data['e_z']
t = t[0:7500]
x_err = x_err[0:7500]
y_err = y_err[0:7500]
z_err = z_err[0:7500]


plt.subplot(3,1,1)
plt.plot(t, x_err, label = 'X error')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('m')
plt.title('Errors')

plt.subplot(3,1,2)
plt.plot(t, y_err, label = 'Y error')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('m')

plt.subplot(3,1,3)
plt.plot(t, z_err, label = 'Z error')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('m')
plt.show()

#########################################################################################################
x_command = data['c_x']
y_command = data['c_y']
z_command = data['c_z']
x_command = x_command[0:7500]
y_command = y_command[0:7500]
z_command = z_command[0:7500]

plt.subplot(3,1,1)
plt.plot(t, x_command, label = 'X command')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('V(cm/s)')
plt.title('Command')

plt.subplot(3,1,2)
plt.plot(t, y_command, label = 'Y command')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('V(cm/s)')

plt.subplot(3,1,3)
plt.plot(t, z_command, label = 'Z command')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('PWM')
plt.show()

##########################################################################################################
one_obs_data = np.load('G6_obs_data_one_static.npz')
obstacle_data = one_obs_data['obs_data']
obs_x = -1.21217728
obs_y = 0.07643756
obs_z = 0.7

one_obs_cf_data = np.load('G6_data_one_static.npz')
x_real = one_obs_cf_data['x']
y_real = one_obs_cf_data['y']
z_real = one_obs_cf_data['z']
x_goal = one_obs_cf_data['g_x']
y_goal = one_obs_cf_data['g_y']
z_goal = one_obs_cf_data['g_z']
z_goal = (z_goal >= 0)*0.7


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x_real, y_real, z_real, label = "Real")
ax.plot3D(x_goal, y_goal, z_goal, label = "Goal")
ax.scatter(obs_x, obs_y, obs_z, s = 200, color = 'red', label= 'obstacle')
ax.set_zlim(0, 1.5)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
plt.legend(loc = 'best')
plt.title('Trajectory Tracking with One Obstacle')

fig = plt.figure()
plt.plot(x_real, y_real, label = 'Real')
plt.plot(x_goal, y_goal, label = 'Goal')
plt.scatter(obs_x, obs_y, s = 200, color = 'red')
plt.ylabel('Y(m)')
plt.xlabel('X(m)')
plt.legend()

fig = plt.figure()
plt.plot(x_real, z_real, label = 'Real')
plt.plot(x_goal, z_goal, label = 'Goal')
plt.scatter(obs_x, obs_z, s = 200, color = 'red')
plt.ylabel('Z(m)')
plt.xlabel('X(m)')
plt.ylim(0,1.5)
plt.legend()

fig = plt.figure()
plt.plot(y_real, z_real, label = 'Real')
plt.plot(y_goal, z_goal, label = 'Goal')
plt.scatter(obs_y, obs_z, s = 200, color = 'red')
plt.ylabel('Z(m)')
plt.xlabel('Y(m)')
plt.ylim(0,1.5)
plt.legend()

plt.show()

##################################################################################

x_err = one_obs_cf_data['e_x']
y_err = one_obs_cf_data['e_y']
z_err = one_obs_cf_data['e_z']
t = one_obs_cf_data['t']
t = t[0:4500]
x_err = x_err[0:4500]
y_err = y_err[0:4500]
z_err = z_err[0:4500]

plt.subplot(3,1,1)
plt.plot(t, x_err, label = 'X error')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('m')
plt.title('Errors')

plt.subplot(3,1,2)
plt.plot(t, y_err, label = 'Y error')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('m')

plt.subplot(3,1,3)
plt.plot(t, z_err, label = 'Z error')
plt.legend(loc = 'best')
plt.xlabel('Time(s)')
plt.ylabel('m')
plt.show()


