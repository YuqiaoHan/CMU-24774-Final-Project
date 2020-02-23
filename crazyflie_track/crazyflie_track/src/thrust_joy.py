#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from optitrack.msg import RigidBody, RigidBodyArray
from crazyflie_track.msg import TrajectoryPoint, Trajectory
from numpy import linalg as LA
import numpy as np
import time

pi = 3.14159265
G = 9.8  # Gravity acceleration
M = 0.03 # Weight of the drone.

class PID_Controller(object):

    def __init__(self, x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal):
        '''
        Initialize variables for controller.
        Args:
            xxx_goal: The input reference of the drone's states.
        '''
        print("Coming here.")
        self.z_goal = z_goal
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.roll_goal = roll_goal
        self.pitch_goal = pitch_goal
        self.yaw_goal = yaw_goal

        self.z_opti = 0
        self.x_opti = 0
        self.y_opti = 0
        self.roll_opti = 0
        self.pitch_opti = 0
        self.yaw_opti = 0

        self.prev_error_x = 0
        self.prev_error_y = 0
        self.prev_error_z = 0
        self.prev_error_yaw = 0

        self.sum_error_x = 0
        self.sum_error_y = 0 
        self.sum_error_z = 0
        self.sum_error_yaw = 0

        self.flag = 0
        self.err_z = 0
        self.dt = 0.01
        self.cur_time = 0
        self.prev_time = 0
        self.prev_Ez = 0
        self.Ez = 0

        self.t_takeoff_start = 0
        self.t_takeoff_end = 0
        self.t_landing_start = 0
        self.t_landing_start = 0

        self.height = 0
        self.real_x_data = []
        self.real_y_data = []
        self.real_z_data = []
        self.goal_x_data = []
        self.goal_y_data = []
        self.goal_z_data = []
        self.cmd_x_data = []
        self.cmd_y_data = []
        self.cmd_z_data = []
        self.err_x_data = []
        self.err_y_data = []
        self.err_z_data = []
        self.time_data = []

        self.traj_local = Trajectory()
        self.look_forward = 2
        
        ## The best pid for now
        # self.Kp = [12, -12, 13000, -15]; # [x,y,z,yaw]
        # self.Ki = [2, -2, 500, 0];
        # self.Kd = [50, -50, 9200, 0];

        ## THE BEST PID
        # self.Kp = [30, -30, 13000, -15]; # [x,y,z,yaw]
        # self.Ki = [4, -4, 500, 0];
        # self.Kd = [20, -20, 9200, 0];

        self.Kp = [30, -30, 13000, -15]; # [x,y,z,yaw]
        self.Ki = [4, -4, 500, 0];
        self.Kd = [20, -20, 9200, 0];

        # self.Kp = [13, -13, 11000, -15]; # [x,y,z,yaw]
        # self.Ki = [3, -3, 350, 0];
        # self.Kd = [55, -55, 9000, 0];

        self.command = Twist()
        self.pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=100)

        rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, self.rigid_body_cb)
        rospy.Subscriber('/crazyflie/local_trajectory', Trajectory, self.planner_cb)

        self.start_time = rospy.get_time()

        self.loop()


    def loop(self):
        '''
        Rospy loop function. Keep sending control message.
        '''
        print("Comes to the loop!!!!!!!!!!!!!")
        rate = rospy.Rate(100)
        rospy.on_shutdown(self.set_zero)
        
        while not rospy.is_shutdown():
            
            self.cur_time = rospy.get_time() - self.start_time
            self.time_data.append(self.cur_time)

            self.get_traj_circle()
            self.controller()
            self.pub.publish(self.command)

            rate.sleep()

        rospy.loginfo("lalalalalalalaaalalalalalalalalala")
        t_data = np.array(self.time_data)

        x_data = np.array(self.real_x_data)
        y_data = np.array(self.real_y_data)
        z_data = np.array(self.real_z_data)

        g_x_data = np.array(self.goal_x_data)
        g_y_data = np.array(self.goal_y_data)
        g_z_data = np.array(self.goal_z_data)

        c_x_data = np.array(self.cmd_x_data)
        c_y_data = np.array(self.cmd_y_data)
        c_z_data = np.array(self.cmd_z_data)

        e_x_data = np.array(self.err_x_data)
        e_y_data = np.array(self.err_y_data)
        e_z_data = np.array(self.err_z_data)

        np.savez('/home/acsi-mlc/Team6/crazyflie_ws/src/crazyflie_ros/crazyflie_track/src/G6_data_drone_and_obs2.npz', t=t_data, x=x_data, y=y_data, z=z_data, g_x=g_x_data, g_y=g_y_data, g_z=g_z_data,
                             c_x=c_x_data, c_y=c_y_data, c_z=c_z_data, e_x=e_x_data, e_y=e_y_data, e_z=e_z_data)



    def rigid_body_cb(self, array_msg):
        '''
        Call back function to update opti-track information.
        Args:
            array_msg: Message from rigid_bodies topic.
        '''
        self.x_opti = array_msg.bodies[0].pose.position.x
        self.y_opti = array_msg.bodies[0].pose.position.y
        self.z_opti = array_msg.bodies[0].pose.position.z


        q0 = array_msg.bodies[0].pose.orientation.x
        q1 = array_msg.bodies[0].pose.orientation.y
        q2 = array_msg.bodies[0].pose.orientation.z
        q3 = array_msg.bodies[0].pose.orientation.w

        [self.roll_opti,self.pitch_opti,self.yaw_opti] = tf.transformations.euler_from_quaternion([q0,q1,q2,q3])    

    
    def planner_cb(self,trajpoints):
        rospy.loginfo("Get the local trajectory.")
        self.traj_local = trajpoints
        self.height = self.traj_local.traj[0].z



    def traj_2_array(self, trajectory):
        '''
        Transfer Trajectory type to numpy array.
        Args:
            trajectory: list of TrajectoryPoint. Shape: n x 3
        '''
        n = len(trajectory)
        rospy.loginfo("n is: {}".format(n))
        global_traj_array = np.zeros((n, 3))
        for i in range(n):
            point = trajectory[i]
            global_traj_array[i] = np.array([point.x, point.y, point.z])
        
        return global_traj_array


    def project_traj(self):
        traj_array = self.traj_2_array(self.traj_local.traj)
        rospy.loginfo("Traj array shape {}".format(traj_array.shape))
        min_dist = 1000

        idx_forward = 0
        for i in range(len(traj_array)):
            traj_pos = traj_array[i][0:2]
            quad_pos = np.array([self.x_opti,self.y_opti])
            diff = LA.norm(traj_pos - quad_pos)
            if diff < min_dist:
                min_dist = diff
                idx_forward = i
        
        self.look_forward = 6
        self.x_goal = traj_array[idx_forward+self.look_forward][0]
        self.y_goal = traj_array[idx_forward+self.look_forward][1]
        self.z_goal = self.height

    def controller(self):
        '''
        Update 

        '''
        # ----------------------------record data for plotting--------------------------
        self.real_x_data.append(self.x_opti)
        self.real_y_data.append(self.y_opti)
        self.real_z_data.append(self.z_opti)

        self.goal_x_data.append(self.x_goal)
        self.goal_y_data.append(self.y_goal)
        self.goal_z_data.append(self.z_goal)

        # -------------------------------------------------------------------

        self.dt = 0.01 

        err_x_b = self.x_goal - self.x_opti
        err_y_b = self.y_goal - self.y_opti


        err_x = err_x_b*np.cos(self.yaw_opti) + err_y_b*np.sin(self.yaw_opti)
        err_y = -err_x_b*np.sin(self.yaw_opti) + err_y_b*np.cos(self.yaw_opti)
        err_z = self.z_goal - self.z_opti
        err_yaw = self.yaw_goal - self.yaw_opti

        # ----------------------------record data for plotting--------------------------
        self.err_x_data.append(err_x)
        self.err_y_data.append(err_y)
        self.err_z_data.append(err_z)
        # -------------------------------------------------------------------


        self.sum_error_x += err_x*self.dt
        self.sum_error_y += err_y*self.dt   
        self.sum_error_z += err_z*self.dt
        self.sum_error_yaw += err_yaw*self.dt

        if np.abs(self.sum_error_x) > 0.5: 
        	self.sum_error_x = 0
        if np.abs(self.sum_error_y) > 0.5: 
        	self.sum_error_y = 0
        if np.abs(self.sum_error_z) > 1000: 
        	self.sum_error_z = 0

        rospy.loginfo('The x sum error is: '+ str(self.sum_error_x))
        rospy.loginfo('The y sum error is: '+ str(self.sum_error_y))
        rospy.loginfo('The z sum error is: '+ str(self.sum_error_z))

        # -------------------------------------------------------------------
        # low pass filter on z derivative
        # I DIDNT TEST LPF YET, SO PROBABLY WONT WORK !! CHANGE lowpass TO TRUE/FALSE TO TURN LPF ON/OFF
        lowpass = False # turn on low pass filter for z derivative
        k = 0.01 # smooth factor
        self.Ez = (1-k)*self.prev_Ez+k*err_z/self.dt
        # -------------------------------------------------------------------
        if lowpass==True: 

            self.command.linear.x = self.Kp[0]*err_x + self.Kd[0]*(err_x-self.prev_error_x)/self.dt + self.Ki[0]*self.sum_error_x
            self.command.linear.y = self.Kp[1]*err_y + self.Kd[1]*(err_y-self.prev_error_y)/self.dt + self.Ki[1]*self.sum_error_y
            self.command.linear.z = self.Kp[2]*err_z + self.Kd[2]*(self.Ez-self.prev_Ez)/self.dt + self.Ki[2]*self.sum_error_z + 40700
        
        else:
            self.command.linear.x = self.Kp[0]*err_x + self.Kd[0]*(err_x-self.prev_error_x)/self.dt + self.Ki[0]*self.sum_error_x
            self.command.linear.y = self.Kp[1]*err_y + self.Kd[1]*(err_y-self.prev_error_y)/self.dt + self.Ki[1]*self.sum_error_y
            self.command.linear.z = self.Kp[2]*err_z + self.Kd[2]*(err_z-self.prev_error_z)/self.dt + self.Ki[2]*self.sum_error_z + 40700
            # self.command.angular.z = self.Kp[3]*err_yaw + self.Kd[3]*(err_yaw-self.prev_error_yaw)/self.dt + self.Ki[3]*self.sum_error_yaw

        self.saturate()

        self.prev_error_x = err_x
        self.prev_error_y = err_y     
        self.prev_error_z = err_z
        self.prev_error_yaw = err_yaw
        self.prev_Ez = self.Ez





        # ----------------------------record data for plotting--------------------------
        self.cmd_x_data.append(self.command.linear.x)
        self.cmd_y_data.append(self.command.linear.y)
        self.cmd_z_data.append(self.command.linear.z)

        # -------------------------------------------------------------------


        rospy.loginfo('----------------------------------')
        # print("Yaw opti: {}".format(self.yaw_opti))
        # rospy.loginfo('error x: {}, error y: {}, error z: {}'.format(err_x, err_y, err_z))

        # rospy.loginfo('linear x: ' + str(self.command.linear.x))
        # rospy.loginfo('linear y: ' + str(self.command.linear.y))
        rospy.loginfo('linear z: ' + str(self.command.linear.z))
        # rospy.loginfo('Angular z: ' + str(self.command.angular.z))
        rospy.loginfo('x goal: ' + str(self.x_goal) + ' y goal: ' + str(self.y_goal) + ' z goal: ' + str(self.z_goal))
        # rospy.loginfo('x goal: ' + str(self.x_goal))

    def saturate(self):

    	x_sat = 10
    	y_sat = 10
    	z_sat = 65000

        if self.command.linear.x >= 0:
            self.command.linear.x = min(x_sat, self.command.linear.x)
        elif self.command.linear.x < 0:
            self.command.linear.x = max(-x_sat, self.command.linear.x)
        if self.command.linear.y >= 0:
            self.command.linear.y = min(y_sat, self.command.linear.y)
        elif self.command.linear.y < 0:
            self.command.linear.y = max(-y_sat, self.command.linear.y)
        if self.command.linear.z >= 0:
            self.command.linear.z = min(z_sat, self.command.linear.z)
        elif self.command.linear.z < 0:
            self.command.linear.z = max(0, self.command.linear.z)


    def set_zero(self):

        self.command = Twist()
        self.command.linear.x = 0
        self.command.linear.y = 0
        self.command.linear.z = 0
        self.command.angular.x = 0
        self.command.angular.y = 0
        self.command.angular.z = 0

        rospy.loginfo("Shutdown initiated. Crazyflie will stop!")


    def change_goal(self, x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal):

        self.x_goal = x_goal
        self.y_goal = y_goal
        self.z_goal = z_goal
        self.roll_goal = roll_goal
        self.pitch_goal = pitch_goal
        self.yaw_goal = yaw_goal


    def get_traj_circle(self):

        t = self.cur_time;
        r = 0.75
        # self.height = 1.5

        self.t_takeoff_start = 0
        self.t_takeoff_end = 8
        self.t_traj_start = 8
        self.t_traj_end = 60
        self.t_landing_start = 60
        self.t_landing_end = 65

        if t>=self.t_takeoff_start and t<self.t_takeoff_end:
            self.z_goal = self.height*(t - self.t_takeoff_start)/(self.t_takeoff_end - self.t_takeoff_start)

        if t>=self.t_traj_start and t<=self.t_traj_end:
            self.project_traj()

        if t>self.t_landing_start and t<self.t_landing_end:
            self.z_goal = self.height*(self.t_landing_end - t)/(self.t_landing_end - self.t_landing_start)

        if t>= self.t_landing_end:
            self.z_goal = -20


    def get_traj_square(self):

        t = self.cur_time;
        d = 0.001

        if t<20:
            pass

        if t>=20:
            self.x_goal += d

        if t>=25:
            self.y_goal += d

        if t>=30:
            self.x_goal -= d

        if t>=35:
            self.y_goal -= d


if __name__ == '__main__':
    
    rospy.init_node('thrust_joy', anonymous=True)
    rospy.loginfo("Initialized Thrust_joy controller node.")
    
    thrust_joy = PID_Controller(0, 0, 0.0, 0, 0, 0)
    
    rospy.spin()