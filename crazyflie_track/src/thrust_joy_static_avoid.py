#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from optitrack.msg import RigidBody, RigidBodyArray
import numpy as np
import time
import math

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

        # self.Kp = [12, -12, 11000, -15]; # [x,y,z,yaw]
        # self.Ki = [2, -2, 350, 0];
        # self.Kd = [50, -50, 9000, 0];

        self.Kp = [12, -12, 13000, -15]; # [x,y,z,yaw]
        self.Ki = [2, -2, 500, 0];
        self.Kd = [50, -50, 9200, 0];

        self.x_obst = 0
        self.y_obst = 0
        self.z_obst = 0

        self.x_final_traj = []
        self.y_final_traj = []

        self.x_trajectory = []
        self.y_trajectory = []
        self.traj_radius = 0.75
        self.direction = 1
        
        self.start_pos = [0,0]
        self.current_pos = [0,0]
        self.pos_index = 0

        self.obstacle_pos = [5,5]
        self.obstacle_size = 0.1
        self.avoid_traj_x = []
        self.avoid_traj_y = []

        self.n = 3000 # more points, slower the trajectory

        self.intersect1 = []
        self.intersect2 = []
        self.index1 = 0
        self.index2 = 0

        self.m = int(2*self.n*(2*self.obstacle_size/(2*pi*self.traj_radius)))

        for i in range(self.n+1):
            t = 2*pi*i/self.n
            x_traj = self.traj_radius*np.cos(t) - self.traj_radius
            y_traj = self.traj_radius*np.sin(t)
            self.x_trajectory.append(x_traj)
            self.y_trajectory.append(y_traj)

            # x_ob = self.obstacle_size*np.cos(t) + self.obstacle_pos[0]
            # y_ob = self.obstacle_size*np.sin(t) + self.obstacle_pos[1]
            # self.x_obst.append(x_ob)
            # self.y_obst.append(y_ob)

        self.command = Twist()
        self.pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=100)

        rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, self.rigid_body_cb)


        self.start_time = rospy.get_time()

        self.loop()


    def loop(self):
        '''
        Rospy loop function. Keep sending control message.
        '''
        print("Comes to the loop!!!!!!!!!!!!!")
        rate = rospy.Rate(100)
        rospy.on_shutdown(self.set_zero)
    
        # self.compute_traj()
        compute_init = True

        while not rospy.is_shutdown():
            
            self.cur_time = rospy.get_time() - self.start_time
            # rospy.loginfo('x obstacle: ' + str(self.x_obst))

            # if self.cur_time>5:
            #     if compute_init==True:
            #         self.compute_traj()
            #         compute_init=False

            self.get_traj_circle()
            self.controller()  
            self.pub.publish(self.command)

            rate.sleep()

    def rigid_body_cb(self, array_msg):
        '''
        Call back function to update opti-track information.
        Args:
            array_msg: Message from rigid_bodies topic.
        '''
        self.x_opti = array_msg.bodies[0].pose.position.x
        self.y_opti = array_msg.bodies[0].pose.position.y
        self.z_opti = array_msg.bodies[0].pose.position.z

        # self.x_obst = array_msg.bodies[1].pose.position.x
        # self.y_obst = array_msg.bodies[1].pose.position.y
        # self.z_obst = array_msg.bodies[1].pose.position.z

        # self.obstacle_pos[0] = self.x_obst
        # self.obstacle_pos[1] = self.y_obst

        q0 = array_msg.bodies[0].pose.orientation.x
        q1 = array_msg.bodies[0].pose.orientation.y
        q2 = array_msg.bodies[0].pose.orientation.z
        q3 = array_msg.bodies[0].pose.orientation.w

        [self.roll_opti,self.pitch_opti,self.yaw_opti] = tf.transformations.euler_from_quaternion([q0,q1,q2,q3])    

    def controller(self):
        '''
        Update 

        '''

        self.dt = 0.01 

        err_x_b = self.x_goal - self.x_opti
        err_y_b = self.y_goal - self.y_opti

        err_x = err_x_b*np.cos(self.yaw_opti) + err_y_b*np.sin(self.yaw_opti)
        err_y = -err_x_b*np.sin(self.yaw_opti) + err_y_b*np.cos(self.yaw_opti)
        err_z = self.z_goal - self.z_opti
        err_yaw = self.yaw_goal - self.yaw_opti

        self.sum_error_x += err_x*self.dt
        self.sum_error_y += err_y*self.dt   
        self.sum_error_z += err_z*self.dt
        self.sum_error_yaw += err_yaw*self.dt

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
        if self.command.linear.x >= 0:
            self.command.linear.x = min(30, self.command.linear.x)
        elif self.command.linear.x < 0:
            self.command.linear.x = max(-30, self.command.linear.x)
        if self.command.linear.y >= 0:
            self.command.linear.y = min(30, self.command.linear.y)
        elif self.command.linear.y < 0:
            self.command.linear.y = max(-30, self.command.linear.y)
        if self.command.linear.z >= 0:
            self.command.linear.z = min(65000, self.command.linear.z)
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


    # def get_traj_circle(self):

    #     t = self.cur_time;
    #     r = 0.75
    #     height = 2

    #     self.x_goal = self.x_final_traj[self.pos_index]
    #     self.y_goal = self.y_final_traj[self.pos_index]
    #     self.z_goal = height
    #     self.pos_index += 1



    def get_traj_circle(self):

        t = self.cur_time;
        r = 0.75
        height = 2

        self.t_takeoff_start = 0
        self.t_takeoff_end = 25
        self.t_traj_start = 25
        self.t_traj_end = 65
        self.t_landing_start = 65
        self.t_landing_end = 70

        if t>=self.t_takeoff_start and t<self.t_takeoff_end:
            self.z_goal = height*(t - self.t_takeoff_start)/(self.t_takeoff_end - self.t_takeoff_start)

        if t>=self.t_traj_start and t<=self.t_traj_end:

            self.x_goal = self.x_final_traj[self.pos_index]
            self.y_goal = self.y_final_traj[self.pos_index]
            self.z_goal = height
            self.pos_index += 1

            if self.pos_index == (len(self.x_final_traj)-1):
                self.pos_index = 0


        if t>self.t_landing_start and t<self.t_landing_end:
            self.z_goal = height*(self.t_landing_end - t)/(self.t_landing_end - self.t_landing_start)

        if t>= self.t_landing_end:
            rospy.signal_shutdown("Landed the quadcopter!")


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

    def check_intersection(self):
        self.flag = 0

        for i in range(1,self.n-1):
            circle_eqn_prev = ((self.x_trajectory[i-1] - self.obstacle_pos[0])**2 + (self.y_trajectory[i-1] - self.obstacle_pos[1])**2 - self.obstacle_size**2)
            circle_eqn_now = ((self.x_trajectory[i] - self.obstacle_pos[0])**2 + (self.y_trajectory[i] - self.obstacle_pos[1])**2 - self.obstacle_size**2)
            circle_eqn_next = ((self.x_trajectory[i+1] - self.obstacle_pos[0])**2 + (self.y_trajectory[i+1] - self.obstacle_pos[1])**2 - self.obstacle_size**2)

            if circle_eqn_next <= 0 and circle_eqn_now >= 0:
                self.intersect1 = [self.x_trajectory[i],self.y_trajectory[i]]
                self.flag = 1
                self.index1 = i
            elif circle_eqn_prev <= 0 and circle_eqn_now >= 0:
                self.intersect2 = [self.x_trajectory[i],self.y_trajectory[i]]
                self.flag = 1
                self.index2 = i

        if (self.flag == 1):
            rospy.loginfo("Obstacle intersects the path!")
            print(self.intersect1)
            print(self.intersect2)
        else:
            rospy.loginfo("Obstacle does not intersect the path!")

    def avoid_obstacle(self):
        
        cos_theta1 = (self.intersect1[0] - self.obstacle_pos[0])/self.obstacle_size
        sin_theta1 = (self.intersect1[1] - self.obstacle_pos[1])/self.obstacle_size
        theta1 = math.atan2(sin_theta1,cos_theta1)
        cos_theta2 = (self.intersect2[0] - self.obstacle_pos[0])/self.obstacle_size
        sin_theta2 = (self.intersect2[1] - self.obstacle_pos[1])/self.obstacle_size
        theta2 = math.atan2(sin_theta2,cos_theta2)

        t = np.linspace(theta1,theta2,self.m)

        for i in range(self.m):
            x = self.obstacle_pos[0] + self.obstacle_size*np.cos(t[i])
            y = self.obstacle_pos[1] + self.obstacle_size*np.sin(t[i])
            self.avoid_traj_x.append(x)
            self.avoid_traj_y.append(y)
        

    def traj_to_follow(self):
        self.x_final_traj = self.x_trajectory[self.pos_index:self.index1] + self.avoid_traj_x + self.x_trajectory[self.index2:-1] + self.x_trajectory[0:self.pos_index]
        self.y_final_traj = self.y_trajectory[self.pos_index:self.index1] + self.avoid_traj_y + self.y_trajectory[self.index2:-1] + self.y_trajectory[0:self.pos_index]
        # print("----------------------------Generated trajectory:----------------------------")
        # print(self.x_final_traj)


        # print(self.y.final_traj)

    def get_pos_index(self):
        min_dist = (self.x_trajectory[0]-self.current_pos[0])**2 + (self.y_trajectory[0]-self.current_pos[1])**2

        for i in range(len(self.x_final_traj)):
            dist = (self.x_trajectory[i]-self.current_pos[0])**2 + (self.y_trajectory[i]-self.current_pos[1])**2
            if (dist < min_dist):
                min_dist = dist
                self.pos_index = i

    def compute_traj(self):

        self.x_final_traj = self.x_trajectory
        self.y_final_traj = self.y_trajectory

        # self.check_intersection()	
        # if self.flag == 1:
        #     self.avoid_obstacle()
        #     self.traj_to_follow()
        # else:
        #     self.x_final_traj = self.x_trajectory
        #     self.y_final_traj = self.y_trajectory


if __name__ == '__main__':
    
    rospy.init_node('thrust_joy', anonymous=True)
    rospy.loginfo("Initialized Thrust_joy controller node.")
    
    thrust_joy = PID_Controller(0, 0, 0.0, 0, 0, 0)
    
    rospy.spin()