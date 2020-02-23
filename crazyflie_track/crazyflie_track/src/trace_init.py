#!/usr/bin/env python
import rospy
# from sensor_msgs.msg import Joy
# from geometry_msgs.msg import Pose, Point, Quaternion
from crazyflie_track.msg import TrajectoryPoint, Trajectory
import matplotlib.pyplot as plt
import numpy as np
import math

class TraceInit(object):
    def __init__(self):
        self.flag = 1
        self.trajectory = Trajectory()
        # self.trajectory = []
        self.radius = 60 *0.01                       # Unit: meter.
        self.point0 = np.array([0, 0]) * 0.01
        self.point1 = np.array([0, 50]) * 0.01
        self.point2 = np.array([-120, 50]) * 0.01
        self.point3 = np.array([-120, -50]) * 0.01
        self.point4 = np.array([0, -50]) * 0.01
        self.height = 0.7
        self.pub = rospy.Publisher("original_trajectory", Trajectory, queue_size=10)
        self.create_trajectory()
        # print(self.trajectory)
        # self.draw_trajectory()

    def create_trajectory(self):
        self.get_interpolate(self.point0, self.point1, 0.05, "straight_line")
        self.get_interpolate(self.point1, self.point2, math.pi/38, "circle", 0)
        self.get_interpolate(self.point2, self.point3, 0.05, "straight_line")
        self.get_interpolate(self.point3, self.point4, math.pi/38, "circle", math.pi)
        self.get_interpolate(self.point4, self.point0, 0.05, "straight_line")
        # self.trajectory.traj.append(self.point1)
        
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.trajectory)
            rate.sleep()
        

    def get_interpolate(self, loc_1, loc_2, resolution = 5, mode = "straight_line", angle_start = 0):
        '''
        Args:
            loc_1, loc_2: Fisrt and last point location. Type: List. Dimension: 1x2
            resolution: For straight_line, it is the distance between two points. For circle
                        it is the angle between two points.
            mode: Mode of interpolation. It can be straight_line or circle.
            angle_start: Start angle for the circle interpolation
        '''

        if mode is "straight_line":
            dist = np.linalg.norm(loc_1 - loc_2)
            points_num = int(dist // resolution) + 1 # From loc_1 point to the last point between loc_1 and loc_2. Not including loc_2.
            # print(points_num)
            for i in range(points_num):
                point = TrajectoryPoint()
                # point = np.zeros((2))
                point.x = (points_num-i)/float(points_num) * loc_1[0] + i/float(points_num) * loc_2[0]
                point.y = (points_num-i)/float(points_num) * loc_1[1] + i/float(points_num) * loc_2[1]
                point.z = self.height
                self.trajectory.traj.append(point)
            
        elif mode is "circle":
            center = (loc_1 + loc_2) / 2                 # Get the circle center defined by the two points
            points_num = int(math.pi // resolution) + 1  # Get number of interpolate points.
            # print("Center is: {}".format(center))
            for i in range(points_num):
                point = TrajectoryPoint()
                # point = np.zeros((2))
                theta = angle_start + math.pi * (i/float(points_num))         # Rotation angle
                point.x = center[0] + self.radius * math.cos(theta)
                point.y = center[1] + self.radius * math.sin(theta)
                point.z = self.height
                # print("theta: {}, cos(theta): {}".format(theta, math.cos(theta)))
                # print(point)
                self.trajectory.traj.append(point)
        
        else:
            pass
            # rospy.loginfo("Interpolation mode not defined.")

    def draw_trajectory(self):
        array_traj = np.array(self.trajectory)
        fig = plt.figure()
        plt.plot(array_traj[:, 0], array_traj[:, 1])
        plt.xlim(-1.60, 0.80)
        plt.ylim(-1.20, 1.20)
        plt.show()


if __name__ == '__main__':
    rospy.init_node("Trace_init")
    rospy.loginfo("Initialized trace publishing node.")
    trace = TraceInit()
    

    rospy.spin()
    # try:
    #     trace = TraceInit()    # z_goal, x_goal, y_goal, roll_goal, pitch_goal, yaw_goal
    # except:
    #     rospy.loginfo("thrust_joy object initialization failure.")
    
    # rospy.spin()