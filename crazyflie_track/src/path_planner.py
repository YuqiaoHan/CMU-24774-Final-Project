#!/usr/bin/env python
import rospy
from crazyflie_track.msg import TrajectoryPoint, Trajectory

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from optitrack.msg import RigidBody, RigidBodyArray

import matplotlib.pyplot as plt
import time
from cvxpy import *
from numpy import linalg as LA
import numpy as np
import math

SAFE_MARGIN = 30 * 0.01

class PathPlanner(object):
    def __init__(self):
        self.flag = 0
        self.replanning_time = 0.2
        
        self.pathnew = None
        self.ego_pos = np.array([0, 0, 0]) # Initialize the ego position
        self.ego_idx = 0
        self.horizon = 40
        self.num_path_points = 0
        self.global_traj_array = None      # Shape: n x 3
        self.planning_flag = 0
        self.obstacles = []                # Obstacle list
        self.obstacles_height = []
        self.local_traj = Trajectory()
        self.obs_for_save = []
        
        rospy.Subscriber("original_trajectory", Trajectory, self.ori_traj_cb)
        # rospy.Subscirber("Obstacles", obstacle, self.obs_cb)
        print("Come here.")
        rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, self.position_cb)
        self.pub = rospy.Publisher("local_trajectory", Trajectory, queue_size=10)
        self.loop()
        

    def ori_traj_cb(self, data):
        '''
        Updates the original trajectory.
        Args:
            data: original trajectory file.
        '''
        if self.flag is 0:
            rospy.loginfo("Get into the original trajectory call back.")
            trajectory = data.traj
            self.global_traj_array = self.traj_2_array(trajectory)
            rospy.loginfo("Global trajectory is: {}".format(self.global_traj_array))
            self.pathnew = self.global_traj_array[0:self.horizon, :]
            self.num_path_points = self.global_traj_array.shape[0]
            rospy.loginfo("Global route's number of path points: {}".format(self.num_path_points))
            self.flag = 1
        else:
            pass

    def position_cb(self, array_msg):
        '''
        Updates the current position.
        Args:
            array_msg: current drone&obstacles position.
        '''
        self.ego_pos = np.empty(3)
        self.ego_pos[0] = array_msg.bodies[0].pose.position.x
        self.ego_pos[1] = array_msg.bodies[0].pose.position.y
        self.ego_pos[2] = array_msg.bodies[0].pose.position.z
        # rospy.loginfo("{}, {}, {}".format(array_msg.bodies[0].pose.position.x, array_msg.bodies[0].pose.position.y, array_msg.bodies[0].pose.position.z))

        self.obstacles = []
        self.obstacles_height = []

        for i in range(len(array_msg.bodies) - 1):
            obstacle = [array_msg.bodies[i+1].pose.position.x, array_msg.bodies[i+1].pose.position.y]
            self.obstacles.append(obstacle)
            self.obstacles_height.append(array_msg.bodies[i+1].pose.position.z)

        self.obs_for_save.append(self.obstacles)
        
        # rospy.loginfo("POSITION CALLBACK")
        # rospy.loginfo("Current ego position: x: {}, y: {}, z: {}".format(self.ego_pos[0], self.ego_pos[1], self.ego_pos[2]))
        # rospy.loginfo("Obstacles position shape: {}".format(self.obstacles))
    
    def loop(self):
        '''
        Run the planner with replanning time 0.2s.
        '''
        freq = int(1/self.replanning_time)
        rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            self.update_planner()
            rate.sleep()

        print("mamamamamamamamamamamamamamamamamamama")
        obs_traj = np.array(self.obs_for_save)
        np.savez('/home/acsi-mlc/Team6/crazyflie_ws/src/crazyflie_ros/crazyflie_track/src/G6_obs_data_drone_and_obs2.npz', obs_data=obs_traj)


    def traj_2_array(self, trajectory):
        '''
        Transfer Trajectory type to numpy array.
        Args:
            trajectory: list of TrajectoryPoint. Shape: n x 3
        '''
        n = len(trajectory)
        global_traj_array = np.zeros((n, 3))
        for i in range(n):
            point = trajectory[i]
            global_traj_array[i] = np.array([point.x, point.y, point.z])
        
        return global_traj_array

    def array_2_traj(self, traj_array):
        '''
        Transfer array type to Trajectory type
        Args:
            traj_array: Trajectory that is in array type.
        '''
        trajectory = Trajectory()
        for i in range(traj_array.shape[0]):
            point = TrajectoryPoint()
            point.x = traj_array[i][0]
            point.y = traj_array[i][1]
            point.z = traj_array[i][2]
            trajectory.traj.append(point)
        return trajectory
        

    def draw_trajectory(self):
        fig = plt.figure()
        plt.plot(self.pathnew[:, 0], self.pathnew[:, 1], color = "black")
        plt.plot(self.global_traj_array[:, 0], self.global_traj_array[:, 1], color = "green")
        for i in range(len(self.obstacles)):
            plt.scatter(self.obstacles[i][0], self.obstacles[i][1], marker = 'o', color = "red")
        plt.xlim(-1.60, 1.60)
        plt.ylim(-1.60, 1.60)
        plt.show()
    
    def get_taylor_expansion_pt(self, center, ref, radius):
        '''
        Get the reference points to do taylor expansion.
        Args:
            center: The center position of the obstacle.
            ref: The position of the reference point.
            radius: Minimum distance that new planned point should maintain with the obstacle.
        '''
        new_ref = np.zeros((2, 1))
        dx = center[0] - ref[0]
        dy = center[1] - ref[1]
        new_ref[0] = center[0] - radius * dx / np.sqrt(dx**2 + dy**2)
        new_ref[1] = center[1] - radius * dy / np.sqrt(dx**2 + dy**2)
        return new_ref
        
    def update_planner(self):
        '''
        Update the planned path.
        '''
        if self.flag is 1:  # If global trajectory is received, update planner. Otherwise, pass.
            idx_forward = 0
            min_dist = 10000
            for i in range(len(self.pathnew)):
                pos = self.pathnew[i][0:3]
                diff = LA.norm(pos - self.ego_pos)
                if diff < min_dist:
                    min_dist = diff
                    idx_forward = i
            
            
            self.ego_idx = self.ego_idx + idx_forward
            end_idx = self.ego_idx + self.horizon
            
            ref_path = None
            ref_path_1 = self.pathnew[idx_forward :]
            ref_2_flag = 0
            if end_idx < self.num_path_points:
                ref_2_flag = 0
                ref_path_2 = self.global_traj_array[self.ego_idx+self.horizon-idx_forward : self.ego_idx+self.horizon]
                ref_path = np.concatenate((ref_path_1, ref_path_2), axis = 0)
            else:
                start = self.ego_idx + self.horizon - idx_forward
                if start < self.num_path_points:
                    ref_2_flag = 1
                    temp_1 = self.global_traj_array[start :]
                    temp_2 = self.global_traj_array[0 : end_idx - self.num_path_points]
                    ref_path_2 = np.concatenate((temp_1, temp_2), axis = 0)
                else:
                    ref_2_flag = 2
                    ref_path_2 = self.global_traj_array[start - self.num_path_points : self.ego_idx+self.horizon-self.num_path_points]
                ref_path = np.concatenate((ref_path_1, ref_path_2), axis = 0)
            
            if self.ego_idx >= self.num_path_points:
                self.ego_idx = self.ego_idx - self.num_path_points

            # rospy.loginfo("\n")
            rospy.loginfo("\n ------------- UPDATE PLANNER -------------")
            rospy.loginfo("ego idx is: {}".format(self.ego_idx))
            rospy.loginfo("idxforward is: {}".format(idx_forward))
            rospy.loginfo("Ref 1 shape: {}, Ref 2 shape: {}".format(ref_path_1.shape, ref_path_2.shape))
            rospy.loginfo("Ref 2 type: {}".format(ref_2_flag))
            rospy.loginfo("Reference path shape: {}".format(ref_path.shape))
            rospy.loginfo("The ego position is : {}".format(self.ego_pos))
            rospy.loginfo("The obstacles are: {}".format(self.obstacles))

            '''
            Use current drone location and obstacle location to decide
            whether to consider the obstacles in planning.
            '''
            # Write code here...
            # Decide based on height.

            #Decide based on x and y.

            
            
            pathnew = self.plan_trajectory(10, ref_path, SAFE_MARGIN, self.obstacles)
            # self.obstacles = [] # Clean the obstacles list for next replanning.
            self.pathnew = pathnew.reshape(-1, 2)
            rospy.loginfo("pathnew shape: {}".format(self.pathnew.shape))
            self.pathnew = np.concatenate((self.pathnew, ref_path[:, 2].reshape(-1, 1)), axis = 1)
            rospy.loginfo("self.pathnew shape: {}".format(self.pathnew.shape))
            # self.draw_trajectory()
            self.local_traj = self.array_2_traj(self.pathnew)
            # print(self.local_traj)
            self.pub.publish(self.local_traj)
            self.local_traj = Trajectory() # Initialize the loca_traj after publishing it.
            rospy.loginfo("Successfully published a local trajectory.")
        else:
            pass


    def plan_trajectory(self, MAX_ITER, refpath_array, mini_distance, obstacles):
        '''
        Plan local trajectory.
        Args:
            MAX_ITER: Maximum number of iterations.
            refpath_array: Reference path.
            mini_distance: Minimum distance between drone and obstacles.
            obstacles: Obstacles that are taken into account.
        '''
        rospy.loginfo("Planning....")
        pathnew = None

        obstacle_status = 0
        if len(obstacles) >= 1:
            obstacle_status = 1
        
        if obstacle_status is 0:
            pathnew = refpath_array[:, 0:2].reshape(-1, 1)
            return pathnew
        
        oripath = refpath_array[:, 0:2].reshape(-1, 1)
        dim = 2

        Qref, Qabs, Aeq, beq, I_2, oripath, refpath = self.setup_problem(oripath)

        n = refpath.shape[0]
        nstep = int(refpath.shape[0] / 2)
        Qe = Qref + Qabs
        
        # Start the optimization.
        start = time.time()
        for i in range(MAX_ITER):
            print("Iteration {}".format(i))
            x = Variable(n)
            objective = Minimize(0.5*quad_form(x, Qe) + (np.matmul(-Qref, oripath)).T*x)
            constraints = []
            for j in range(nstep):
                if j < 1 or j >= nstep - 2:
                    cons_road_1 = x[2*j] - oripath[2*j] == 0
                    cons_road_2 = x[2*j+1] - oripath[2*j+1] == 0
                    constraints.append(cons_road_1)
                    constraints.append(cons_road_2)
                else:
                    for obs in obstacles:
                        obs = np.array([obs]).T
                        old_xref = refpath[2*j : 2*(j+1)]
                        xref = self.get_taylor_expansion_pt(obs, old_xref, mini_distance)
                        Q_obj = -2*(xref.T - obs.T)
                        R = np.matmul(xref.T, xref) - np.matmul(obs.T, obs) + mini_distance**2
                        cons_obs = Q_obj * x[2*j : 2*(j+1)] + R <= 0
                        constraints.append(cons_obs)

            p = Problem(objective, constraints)
            try:
                primal_result = p.solve()
            except:
                rospy.loginfo("Solve failure during iterations.")
                return refpath
            
            # Update the new path.
            pathnew = x.value

            if pathnew is None:
                rospy.loginfo("Optimal path is not found.")
                return refpath
            
            try:
                diff = LA.norm(refpath - pathnew)
            except:
                rospy.loginfo("Error when calculating diff.")
                return refpath
            if diff < 0.01*nstep*dim:
                break
            refpath = pathnew

        run_time = time.time() - start
        return pathnew

    def setup_problem(self, refpath):
        '''
        Set up the optimization problem.
        Args:
            refpath: Reference trajectory.
        '''
        I_2 = np.array([[1, 0, -1, 0],
                        [0, 1, 0, -1],
                        [-1, 0, 1, 0],
                        [0, -1, 0, 1]])
        nstep = int(len(refpath) / 2)
        oripath = refpath

        # Define the cost matrix
        dim = 2
        Q1 = np.eye(nstep*dim)
        Vdiff = np.eye(nstep*dim) - np.diag(np.ones((1, (nstep-1)*dim)).squeeze(), dim)
        Q2 = np.matmul(Vdiff[0:(nstep-1)*dim, :].T, Vdiff[0:(nstep-1)*dim, :])
        Adiff = Vdiff - np.diag(np.ones((1, (nstep-1)*dim)).squeeze(), dim) + np.diag(np.ones((1, (nstep-2)*dim)).squeeze(), dim*2)
        Q3 = np.matmul(Adiff[0:(nstep-2)*dim, :].T, Adiff[0:(nstep-2)*dim, :])

        # Define the cost matrix
        cref = np.array([0.2, 0, 0])
        cabs = np.array([0, 0, 100])

        Qref = Q1 * cref[0] + Q2 * cref[1] + Q3 * cref[2]
        Qabs = Q1 * cabs[0] + Q2 * cabs[1] + Q3 * cabs[2]

        # Fix the start and end point.
        Aeq = np.zeros((2*dim, nstep*dim))
        beq = np.zeros((2*dim, 1))

        Aeq[0:dim, 0:dim] = np.eye(dim)
        Aeq[dim:2*dim, (nstep-1)*dim:nstep*dim] = np.eye(dim)
        beq[0:2*dim] = np.concatenate((oripath[0:2], oripath[len(oripath)-2:]), axis = 0)

        return Qref, Qabs, Aeq, beq, I_2, oripath, refpath




if __name__ == '__main__':
    rospy.init_node("Path_planner")
    rospy.loginfo("Initialized planning node.")
    planner = PathPlanner()

    rospy.spin()

    # try:
    #     trace = TraceInit()    # z_goal, x_goal, y_goal, roll_goal, pitch_goal, yaw_goal
    # except:
    #     rospy.loginfo("thrust_joy object initialization failure.")
    
    # rospy.spin()