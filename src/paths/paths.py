#!/usr/bin/env python

"""
Starter script for lab1.
Author: Chris Correa
"""
import numpy as np
import math
import matplotlib.pyplot as plt

import utils.utils as utils

try:
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except:
    pass

class MotionPath:
    def __init__(self, limb, kin, total_time):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        total_time : float
            number of seconds you wish the trajectory to run for
        """
        self.limb = limb
        self.kin = kin
        self.total_time = total_time

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        pass

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        pass

    def plot(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t) for t in times])
        target_velocities = np.vstack([self.target_velocity(t) for t in times])
        target_acceleration = np.vstack([self.target_acceleration(t) for t in times])

        plt.figure()
        plt.subplot(3,3,1)
        plt.plot(times, target_positions[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Position")

        plt.subplot(3,3,2)
        plt.plot(times, target_velocities[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Velocity")

        plt.subplot(3,3,4)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,3,5)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")

        plt.subplot(3,3,7)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,3,8)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.subplot(3,3,3)
        plt.plot(times, target_acceleration[:,0], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("x acceleration")

        plt.subplot(3,3,6)
        plt.plot(times, target_acceleration[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("y acceleration")

        plt.subplot(3,3,9)
        plt.plot(times, target_acceleration[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("z acceleration")


        plt.show()

    def to_robot_trajectory(self, num_waypoints=300, jointspace=True):
        """
        Parameters
        ----------
        num_waypoints : float
            how many points in the :obj:`moveit_msgs.msg.RobotTrajectory`
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.
        """
        traj = JointTrajectory()
        traj.joint_names = self.limb.joint_names()
        points = []
        for t in np.linspace(0, self.total_time, num=num_waypoints):
            point = self.trajectory_point(t, jointspace)
            points.append(point)

        # We want to make a final point at the end of the trajectory so that the
        # controller has time to converge to the final point.
        extra_point = self.trajectory_point(self.total_time, jointspace)
        extra_point.time_from_start = rospy.Duration.from_sec(self.total_time + 1)
        points.append(extra_point)

        traj.points = points
        traj.header.frame_id = 'base'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def trajectory_point(self, t, jointspace):
        """
        takes a discrete point in time, and puts the position, velocity, and
        acceleration into a ROS JointTrajectoryPoint() to be put into a
        RobotTrajectory.

        Parameters
        ----------
        t : float
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.

        Returns
        -------
        :obj:`trajectory_msgs.msg.JointTrajectoryPoint`
        """
        point = JointTrajectoryPoint()
        delta_t = .01
        if jointspace:
            x_t, x_t_1, x_t_2 = None, None, None
            ik_attempts = 0
            theta_t_2 = self.get_ik(self.target_position(t-2*delta_t))
            theta_t_1 = self.get_ik(self.target_position(t-delta_t))
            theta_t   = self.get_ik(self.target_position(t))

            # we said you shouldn't simply take a finite difference when creating
            # the path, why do you think we're doing that here?
            point.positions = theta_t
            point.velocities = (theta_t - theta_t_1) / delta_t
            point.accelerations = (theta_t - 2*theta_t_1 + theta_t_2) / (2*delta_t)
        else:
            point.positions = self.target_position(t)
            point.velocities = self.target_velocity(t)
            point.accelerations = self.target_acceleration(t)
        point.time_from_start = rospy.Duration.from_sec(t)
        return point

    def get_ik(self, x, max_ik_attempts=10):
        """
        gets ik

        Parameters
        ----------
        x : 3x' :obj:`numpy.ndarray`
            workspace position of the end effector
        max_ik_attempts : int
            number of attempts before short circuiting

        Returns
        -------
        7x' :obj:`numpy.ndarray`
            joint values to achieve the passed in workspace position
        """
        ik_attempts, theta = 0, None
        while theta is None and not rospy.is_shutdown():
            theta = self.kin.inverse_kinematics(
                position=x,
                orientation=[0, 1, 0, 0]
            )
            ik_attempts += 1
            if ik_attempts > max_ik_attempts:
                rospy.signal_shutdown(
                    'MAX IK ATTEMPTS EXCEEDED AT x(t)={}'.format(x)
                )
        return theta

class LinearPath(MotionPath):
    def __init__(self, limb, kin, tag_pos, total_time, current_position):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin, total_time)
        self.goal = tag_pos
        self.current_position = current_position
        self.distance = self.goal -self.current_position
        print(self.distance)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired x,y,z position in workspace coordinates of the end effector
        """
        if time <= self.total_time/2.0:
            distance= 1/2.0 * self.target_acceleration(time) * (time ** 2) +  self.target_velocity(time) * time
            return distance + self.current_position

        else:
        	from_half = time - self.total_time/2.0 
        	d_halfway = 1/2.0 * self.target_acceleration(self.total_time/2.0) * (self.total_time/2.0 ** 2) + self.target_velocity(self.total_time/2.0) * self.total_time/2.0
        	d_remain =  1/2.0 * self.target_acceleration(time) * (from_half ** 2)  
        	distance = d_halfway + d_remain
        	return distance + self.current_position

    def target_velocity(self, time):
        """
        Returns the arm's desired x,y,z velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        if time <= self.total_time/2.0:
            velocity = time * self.target_acceleration(time)
        else:
            velocity = self.target_velocity(self.total_time/2.0) + (time-self.total_time/2.0) * self.target_acceleration(time)
        return velocity

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`

            desired acceleration in workspace coordinates of the end effector
        """
        acceleration = (self.distance * 4) / (self.total_time**2);
        if time <= self.total_time/2.0:
            return acceleration
        else:
            return -1 * acceleration

class CircularPath(MotionPath):
    def __init__(self, limb, kin, tag_pos, total_time, current_position):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin, total_time)
        #TODO: figure out how to get this stuff
        self.current_position = current_position
        self.goal = tag_pos
        self.radius = utils.length(self.current_position - self.goal)
        self.circumference = 2 * math.pi * self.radius
        self.theta_0 = np.arctan((self.current_position[0]-self.goal[0])/(self.current_position[1]-self.goal[1]))
        print(self.theta_0)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z position in workspace coordinates of the end effector
        """
        if time < self.total_time/2.0:
            angular_acceleration = (math.pi * 8) /(self.total_time**2);
        else:
            angular_acceleration = (- 1* math.pi * 8) /(self.total_time**2);

        theta = 1/2.0 * angular_acceleration * time **2 - self.theta_0
        return np.array([self.radius * np.cos(theta), self.radius * np.sin(theta), 0]) - self.goal

    def target_velocity(self, time):
        """
        Returns the arm's desired velocity in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_position()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired x,y,z velocity in workspace coordinates of the end effector
        """

        if time < self.total_time/2.0:
            angular_acceleration = math.pi * 8 /self.total_time**2;
        else:
            angular_acceleration = - 1* math.pi * 8 /self.total_time**2;
        theta = 1/2.0 * angular_acceleration * time **2 - self.theta_0
        omega = angular_acceleration * time
        velocity = np.array([-self.radius * omega * np.sin(theta), self.radius * omega * np.cos(theta), 0])

        return velocity

    def target_acceleration(self, time):
        """
        Returns the arm's desired x,y,z acceleration in workspace coordinates
        at time t.  You should NOT simply take a finite difference of
        self.target_velocity()

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
           desired acceleration in workspace coordinates of the end effector
        """
        from_half = time - self.total_time/2.0 
        if time <= self.total_time/2.0:
        	angular_acceleration = (math.pi * 8) /(self.total_time**2)
        	omega = angular_acceleration * time
        	theta = 1/2.0 * angular_acceleration * time **2 + omega*time - self.theta_0

        else:
        	angular_acceleration = (math.pi * -8) /(self.total_time**2)
       		omega = angular_acceleration * from_half - angular_acceleration * time
       		
       		theta = (1/2.0 * (1) * angular_acceleration * from_half **2) + 1/2.0 * angular_acceleration * time **2 + omega*time - self.theta_0



        acceleration = np.array([-self.radius * (omega **2) * np.cos(theta),-self.radius * (omega **2) * np.sin(theta), angular_acceleration])
        return acceleration

class MultiplePaths(MotionPath):
    """
    Remember to call the constructor of MotionPath

    You can implement multiple paths a couple ways.  The way I chose when I took
    the class was to create several different paths and pass those into the
    MultiplePaths object, which would determine when to go onto the next path.
    """
    def __init__(self, paths, limb, kin, total_time, current_position):
        MultiplePaths.__init__(self, limb, kin, total_time)
        #TODO: figure out how to get this stuff
        self.numpaths = len(paths)
        if self.num_paths != 4:
            return
        paths = paths.sort(key=lambda a: a[0])
        if (paths[0][1] > paths[1][1]):
            temp = paths[0]
            paths[0] = paths[1]
            paths[1] = temp
        if (paths[2][1] < paths[3][1]):
            temp = paths[2]
            paths[2] = paths[3]
            paths[3] = temp
        self.paths = paths
        self.trajectories = []
        self.trajectories[0] = LinearPath(limb, kin, total_time/num_paths, paths[0], current_position)
        self.trajectories[1] = LinearPath(limb, kin, total_time/num_paths, paths[1], paths[0])
        self.trajectories[2] = LinearPath(limb, kin, total_time/num_paths, paths[2], paths[1])
        self.trajectories[3] = LinearPath(limb, kin, total_time/num_paths, paths[3], paths[2])




    def get_current_path(self, time):
        return (int)(time/self.total_time * self.num_paths)

    def target_position(self, time):
        """
        Returns where the arm end effector should be at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired position in workspace coordinates of the end effector
        """
        current_path = get_current_path(time)
        return self.trajectories[current_path].target_position(time - current_path * total_time/num_paths)

    def target_velocity(self, time):
        """
        Returns the arm's desired velocity in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired velocity in workspace coordinates of the end effector
        """
        current_path = get_current_path(time)
        return self.trajectories[current_path].target_velocity(time - current_path * total_time/num_paths)

    def target_acceleration(self, time):
        """
        Returns the arm's desired acceleration in workspace coordinates
        at time t

        Parameters
        ----------
        time : float

        Returns
        -------
        3x' :obj:`numpy.ndarray`
            desired acceleration in workspace coordinates of the end effector
        """
        current_path = get_current_path(time)
        return self.trajectories[current_path].target_acceleration(time - current_path * total_time/num_paths)
