#!/usr/bin/env python

"""
Starter script for lab1.
Author: Chris Correa
"""
import numpy as np
import math
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt


try:
    import utils.utils as utils
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
    def get_total_time(self):
        return self.total_time
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

        plt.subplot(3,3,3)
        plt.plot(times, target_acceleration[:,0], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("x acceleration")

        plt.subplot(3,3,4)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,3,5)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")

        plt.subplot(3,3,6)
        plt.plot(times, target_acceleration[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("y acceleration")

        plt.subplot(3,3,7)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,3,8)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.subplot(3,3,9)
        plt.plot(times, target_acceleration[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("z acceleration")

        plt.pause(0.1)
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
            # print(theta_t, theta_t_1)
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
    def __init__(self, limb, kin, tag_pos, current_position):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin, 0)
        self.goal = tag_pos
        self.current_position = current_position
        self.distance = self.goal - self.current_position
        radius = np.linalg.norm(self.current_position - self.goal,ord = 2)

        self.total_time = radius * 12


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
        # return self.current_position

        if time <= self.total_time / 2.0:
            distance = 1 / 2.0 * self.target_acceleration(time) * (time ** 2)
            return distance + self.current_position
        else:
            half_time = self.total_time / 2.0
            after_half_time = time - half_time

            d_halfway = self.target_position(self.total_time / 2.0)
            v_halfway = self.target_velocity(self.total_time / 2.0)

            d_remain = 1 / 2.0 * self.target_acceleration(time) * (after_half_time ** 2) + v_halfway * after_half_time
            distance = d_halfway + d_remain
            return distance


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
        # return np.array([0,0,0])
        if time <= self.total_time / 2.0:
            velocity = time * self.target_acceleration(time)
        else:
            velocity = self.target_velocity(self.total_time / 2.0) + (
                        time - self.total_time / 2.0) * self.target_acceleration(time)
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
        # return np.array([0,0,0])

        acceleration = (self.distance * 4.0) / (self.total_time * self.total_time);
        if time <= self.total_time / 2.0:
            return acceleration
        else:
            return -1 * acceleration

class CircularPath(MotionPath):
    def __init__(self, limb, kin, tag_pos, current_position):
        """
        Remember to call the constructor of MotionPath

        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        MotionPath.__init__(self, limb, kin,0)
        #TODO: figure out how to get this stuff
        self.current_position = current_position
        # print("current", self.current_position)
        self.goal = tag_pos
        # print("distance", self.current_position - self.goal)

        self.radius = np.linalg.norm(self.current_position - self.goal,ord = 2)
        self.circumference = 2 * math.pi * self.radius
        self.theta_0 =  np.pi - np.arctan2((self.current_position[1] - self.goal[1]),(self.goal[0]-self.current_position[0]))
        self.total_time = self.circumference * 10
        # print("radius", self.radius)
        # print("theta 0" , self.theta_0)

    def angular_kinematics(self, time):

        if time <= self.total_time / 2.0:
            alpha = (math.pi * 8) / (self.total_time ** 2)
            omega = alpha * time
            theta = 1 / 2.0 * alpha * time ** 2
        else:
            half_time = self.total_time / 2.0
            after_half_time = time - half_time

            alpha_0 = (math.pi * 8) / (self.total_time ** 2)
            omega_0 = alpha_0 * half_time
            theta_0 = 1 / 2.0 * alpha_0 * half_time ** 2

            alpha = (-1) * alpha_0
            omega = alpha * after_half_time + omega_0

            theta_remain = 1 / 2.0 * alpha * after_half_time ** 2 + omega_0 * after_half_time
            theta = theta_0 + theta_remain

        return np.array([alpha, omega, theta])

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
        alpha, omega, theta = self.angular_kinematics(time)
        theta =  theta + self.theta_0
        position = self.radius * np.array([ np.cos(theta),np.sin(theta), 0]) + self.goal
        # position[0] = -1 * position[0]
        # position[1] = -1 * position[1]
        return position
        # return self.angular_kinematics(time)
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

        alpha, omega, theta = self.angular_kinematics(time)
        theta = theta + self.theta_0
        velocity = self.radius * omega * np.array([-np.sin(theta), np.cos(theta), 0])
        # velocity[0] = -1 * velocity[0]
        # velocity[1] = -1 * velocity[1]


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

        alpha, omega, theta = self.angular_kinematics(time)
        theta = theta + self.theta_0
        acceleration =  self.radius *  np.array([-alpha * np.sin(theta) - omega **2 * np.cos(theta), alpha * np.cos(theta) - omega **2 * np.sin(theta), 0])
        return acceleration

    def plot(self, num=300):
        MotionPath.plot(self)
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t) for t in times])
        target_velocities = np.vstack([self.target_velocity(t) for t in times])
        target_acceleration = np.vstack([self.target_acceleration(t) for t in times])

        plt.plot(target_positions[:,0], target_positions[:,1], label='Desired')
        plt.scatter(target_positions[0,0], target_positions[0,1], marker='o')
        plt.scatter(self.goal[0], self.goal[1], marker='*')
        plt.scatter(self.current_position[0], self.current_position[1], marker='s')

        plt.xlabel("x")
        plt.ylabel("y")
        plt.pause(0.1)

        plt.show()

class MultiplePaths(MotionPath):
    """
    Remember to call the constructor of MotionPath

    You can implement multiple paths a couple ways.  The way I chose when I took
    the class was to create several different paths and pass those into the
    MultiplePaths object, which would determine when to go onto the next path.
    """
    def __init__(self, limb, kin, paths, current_position):
        MotionPath.__init__(self, limb, kin, total_time = 0)
        #TODO: figure out how to get this stuff
        self.numpaths = len(paths)+1
        self.paths = paths
        print("current_position", current_position)
        print("path", self.paths)
        self.trajectories = []
        self.trajectories.append(LinearPath(limb, kin, paths[0], current_position))
        self.trajectories.append(LinearPath(limb, kin, paths[1], paths[0]))
        self.trajectories.append(LinearPath(limb, kin, paths[2], paths[1]))
        self.trajectories.append(LinearPath(limb, kin, current_position, paths[2]))
        self.total_time = sum(t.total_time for t in self.trajectories)
        print(self.total_time)
        self.path_times = [t.total_time for t in self.trajectories]
        print(self.path_times)

    def get_current_path(self, time):
        curpath = 0
        i = 0
        while(i < time):
            i += self.path_times[curpath]
            curpath+=1
        curpath -=1
        if curpath > 0:
            time_on_path = time - sum(self.path_times[:curpath])
        else:
            time_on_path = time
        return curpath, time_on_path

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
        current_path, time_on_path = self.get_current_path(time)
        # return np.array([current_path, time_on_path, 0])
        return self.trajectories[current_path].target_position(time_on_path)

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
        current_path, time_on_path = self.get_current_path(time)
        return self.trajectories[current_path].target_velocity(time_on_path)

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
        current_path, time_on_path = self.get_current_path(time)
        return self.trajectories[current_path].target_acceleration(time_on_path)

    def plot(self, num=300):
        times = np.linspace(0, self.total_time, num=num)
        target_positions = np.vstack([self.target_position(t) for t in times])
        target_velocities = np.vstack([self.target_velocity(t) for t in times])
        target_acceleration = np.vstack([self.target_acceleration(t) for t in times])
        current_path =[self.get_current_path(t)[0] for t in times]
        current_time =[self.get_current_path(t)[1] for t in times]


        plt.figure()
        plt.subplot(3,4,1)
        plt.plot(times, target_positions[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Position")

        plt.subplot(3,4,2)
        plt.plot(times, target_velocities[:,0], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("X Velocity")

        plt.subplot(3,4,3)
        plt.plot(times, target_acceleration[:,0], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("x acceleration")

        plt.subplot(3,4,4)
        plt.plot(times, target_positions[:,1], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Y Position")

        plt.subplot(3,4,5)
        plt.plot(times, target_velocities[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Y Velocity")

        plt.subplot(3,4,6)
        plt.plot(times, target_acceleration[:,1], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("y acceleration")

        plt.subplot(3,4,7)
        plt.plot(times, target_positions[:,2], label='Desired')
        plt.xlabel("time (t)")
        plt.ylabel("Z Position")

        plt.subplot(3,4,8)
        plt.plot(times, target_velocities[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("Z Velocity")

        plt.subplot(3,4,9)
        plt.plot(times, target_acceleration[:,2], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("z acceleration")

        plt.subplot(3,4,10)
        plt.plot(times, current_path[:], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("current_path")
        plt.subplot(3,4,11)
        plt.plot(times, current_time[:], label='Desired')
        plt.xlabel("Time (t)")
        plt.ylabel("current time")

        plt.pause(0.1)

        plt.show()





# if __name__ == "__main__":
#     target_pos = [np.array([0, 0, 3]),np.array([0, 4, 3]),np.array([4, 0, 3]),np.array([-0.3, 0.3, 3])]
#     total_time = 10
#     current_position = np.array([0.53, 0.54, 3])
#     # target_pos = np.array([0.4, 0.54, 3])
#
#     # path = LinearPath(None, None, target_pos, current_position)
#     # path = CircularPath(None, None, target_pos, current_position)
#     path = MultiplePaths(None, None, target_pos, current_position)
#
#     path.plot()
