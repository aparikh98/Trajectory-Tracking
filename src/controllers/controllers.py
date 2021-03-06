#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa, Valmik Prabhu
"""

# Python imports
import sys
import numpy as np
import itertools
import matplotlib 
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
# Lab imports
from utils.utils import *
from paths.paths import LinearPath, CircularPath, MultiplePaths

# ROS imports
try:
    import tf
    import rospy
    import baxter_interface
    import intera_interface
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
except:
    pass

NUM_JOINTS = 7

class Controller:

    def __init__(self, limb, kin):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        """

        # Run the shutdown function when the ros node is shutdown
        rospy.on_shutdown(self.shutdown)
        self._limb = limb
        self._kin = kin

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly.  

        Parameters
        ----------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        """
        pass

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        current_index : int
            waypoint index at which search was terminated 
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)


    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.stop_moving()
        rospy.sleep(0.1)

    def stop_moving(self):
        """
        Set robot joint velocities to zero
        """
        zero_vel_dict = joint_array_to_dict(np.zeros(NUM_JOINTS), self._limb)
        self._limb.set_joint_velocities(zero_vel_dict)



    def plot_results(
        self,
        times,
        actual_positions, 
        actual_velocities, 
        target_positions, 
        target_velocities,
        # actual_positions_workspace,
        # actual_velocities_workspace,
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        # actual_positions_workspace = np.array(actual_positions_workspace)
        # actual_velocities_workspace = np.array(actual_velocities_workspace)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)
        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            actual_workspace_positions[i] = \
                self._kin.forward_position_kinematics(positions_dict)[:3]
            actual_workspace_velocities[i] = \
                self._kin.jacobian()[:3].dot(actual_velocities[i])
        # check if joint space
        if target_positions.shape[1] > 3:
            # it's joint space

            target_workspace_positions = np.zeros((len(times), 3))
            target_workspace_velocities = np.zeros((len(times), 3))

            for i in range(len(times)):
                positions_dict = joint_array_to_dict(target_positions[i], self._limb)
                target_workspace_positions[i] = \
                    self._kin.forward_position_kinematics()[:3]
                target_workspace_velocities[i] = \
                    self._kin.jacobian()[:3].dot(target_velocities[i])

            # Plot joint space
            plt.figure()
            # print len(times), actual_positions.shape()
            jointnames = self._limb.joint_names()
            joint_num = len(self._limb.joint_names())
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + jointnames[joint] + " Position Error")

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Velocity Error")

            print "Close the plot window to continue"
            plt.pause(0.1)      
            plt.show()

        else:
            # it's workspace
            target_workspace_positions = target_positions
            target_workspace_velocities = target_velocities

        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_workspace_velocities[:,joint], label='Actual')
            plt.plot(times, target_workspace_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")

        print "Close the plot window to continue"
        plt.pause(0.1)

        plt.show()




    def execute_path(self, path, rate=200, timeout=None, log=False):
        """
        takes in a path and moves the baxter in order to follow the path.  

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()
            # actual_positions_workspace = list()
            # actual_velocities_workspace = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)
            # current_position_workspace = self._limb.forward_position_kinematics()
            # current_velocity_workspace = self._limb.forward_velocity_kinematics()
            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, t, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)
                # actual_positions_workspace.append(current_position_workspace)
                # actual_velocities_workspace.append(current_velocity_workspace)


            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break

        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities,
                # actual_positions_workspace,
                # actual_velocities_workspace
            )
        return True
    # def lookup_tag(self, tag_number, listener):
    #     """
    #     Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    #     You can use either this function or try starting the scripts/tag_pub.py script.  More info
    #     about that script is in that file.

    #     Parameters
    #     ----------
    #     tag_number : int

    #     Returns
    #     -------
    #     3x' :obj:`numpy.ndarray`
    #         tag position

    #     """
    #     from_frame = 'base'
    #     to_frame = 'ar_marker_{}'.format(tag_number)

    #     r = rospy.Rate(200)
    #     while (
    #         not listener.frameExists(from_frame) or not listener.frameExists(to_frame) 
    #         # not listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(0.1))
    #     ) and (
    #         not rospy.is_shutdown()
    #     ):
    #         print 'Cannot find AR marker {}, retrying'.format(tag_number)
    #         r.sleep()

    #     t = listener.getLatestCommonTime(from_frame, to_frame)
    #     tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
    #     print("GETTING AR TAG", tag_pos)
    #     return tag_pos

    def lookup_tag(self, tag_number, listener):
        from_frame = 'base'
        to_frame = 'ar_marker_{}'.format(tag_number)
        if (
            listener.frameExists(from_frame) or not listener.frameExists(to_frame) and not rospy.is_shutdown()):
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
            print("GETTING AR TAG", tag_pos)
            return tag_pos
        else:
            print 'Cannot find AR marker {}, retrying'.format(tag_number)
            return None




    def follow_ar_tag(self, path, tag, rate=200, timeout=None, log= True):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You 
        should look at execute_path() for inspiration on how to write this. 

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """
        timeout = 20
        #from execute_path
        listener = tf.TransformListener()
        rospy.sleep(1)
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = 10000
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)
        latest_path = self.lookup_tag(tag, listener)
        start_2 = start_t

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()
            t2 = (rospy.Time.now() - start_2).to_sec()
            # If the controller has timed out, stop moving and return false
            if timeout is not None and t < timeout:
                # Set velocities to zero
                # self.stop_moving()
                # return False
                if (t2 > 4):
                    new_path = self.lookup_tag(tag, listener)
                    if (new_path != None):
                        current_position_workspace = self._kin.forward_position_kinematics()[:3]
                        new_path[2] = current_position_workspace[2]
                        current_velocity_workspace = np.array(np.matmul(self._kin.jacobian(), get_joint_velocities(self._limb))).reshape(-1)[:3]
                        #for workspace false, js true
                        path = LinearPath(self._limb, self._kin, new_path,  current_position_workspace).to_robot_trajectory(300, False)
                        if (path != None):
                            current_index = 0
                            start_2 = (rospy.Time.now())
                            latest_path = new_path
                            print ("NEW PATH FOUND!!!!!!\n\n\n")
            else:
                max_index = len(path.joint_trajectory.points)-1
            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, t2, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break

        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True

class FeedforwardJointVelocityController(Controller):
    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        self._limb.set_joint_velocities(joint_array_to_dict(target_velocity, self._limb))

class PDWorkspaceVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller compares the baxter's current WORKSPACE position and
    velocity desired WORKSPACE position and velocity to come up with a WORKSPACE velocity command to be sent
    to the baxter.  Then this controller should convert that WORKSPACE velocity command into a joint velocity
    command and sends that to the baxter.  Notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._kin.forward_psition_kinematics() and self._kin.forward_velocity_kinematics() to get 
        the current workspace position and velocity and self._limb.set_joint_velocities() to set 
        the joint velocity to something.  you may have to look at 
        http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html to convert the 
        output of forward_velocity_kinematics() to a numpy array.  You may find joint_array_to_dict() 
        in utils.py useful 

        MAKE SURE TO CONVERT QUATERNIONS TO EULER IN forward_position_kinematics().  
        you can use tf.transformations.euler_from_quaternion()

        your target orientation should be (0,0,0) in euler angles and (0,1,0,0) as a quaternion.  

        Parameters
        ----------
        target_position: 6x' ndarray of desired positions
        target_velocity: 6x' ndarray of desired velocities
        target_acceleration: 6x' ndarray of desired accelerations
        """
        current_position = self._kin.forward_position_kinematics()[:3]
        current_orientation = np.asarray(tf.transformations.euler_from_quaternion(self._kin.forward_position_kinematics()[3:]))
        current_position = np.concatenate((current_position,current_orientation)).reshape(6,1)
        target_position = np.concatenate((target_position, np.array([np.pi,0,np.pi]))).reshape(6,1)
        current_velocity = np.array(np.matmul(self._kin.jacobian(), get_joint_velocities(self._limb))).reshape(6,1)
        target_velocity = np.concatenate((target_velocity, np.array([0,0,0]))).reshape(6,1)
       
        e = current_position- target_position 
        d_e = current_velocity- target_velocity 
        
        d_x = target_velocity - np.matmul(self.Kp, e) - np.matmul(self.Kv, d_e)
        d_q = np.array(np.matmul(self._kin.jacobian_pseudo_inverse(), d_x)).reshape(7,1)
        
        self._limb.set_joint_velocities(joint_array_to_dict(d_q, self._limb))

class PDJointVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the 
    PDJointVelocityController is that this controller turns the desired workspace position and velocity
    into desired JOINT position and velocity.  Then it compares the difference between the baxter's 
    current JOINT position and velocity and desired JOINT position and velocity to come up with a
    joint velocity command and sends that to the baxter.  notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        current_position = get_joint_positions(self._limb)
        current_velocity = get_joint_velocities(self._limb)
        e = current_position - target_position 
        d_e = current_velocity- target_velocity 
        
        d_x = target_velocity - np.matmul(self.Kp, e) - np.matmul(self.Kv, d_e)
        self._limb.set_joint_velocities(joint_array_to_dict(d_x, self._limb))

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly. This method should call
        self._limb.joint_angle and self._limb.joint_velocity to get the current joint position and velocity
        and self._limb.set_joint_velocities() to set the joint velocity to something.  You may find
        joint_array_to_dict() in utils.py useful

        Look in section 4.5 of MLS.

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        target_position = target_position.reshape((7,1))
        target_velocity = target_velocity.reshape((7,1))
        target_acceleration = target_acceleration.reshape((7,1))    
        current_position = get_joint_positions(self._limb).reshape((7,1))
        current_velocity = get_joint_velocities(self._limb).reshape((7,1))
        
        e = current_position - target_position 
        d_e = current_velocity- target_velocity
        
        positions_dict = joint_array_to_dict(current_position, self._limb)
        velocity_dict = joint_array_to_dict(current_velocity, self._limb)

        inertia = self._kin.inertia(positions_dict)
        coriolis = self._kin.coriolis(positions_dict, velocity_dict)[0][0]
        coriolis = np.array([float(coriolis[i]) for i in range(7)]).reshape((7,1))

        gravity_accel = np.array([0,0,.981,0,0,0]).reshape((6,1))
        gravity_jointspace = (np.matmul(self._kin.jacobian_transpose(positions_dict), gravity_accel))
        gravity = (np.matmul(inertia, gravity_jointspace)).reshape((7,1))
        
        torque = np.array(np.matmul(inertia, target_acceleration) +\
            coriolis + gravity + (-np.matmul(self.Kv,d_e).reshape((7,1)) - np.matmul(self.Kp,e).reshape((7,1))))
        torque_dict = joint_array_to_dict(torque, self._limb)
        self._limb.set_joint_torques(torque_dict)







######################
# GRAD STUDENTS ONLY #
######################

class WorkspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Bd, Kd):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Bd = np.diag(Bd)
        self.Kd = np.diag(Kd)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """

        Parameters
        ----------
        target_position: 6x' ndarray of desired positions
        target_velocity: 6x' ndarray of desired velocities
        target_acceleration: 6x' ndarray of desired accelerations
        """

        current_position = self._kin.forward_position_kinematics()[:3]
        current_orientation = np.asarray(tf.transformations.euler_from_quaternion(self._kin.forward_position_kinematics()[3:]))
        current_position = np.concatenate((current_position,current_orientation)).reshape((6,1))
        target_position = np.concatenate((target_position, np.array([np.pi,0,np.pi]))).reshape((6,1))

        current_velocity = np.array(np.matmul(self._kin.jacobian(), get_joint_velocities(self._limb))).reshape((6,1))
        target_velocity = np.concatenate((target_velocity, np.array([0,0,0]))).reshape(6,1)

        target_acceleration = np.concatenate((target_acceleration, np.array([0,0,0])))
        target_acceleration = target_acceleration.reshape((6,1))

        e = current_position - target_position 
        d_e = current_velocity- target_velocity

        current_position_js = get_joint_positions(self._limb)
        current_velocity_js = get_joint_velocities(self._limb)

        positions_dict = joint_array_to_dict(current_position_js, self._limb)
        velocity_dict = joint_array_to_dict(current_velocity_js, self._limb)

        inertia = self._kin.inertia(positions_dict)
        jt = self._kin.jacobian_transpose(positions_dict)

        gravity_accel = np.array([0,0,.981,0,0,0]).reshape((6,1))
        gravity_jointspace = (np.matmul(jt, gravity_accel))
        gravity = (np.matmul(inertia, gravity_jointspace)).reshape((7,1))

        feedforward = np.matmul(inertia, np.matmul(jt,  np.array(target_acceleration)).reshape((7,1)))

        feedback = np.matmul(jt, np.matmul(self.Bd, d_e).reshape((6,1)) +np.matmul(self.Kd, e).reshape((6,1))).reshape(7,1)

        torque =  gravity - feedback + feedforward
        torque = torque.reshape((7,1))

        torque_dict = joint_array_to_dict(torque, self._limb)
        self._limb.set_joint_torques(torque_dict)
        
class JointspaceImpedanceController(Controller):
    def __init__(self, limb, kin, Bd, Kd):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Bd = np.diag(Bd)
        self.Kd = np.diag(Kd)


    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """

        current_position = get_joint_positions(self._limb)
        current_velocity = get_joint_velocities(self._limb)

        e = current_position - target_position 
        d_e = current_velocity- target_velocity
        positions_dict = joint_array_to_dict(current_position, self._limb)
        velocity_dict = joint_array_to_dict(current_velocity, self._limb)
        
        inertia = self._kin.inertia(joint_array_to_dict(target_position, self._limb))
        Md = inertia   

        coriolis = self._kin.coriolis(positions_dict, velocity_dict)[0][0]
        coriolis = np.array([float(coriolis[i]) for i in range(7)]).reshape((7,1))

        gravity_accel = np.array([0,0,.981,0,0,0]).reshape((6,1))
        gravity_jointspace = (np.matmul(self._kin.jacobian_transpose(), gravity_accel))
        gravity = (np.matmul(inertia, gravity_jointspace))

        feedback =  np.matmul(inertia,  np.array(target_acceleration).reshape((7,1)) - np.matmul(np.linalg.inv(Md),np.matmul(self.Bd, d_e) +np.matmul(self.Kd, e)).reshape((7,1)))

        force = coriolis + gravity + feedback
        torque_dict = joint_array_to_dict(force, self._limb)
        self._limb.set_joint_torques(torque_dict)
