#!/usr/bin/env python
"""
Starter script for lab1.
Author: Chris Correa
"""
import copy
import sys
import argparse
import time
import numpy as np
import signal

from paths.paths import LinearPath, CircularPath, MultiplePaths
from controllers.controllers import (
    PDWorkspaceVelocityController,
    PDJointVelocityController,
    PDJointTorqueController,
    FeedforwardJointVelocityController,
    JointspaceImpedanceController,
    WorkspaceImpedanceController)
from utils.utils import *
from path_planner import PathPlanner
from baxter_pykdl import baxter_kinematics

try:
    import rospy
    import tf
    import baxter_interface
    import moveit_commander
    from moveit_msgs.msg import DisplayTrajectory, RobotState
except:
    print 'Couldn\'t import ROS, I assume you\'re working on just the paths on your own computer'

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position

    """
    return [np.array([0.6, 0.4, 0.11])]
    # return [np.array([0.6, 0.3, 0.16881026]), np.array([0.5, 0.3, 0.16881026]), np.array([0.5, 0.2, 0.16881026]), np.array([0.6, 0.2, 0.16881026])]
    # listener = tf.TransformListener()
    # rospy.sleep(1)
    # from_frame = 'base'
    # to_frame = 'ar_marker_{}'.format(tag_number)

    # r = rospy.Rate(200)
    # while (
    #     not listener.frameExists(from_frame) or not listener.frameExists(to_frame) 
    #     # not listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(0.1))
    # ) and (
    #     not rospy.is_shutdown()
    # ):
    #     print 'Cannot find AR marker {}, retrying'.format(tag_number)
    #     r.sleep()

    # t = listener.getLatestCommonTime(from_frame, to_frame)
    # tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
    # return vec(tag_pos)

def get_trajectory(task, tag_pos, num_way, controller_name):
    """
    Returns an appropriate robot trajectory for the specified task.  You should
    be implementing the path functions in paths.py and call them here

    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`

    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    #TODO: for linear path, set goal height to initial height, for multiple paths, set goal height to x above the height

    #TODO part a
    current_position = kin.forward_position_kinematics()[:3];
    print("Current Position", current_position)
    target_pos = tag_pos[0]
    print("target position", tag_pos)
    if task == 'line':
        target_pos[0][2] = current_position[2]; #linear path moves to a Z position above AR Tag.
        path = LinearPath(limb, kin, target_pos[0], current_position)
    elif task == 'circle':
        target_pos[0][2] = current_position[2]; #linear path moves to a Z position above AR Tag.
        path = CircularPath(limb, kin, target_pos[0], current_position)
    elif task == 'square':
        # for tag in target_pos:
            # tag[2] = tag[2] + 0.3  #have a goal position slightly above the AR Tag so we don't hit anything
        path = MultiplePaths(limb, kin, target_pos, current_position)
    else:
        raise ValueError('task {} not recognized'.format(task))
    path.plot()
    return path.to_robot_trajectory(num_way, controller_name!='workspace' and controller_name != 'workspace_impedance')

def get_controller(controller_name):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'workspace':
        # YOUR CODE HERE
        Kp = 5 * np.array([2,1,1,0.01,0.01, 0.01])
        Kv = 0.3 * np.array([1,1, 1, 1,1,1])
        controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'jointspace':
        # YOUR CODE HERE
        Kp = 3* np.array([1, 1,1,1,1,1, 1])
        Kv = 0.0001 * np.array([1,1, 1, 1,1,1,1])
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    elif controller_name == 'torque':
        # YOUR CODE HERE
        Kp = 8 * np.array([1, 1,1,1,0.5,0.5, 0.3])
        Kv = 3 * np.array([1.5, 1.5,1,1,0.3,0.3, 0.3])
        controller = PDJointTorqueController(limb, kin, Kp, Kv)
    elif controller_name == 'workspace_impedance':
        # YOUR CODE HERE
        Md = 0.01 * np.array([1, 1,1,1,1,1])
        Bd = 0.01 * np.array([1, 1,1,1,1,1])
        Kd = 0.01 * np.array([1, 1,1,1,1,1])

        controller = WorkspaceImpedanceController(limb, kin, Md, Bd, Kd)
    elif controller_name == 'jointspace_impedance':
        # YOUR CODE HERE
        Kd = 8 * np.array([1, 1,1,1,60,50, 50])
        Bd = 3 * np.array([1.5, 1.5,1,1,20,20, 20])
        Md = 1 * np.array([1, 1,1,1,1,1,1])
        # Kd = 0 * np.array([1, 1,1,1,1,0,0])
        controller = JointspaceImpedanceController(limb, kin, Md, Bd, Kd)

    elif controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

if __name__ == "__main__":
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t 1 -ar 1 -c workspace -a left --log
    python scripts/main.py -t 2 -ar 2 -c velocity -a left --log
    python scripts/main.py -t 3 -ar 3 -c torque -a right --log
    python scripts/main.py -t 1 -ar 4 5 --path_only --log
    
    python scripts/main.py -t 1 -ar 5 --log
    python scripts/main.py -t 1 -ar 5 --log

    python scripts/main.py -t circle -c jointspace -ar 5 --log
    python scripts/main.py -t line -c jointspace -ar 5 --log
    python scripts/main.py -t line -c torque -ar 5 --log

    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, square.  Default: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='workspace',
        help='Options: workspace, jointspace, or torque.  Default: workspace'
    )
    parser.add_argument('-arm', '-a', type=str, default='left', help=
        'Options: left, right.  Default: left'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=300, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--moveit', action='store_true', help=
        """If you set this flag, moveit will take the path you plan and execute it on
        the real robot"""
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()




    rospy.init_node('moveit_node')
    # this is used for sending commands (velocity, torque, etc) to the robot
    limb = baxter_interface.Limb(args.arm)
    # this is used to get the dynamics (inertia matrix, manipulator jacobian, etc) from the robot
    # in the current position, UNLESS you specify other joint angles.  see the source code
    # https://github.com/valmik/baxter_pykdl/blob/master/src/baxter_pykdl/baxter_pykdl.py
    # for info on how to use each method
    kin = baxter_kinematics(args.arm)

    # ADD COMMENT HERE
    tag_pos = [lookup_tag(marker) for marker in args.ar_marker]
    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(args.task, tag_pos, args.num_way, args.controller_name)
    # print(robot_trajectory)
    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    # print(robot_trajectory)
    planner = PathPlanner('{}_arm'.format(args.arm))
    if args.controller_name == "workspace":
        pose = create_pose_stamped_from_pos_quat(
            robot_trajectory.joint_trajectory.points[0].positions,
            [0, 1, 0, 0],
            'base'
        )
        plan = planner.plan_to_pose(pose)
    else:
        plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    planner.execute_plan(plan)

    if args.moveit:
        # LAB 1 PART A
        # by publishing the trajectory to the move_group/display_planned_path topic, you should
        # be able to view it in RViz.  You will have to click the "loop animation" setting in
        # the planned path section of MoveIt! in the menu on the left side of the screen.
        pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory.append(robot_trajectory)
        # disp_traj.trajectory_start = planner._group.get_current_joint_values()
        disp_traj.trajectory_start = RobotState()
        pub.publish(disp_traj)

        try:
            raw_input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # uses MoveIt! to execute the trajectory.  make sure to view it in RViz before running this.
        # the lines above will display the trajectory in RViz
        planner.execute_plan(robot_trajectory)
    else:
        # LAB 1 PART B
        controller = get_controller(args.controller_name)
        try:
            raw_input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory,
            rate=args.rate,
            timeout=args.timeout,
            log=args.log
        )
        # done = controller.follow_ar_tag(robot_trajectory, args.ar_marker[0], rate = args.rate, timeout = args.timeout, log = args.log)
        if not done:
            print 'Failed to move to position'
            sys.exit(0)
