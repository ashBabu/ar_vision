#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2, JointState
import numpy as np
import rosbag
import cv2
import tf
import actionlib
# from read_points import *
from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from tf.transformations import quaternion_from_euler
import time, sys
from pprint import pprint as pp
import subprocess
import psutil
import signal
import os


def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print '%r  %2.2f ms (%2.2f fps)' % (method.__name__, (te - ts) * 1000, 1. / (te - ts))
        return result
    return timed


class Move():

    def __init__(self, *args):
        moveit_commander.roscpp_initialize(sys.argv)
        robot_state = moveit_commander.RobotCommander()
        # self.record_rosbag = False
        
        self.subs = list()
        # self.bag = rosbag.Bag('test.bag', 'w')
        # self.subs.append(rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.f_ext_callback))
        # self.subs.append(rospy.Subscriber('/joint_states', JointState, self.joint_states_callback))
        

        #robot_pose = moveit_commander.MoveGroupCommander("panda_arm")
        #robot_pose.set_end_effector_link("panda_EE")
        #print  robot_pose.get_current_pose().pose
        self.scene = moveit_commander.PlanningSceneInterface()
        self.setup_planner()
        # self.group.set_goal_position_tolerance(0.01)
        # self.group.set_planner_id("BiTRRTkConfigDefault")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        print "============ Waiting for RVIZ..."
        rospy.sleep(1)

    def setup_planner(self):
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        # self.group.set_workspace((-1, -0.5, 0, 1, 0.5, 2))  # specify the workspace bounding box (static scene)
        # self.group.set_workspace((-0.1,-0.05,0,0.1,0.05,0.2))

        # self.group.set_end_effector_link("panda_hand")    # planning wrt to panda_hand or link8
        self.group.set_max_velocity_scaling_factor(0.05)  # scaling down velocity
        self.group.set_max_acceleration_scaling_factor(0.05)  # scaling down velocity
        self.group.allow_replanning(True)
        self.group.set_num_planning_attempts(5)
        self.group.set_goal_position_tolerance(0.02)
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_planning_time(30)
        self.group.set_planner_id("BiTRRTkConfigDefault")
        # self.group.set_planner_id("FMTkConfigDefault")
        # self.group.set_planner_id("RRTConnectkConfigDefault")
        # self.group.set_planner_id("CHOMP")
        print "CURRENT JOINT VALUES:", self.group.get_current_joint_values()
        self.group.set_start_state_to_current_state()
        print "CURRENT JOINT VALUES:", self.group.get_current_joint_values()


    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    @timeit
    def go(self, joints):
        dir_save_bagfile = '/tmp/'
        rosbag_process = subprocess.Popen('rosbag record -j -o {} /franka_state_controller/F_ext /joint_states'.format("my_rosbag_prefis"), stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)
        self.group.set_joint_value_target(joints)
        self.group.go()
        self.terminate_ros_node('/record')

    # def f_ext_callback(self, msg):
        # if self.record_rosbag:
    #         print "SAVNG F_EX"
    #         self.bag.write('/franka_state_controller/F_ext', msg)

    # def joint_states_callback(self, msg):
        # if self.record_rosbag:
    #         print "SAVING JOINTS"
    #         self.bag.write('/joint_states', msg)

    def plan_execute(self, X, Y, Z, qx, qy, qz, qw):
        # qx = [quaternion[0]], qy = [quaternion[1]], qz = [quaternion[2]], qw = [quaternion[3]]
        # X = trans[0], Y = trans[1], Z = trans[2]

        pose_target = PoseStamped()
        pose_target.header.frame_id = self.group.get_planning_frame()

        pose_target.pose.orientation.x = qx  # was 0.924 for link8
        pose_target.pose.orientation.y = qy
        pose_target.pose.orientation.z = qz
        pose_target.pose.orientation.w = qw

        pose_target.pose.position.x = X
        pose_target.pose.position.y = Y
        pose_target.pose.position.z = Z

        pp(pose_target)

        # self.group.set_end_effector_link("panda_arm")      ## planning wrt to panda hand instead of link8
        self.group.set_pose_target(pose_target)
        # self.group.set_max_velocity_scaling_factor(0.05)     ## scaling down velocity
        # self.group.set_max_acceleration_scaling_factor(0.05)  # scaling down velocity
        # self.group.set_max_acceleration_scaling_factor     ## scaling down velocity

        plan = self.group.plan()

        print " Waiting while RVIZ displays plan..."
        rospy.sleep(1)

        # while True:
        #     text = raw_input("============ Press Y to execute and N to terminate")
        #     if text == "Y" or text == "y":
        #         break
        #     if text == "N" or text == "n":
        #         self.group.clear_pose_targets()
        #         self.group.stop()
        #         raise ValueError('User wanted me to quit :(')
        # print "Executing"
        self.group.execute(plan)

        return


if __name__ == "__main__":

    rospy.init_node('MoveUR5', anonymous=True)
    listener = tf.TransformListener()
    try:
        Run_Moveit = Move()
        # listener.waitForTransform('base_link', 'tool0', rospy.Time(0), rospy.Duration(2.0))
        # (trans, qrt) = listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
        while True:
            listener.waitForTransform('world', 'ar_marker_8', rospy.Time(0), rospy.Duration(10.0))
            (trans, qrt) = listener.lookupTransform('world', 'ar_marker_8', rospy.Time(0))
            print trans, '~~', qrt
            Run_Moveit.plan_execute(trans[0], trans[1], trans[2], qrt[0], qrt[1], qrt[2], qrt[3])
        # while True:
        #     pose = int(raw_input("enter 0: exit, 1: original pose and 2: goal pose, 3: joint_home, 4: joint_random, 5: To tomato").strip())

        #     if pose == 0:
        #         break
        #     elif pose == 1:
        #         Run_Moveit.plan_execute(trans[0]*0.1, trans[1]*0.1, trans[2], qrt[0], qrt[1], qrt[2], qrt[3])
        #     elif pose == 2:
        #         Run_Moveit.plan_execute(0.4, -0.10, 0.75, -0.528, 0.543, -0.478, 0.446)
        #         continue
    except rospy.ROSInterruptException:
        print("program interupted before completion")