#!/usr/bin/env python
# coding=utf-8
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

from math import sin, cos, sqrt, fabs, pi
import time

from parameter import *

class CommandBase:
    def __init__(self):
        self.ego_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.px4_pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=10)
        self.odom_sub = rospy.Subscriber("/vins_fusion/imu_propagate", Odometry, self.odom_cb, queue_size=1)

        self.odom = Odometry()

        rospy.loginfo("waiting for odom...")
        cur_stamp = rospy.Time.now()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.odom.header.stamp >= cur_stamp:
                break
            rate.sleep()
        rospy.loginfo("got odom!")

        time.sleep(2)
    
    def odom_cb(self, msg):
        self.odom = msg

    def wrap_pi(self, val):
        res = val
        while True:
            if res < -pi:
                res = res + 2 * pi
            elif res > pi:
                res = res - 2 * pi
            else:
                return res
            
    def get_current_odom(self):
        current_stamp = rospy.Time.now()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.odom.header.stamp >= current_stamp:
                return self.odom
            rate.sleep()

    def wait_for_target(self, id, x, y, z, yaw, xy_thr, z_thr, yaw_thr, v_thr):
        cnt = 0
        while not rospy.is_shutdown():
            cur_odom = self.get_current_odom()

            cur_position = cur_odom.pose.pose.position
            cur_orientation = cur_odom.pose.pose.orientation
            cur_quat = (cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w)
            cur_yaw = euler_from_quaternion(cur_quat)[2]
            cur_v = cur_odom.twist.twist.linear

            xy_diff = sqrt((x - cur_position.x)**2 + (y - cur_position.y)**2)
            z_diff = fabs(z - cur_position.z)
            yaw_diff = fabs(self.wrap_pi(yaw - cur_yaw))
            v_diff = sqrt(cur_v.x**2 + cur_v.y**2 + cur_v.z**2)
            rospy.loginfo("%s errors xy: %.2f z: %.2f yaw: %.2f v: %.2f", id, xy_diff, z_diff, yaw_diff, v_diff)
            if (xy_thr is None or xy_diff < xy_thr) and \
                (z_thr is None or z_diff < z_thr) and \
                (yaw_thr is None or yaw_diff < yaw_thr) and \
                (v_thr is None or v_diff < v_thr):
                cnt = cnt + 1
                if cnt >= 50:
                    rospy.loginfo("goal" + str(id) +  "reached!")
                    return

    def ego_move(self, id, x, y, z, yaw, xy_thr=xy_threshold, z_thr=z_threshold, yaw_thr=yaw_threshold, v_thr = speed_thr):
        ego_setpoint = PoseStamped()
        ego_setpoint.header.stamp = rospy.Time.now()
        ego_setpoint.pose.position.x = x
        ego_setpoint.pose.position.y = y
        ego_setpoint.pose.position.z = z
        ego_setpoint.pose.orientation.w = cos(yaw / 2.0)
        ego_setpoint.pose.orientation.x = 0
        ego_setpoint.pose.orientation.y = 0
        ego_setpoint.pose.orientation.z = sin(yaw / 2.0)
        self.ego_pub.publish(ego_setpoint)
        rospy.loginfo("ego goal %s %.2f %.2f %.2f %.2f", id, x, y, z, yaw)
        self.wait_for_target(id, x, y, z, yaw, xy_thr, z_thr, yaw_thr, v_thr)

    def px4_move(self, id, x, y, z, yaw, xy_thr=xy_threshold, z_thr=z_threshold, yaw_thr=yaw_threshold, v_thr = speed_thr):
        px4_setpoint = PositionCommand()
        px4_setpoint.header.stamp = rospy.Time.now()
        px4_setpoint.position.x = x
        px4_setpoint.position.y = y
        px4_setpoint.position.z = z
        px4_setpoint.yaw = yaw
        self.px4_pub.publish(px4_setpoint)
        rospy.loginfo("px4 goal %s %.2f %.2f %.2f %.2f", id, x, y, z, yaw)
        self.wait_for_target(id, x, y, z, yaw, xy_thr, z_thr, yaw_thr, v_thr)

    def move(self, id, x, y, z, yaw):
        cur_odom = self.get_current_odom()
        cur_orientation = cur_odom.pose.pose.orientation
        cur_quat = (cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w)
        cur_yaw = euler_from_quaternion(cur_quat)[2]
        self.ego_move(id, x, y, z, cur_yaw, xy_thr=1, z_thr=0.5, yaw_thr=0.5)
        self.px4_move(id, x, y, z, yaw)
        



class UAVCommand(CommandBase):
    def __init__(self):
        self.command_base = CommandBase()
    
    def fsm(self):
        self.command_base.move("cylinder0", 7, 0, 1.3, 0)
        self.command_base.move("cylinder1", 8, -1, 1.3, pi/2)
        self.command_base.move("cylinder2", 9, -1, 1.3, pi/2)
        self.command_base.move("cylinder3", 9, 1, 1.3, pi/2)
        self.command_base.move("cylinder4", 10, 1, 1.3, pi/2)
        self.command_base.move("cylinder5", 10, -1, 1.3, pi/2)
        self.command_base.move("cylinder6", 11, -1, 1.3, pi/2)
        self.command_base.move("cylinder7", 11, 1, 1.3, pi/2)
        self.command_base.move("cylinder8", 12, 1, 1.3, pi/2)
        self.command_base.move("cylinder9", 12, 0, 1.3, pi/2)

    


if __name__ == "__main__":
    rospy.init_node("uav_command")
    #uav_command = UAVCommand()
    #uav_command.fsm()

    def show_cmd(msg):
        print(msg.position.x, msg.position.y, msg.position.z)
    command_base = CommandBase()
    px4_cmd_sub = rospy.Subscriber("/position_cmd", PositionCommand, show_cmd, queue_size=10)
    time.sleep(2)
    command_base.ego_move("test", 6, 0, 1.3, 0)
    rospy.spin()
