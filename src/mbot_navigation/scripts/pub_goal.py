#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

goal1 = [-2.9447, 0.3908, 0.0, 0.0, 0.0, -0.15877, 0.9873]

def odom_callback(data):
    """回调函数，处理从/odom话题接收到的消息"""
    rospy.loginfo("Position: x=%f, y=%f, z=%f", data.pose.pose.position.x,
                                             data.pose.pose.position.y,
                                             data.pose.pose.position.z)
                                             
    if count_dis(-2.9447, 0.3908, data.pose.pose.position.x, data.pose.pose.position.y) > 0.5:
    	print("going to goal!")
    	send_goal(-2.9447, 0.3908, 0.0, 0.0, 0.0, -0.15877, 0.9873)

def count_dis(x, y, ox, oy):
    distance = 0.0
    dis = math.sqrt((x - ox)**2 + (y - oy)**2)
    print(dis)

    return dis

def send_goal(px, py, pz, ox, oy, oz, ow):

    # 等待一会儿，以确保发布者准备好接收消息
    rospy.sleep(1)

    # 创建目标点消息
    goal = PoseStamped()
    goal.header.frame_id = "map"  # 确保这是正确的坐标系
    goal.header.stamp = rospy.Time.now()

    # 设置目标点的位置
    goal.pose.position.x = px  # 目标x坐标
    goal.pose.position.y = py  # 目标y坐标
    goal.pose.position.z = pz  # 目标z坐标

    # 设置目标点的方向（这里假设目标方向是面向正北）
    # 使用tf.transformations库生成四元数
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, 1.57)  # 1.57 rad 是 90 度
    goal.pose.orientation.x = ox
    goal.pose.orientation.y = oy
    goal.pose.orientation.z = oz
    goal.pose.orientation.w = ow

    # 发布目标点
    pub.publish(goal)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('pub_goal', anonymous=True)
    # 创建一个Publisher对象，用于发布目标点
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    rospy.spin()
    
#    print("start move goal")

#    send_goal(-2.9447, 0.3908, 0.0, 0.0, 0.0, -0.15877, 0.9873)
    # 等待一段时间，以便让移动基座有足够的时间达到目标
#    rospy.sleep(10)
