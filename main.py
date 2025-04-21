#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
import rosbag
import os
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import cv2

# 定义输出目录
output_dir = 'output_data'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 初始化cv_bridge
bridge = CvBridge()

# 保存图像数据的回调函数
def save_image(msg, topic):
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # 保存图像
        # image_path = os.path.join(output_dir, f'{topic}_{msg.header.stamp.to_nsec()}.png')
        image_path = os.path.join(output_dir, '%s_%s.png' % (topic, msg.header.stamp.to_nsec()))
        cv2.imwrite(image_path, cv_image)
    except CvBridgeError as e:
        print(e)

# 保存点云数据的回调函数
def save_point_cloud(msg, topic):
    cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    # 保存点云数据为文本文件
    # cloud_path = os.path.join(output_dir, f'{topic}_{msg.header.stamp.to_nsec()}.txt')
    cloud_path = os.path.join(output_dir, '%s_%s.txt' % (topic, msg.header.stamp.to_nsec()))
    with open(cloud_path, 'w') as f:
        for point in cloud_points:
            #f.write(f'{point[0]} {point[1]} {point[2]}\n')
            f.write('%f %f %f\n' % (point[0], point[1], point[2]))

# 保存里程计数据的回调函数
def save_odometry(msg, topic):
    # 保存里程计数据为CSV文件
    # csv_path = os.path.join(output_dir, f'{topic}.csv')
    csv_path = os.path.join(output_dir, '%s.csv' % topic)
    with open(csv_path, 'a') as f:
        # f.write(f'{msg.header.stamp.to_sec()},{msg.pose.pose.position.x},{msg.pose.pose.position.y},{msg.pose.pose.position.z}\n')
        f.write('%f,%f,%f,%f\n' % (msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))

# 打开bag文件
bag = rosbag.Bag('/home/tai/ws/src/scripts/rosbag/camera.bag')

# 遍历bag文件中的所有消息
for topic, msg, t in bag.read_messages():
    if topic == '/image0_raw' or topic == '/image1_raw':
        save_image(msg, topic)
    elif topic == '/lidar_points':
        save_point_cloud(msg, topic)
    elif topic == '/image0_odom' or topic == '/image1_odom' or topic == '/lidar_odom':
        save_odometry(msg, topic)

# 关闭bag文件
bag.close()

print('Data has been saved to', output_dir)