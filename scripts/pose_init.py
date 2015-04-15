#!/usr/bin/env python
import rospy
import roslib
import std_msgs.msg

from geometry_msgs.msg import Pose
from object_tracking_2d_ros.msg import ObjectDetection
from object_tracking_2d_ros.msg import ObjectDetections

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

def talker():
    pub = rospy.Publisher('/object_tracking_2d_ros/init_poses', ObjectDetections, queue_size=10)
    rate = rospy.Rate(30) # 10hz
    msg = ObjectDetections()

    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()

    p = Pose()
    p.position.x = 0
    p.position.y = 0
    p.position.z = 1
    p.orientation.x = 0
    p.orientation.y = 0
    p.orientation.z = 0
    p.orientation.w = 1

    msg.header = h
    msg.detections.append(ObjectDetection())
    msg.detections[0].id = "1"
    msg.detections[0].pose = p

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('custom_talker', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException: pass
