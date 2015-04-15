#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import std_msgs.msg
roslib.load_manifest('object_tracking_2d_ros')

# Import custom message data and dynamic reconfigure variables.
from object_tracking_2d_ros.msg import ObjectDetection
from object_tracking_2d_ros.msg import ObjectDetections
from geometry_msgs.msg import Pose

# publish to init_poses
pub = rospy.Publisher('/object_tracking_2d_ros/init_poses', ObjectDetections, queue_size=1)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
	# create a twist message, fill in the details
	ObjectDetections = ObjectDetections()
	ObjectDetection = ObjectDetection()
	p = Pose()

	p.position.x = 0
	p.position.y = 0
	p.position.z = 5
	p.orientation.x = 0
	p.orientation.y = 0
	p.orientation.z = 0
	p.orientation.w = 1

	ObjectDetection.id = "1"  
	ObjectDetection.pose = p
	ObjectDetections = [ObjectDetection]

	h = std_msgs.msg.Header()
	h.stamp = rospy.Time.now()

	# rospy.loginfo(ObjectDetections)
	pub.publish(h,ObjectDetections)
	rate.sleep()