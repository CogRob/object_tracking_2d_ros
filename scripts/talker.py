#!/usr/bin/env python

# Import required Python code.
import rospy
import roslib
import std_msgs.msg

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from geometry_msgs.msg import Pose
from object_tracking_2d_ros.msg import ObjectDetection
from object_tracking_2d_ros.msg import ObjectDetections
from object_tracking_2d_ros_init.cfg import object_tracking_2d_ros_initConfig as ConfigType

# Node example class.
class NodeExample():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rate = rospy.Rate(10) # 10hz

        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)

        # Create a publisher for our custom message.
        pub = rospy.Publisher('/object_tracking_2d_ros/init_poses', ObjectDetections, queue_size=10)

        # Set the message to publish as our custom message.
        msg = ObjectDetections()

        # Initialize message variables.
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

        # Main while loop.
        while not rospy.is_shutdown():
            # Fill in custom message variables with values from dynamic reconfigure server.
            p.position.x = self.tf_x
            p.position.y = self.tf_y
            p.position.z = self.tf_z
            p.orientation.x = self.rot_x
            p.orientation.y = self.rot_y
            p.orientation.z = self.rot_z
            p.orientation.w = self.rot_w

            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()

            msg.header = h
            msg.detections[0].pose = p

            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        self.message = config["message"]
        self.tf_x = config["tf_x"]
        self.tf_y = config["tf_y"]
        self.tf_z = config["tf_z"]

        self.rot_x = config["rot_x"]
        self.rot_y = config["rot_y"]
        self.rot_z = config["rot_z"]
        self.rot_w = config["rot_q"]

        # Return the new variables.
        return config

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('custom_talker', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = NodeExample()
    except rospy.ROSInterruptException: pass
