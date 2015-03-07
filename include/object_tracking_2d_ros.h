//Include ROS libraries for node messages
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

// Include OpenCV for images and viewer
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

//Include Eigen tools for pose handeling
#include <Eigen/Core>
#include <Eigen/Geometry>

//Include Visualization tools for mesh marker
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>

//Include Boost tools for detection arrays
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

// Include the trackers from the object_tracking_2d project
#include "include/object_tracking_2D/tracker_irls.h"
#include "include/object_tracking_2D/tracker_pf_texture.h"
#include "include/object_tracking_2D/tracker_pf_textureless.h"

// Include object_tracking_2d_ros message types
#include "ObjectDetection.h"
#include <object_tracking_2d_ros/ObjectDetections.h>
#include <object_tracking_2d_ros/object_tracking_2d_rosConfig.h>

// Define the default topic names
const std::string DEFAULT_IMAGE_TOPIC = "image";
const std::string DEFAULT_CAMERA_INFO_TOPIC = "camera_info";
const std::string DEFAULT_MARKER_TOPIC = "marker_array";
const std::string DEFAULT_DETECTIONS_TOPIC = "detections";

// ROS parts
ros::NodeHandlePtr node_;
boost::shared_ptr<image_transport::ImageTransport> image_;
sensor_msgs::CameraInfo camera_info_;

// ROS Publishers and Subscribers
ros::Publisher marker_publisher_;
ros::Publisher ebt_publisher_;
ros::Subscriber info_subscriber;
image_transport::Subscriber image_subscriber;


// Config
object_tracking_2d_ros::object_tracking_2d_rosConfig config_;
typedef object_tracking_2d_ros::object_tracking_2d_rosConfig Config;

// EBT plugin perams
std::string ebt_tracker_type_;
std::string ebt_mesh_resource_;
int ebt_num_particle_;
int ebt_min_keypoint_;
double ebt_th_cm_;
std::string ebt_obj_path_;
std::string ebt_mesh_path_;
std::vector<float> ebt_init_pose_;
bool ebt_dull_edge_;
int ebt_width_;
int ebt_height_;
double ebt_sample_step_;
std::string ebt_intrinsic_;
std::string ebt_distortion_;
bool ebt_display_;
int ebt_th_canny_l_;
int ebt_th_canny_h_;

// Settings and local information
bool viewer_;
bool running_;
bool has_camera_info_;
std::string display_type_;

// Package Functions

// Returns pose from detection
Eigen::Matrix4d GetDetectionTransform(ObjectDetection detection);

// Callback for camera info
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

// Callback for new subcription
void ConnectCallback(const ros::SingleSubscriberPublisher& info);

// Callback for unsubcription
void DisconnectCallback(const ros::SingleSubscriberPublisher& info);

// Handler for unsubcription
void DisconnectHandler();

// Read Parameters for unsubcription
void GetParameterValues();

// Advertize Publishers
void SetupPublisher();

// Initialize EBT tracker using Parameters
void InitializeTracker();

// Initialize the ROS Node
void InitializeROSNode();

// Global EBT tracker
TrackerBase* tracker_;

