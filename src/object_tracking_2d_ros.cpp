//Include ROS libraries for node messages
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

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

//Include Boost tools for detection arrays
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

//Include the base of object_tracking_2d_ros
#include "object_tracking_2d_ros.h"


using namespace std;

Eigen::Matrix4d GetDetectionTransform(ObjectDetection detection)
{
    // Draft function for now
    return detection.pose;
}

void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    camera_info_ = (*camera_info);
    has_camera_info_ = true;
}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Dont atempt to use the image without having info about the camera first
    if(!has_camera_info_){
        ROS_WARN("No Camera Info Received Yet");
        return;
    }

    // Get the image
    cv_bridge::CvImagePtr subscribed_ptr;
    try
    {
        subscribed_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat subscribed_gray = subscribed_ptr->image;
    cv::Mat tmp;
    cv::Point2d opticalCenter(0.5*subscribed_gray.rows,
                              0.5*subscribed_gray.cols);

    // Apply the tracker to the image
    tracker_->setImage(subscribed_gray);
    tracker_->tracking();

    // Convert pose matrix from CvMat to Eigen
    CvMat* pose_cv_;
    Eigen::Matrix4d pose_;
    pose_cv_ = tracker_->obj_model_->pose_;
    for(int r = 0; r < 4; r++){
        for(int c = 0; c < 4; c++){
            pose_(r,c) = CV_MAT_ELEM(*pose_cv_, float, r, c);
        }
    }

    // Store the detection into an array struture
    ObjectDetection d;
    d.pose = pose_;
    d.good = true;
    d.id = 1;
    ObjectDetectionArray detections;
    detections.push_back(d);

    // Store the detection into an array struture
    visualization_msgs::MarkerArray marker_transforms;
    object_tracking_2d_ros::ObjectDetections object_detections;
    object_detections.header.frame_id = msg->header.frame_id;
    object_detections.header.stamp = msg->header.stamp;

    if(viewer_)
    {
        // Render the results for the viewer
        CvScalar color = cvScalar(255,255,0);
        tracker_->obj_model_->displayPoseLine(tracker_->img_result_, pose_cv_, color, 1, false);
        tracker_->obj_model_->displaySamplePointsAndErrors(tracker_->img_edge_);
    }

    // Loop over each detection
    for(unsigned int i = 0; i < detections.size(); ++i)
    {
        // skip bad detections
        if(!detections[i].good)
        {
            continue;
        }

        // Get quaternion for the marker
        Eigen::Matrix4d pose = GetDetectionTransform(detections[i]);
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Quaternion<double> q(R);

        // Set the attributes for the marker
        visualization_msgs::Marker marker_transform;
        marker_transform.header.frame_id = msg->header.frame_id;
        marker_transform.header.stamp = msg->header.stamp;
        stringstream convert;
        convert << "tag" << detections[i].id;
        marker_transform.ns = convert.str().c_str();
        marker_transform.id = detections[i].id;

        // Set the object mesh for the marker
        marker_transform.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_transform.mesh_resource = ebt_mesh_path_;
        marker_transform.scale.x = 1;
        marker_transform.scale.y = 1;
        marker_transform.scale.z = 1;
        marker_transform.color.r = 0.0;
        marker_transform.color.g = 1.0;
        marker_transform.color.b = 0.0;
        marker_transform.color.a = 1.0;

        // Set the marker's pose
        marker_transform.action = visualization_msgs::Marker::ADD;
        marker_transform.pose.position.x = pose(0,3);
        marker_transform.pose.position.y = pose(1,3);
        marker_transform.pose.position.z = pose(2,3);
        marker_transform.pose.orientation.x = q.x();
        marker_transform.pose.orientation.y = q.y();
        marker_transform.pose.orientation.z = q.z();
        marker_transform.pose.orientation.w = q.w();

        // Add the marker to marker array message
        marker_transforms.markers.push_back(marker_transform);

        // Fill in Object detection.
        object_tracking_2d_ros::ObjectDetection object_det;
        object_det.header = marker_transform.header;
        object_det.id = marker_transform.id;
        object_det.pose = marker_transform.pose;

        // Add the detection to detection array message
        object_detections.detections.push_back(object_det);
    }

    // Publish the marker and detection messages
    marker_publisher_.publish(marker_transforms);
    ebt_publisher_.publish(object_detections);

    if(viewer_)
    {
        // Show the results for the viewer
        cv::Mat my_image = tracker_->img_edge_;
        cv::imshow("ObjectTrackin2D", my_image);
    }
}

void ConnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = marker_publisher_.getNumSubscribers()
            + ebt_publisher_.getNumSubscribers();
    ROS_DEBUG("Subscription detected! (%d subscribers)", subscribers);

    if(subscribers && !running_)
    {
        ROS_DEBUG("New Subscribers, Connecting to Input Image Topic.");
        ros::TransportHints ros_transport_hints(ros::TransportHints().tcpNoDelay());
        image_transport::TransportHints image_transport_hint(image_transport::TransportHints(
                                                                 "raw", ros_transport_hints, (*node_),
                                                                 "image_transport"));

        image_subscriber = (*image_).subscribe(
                    DEFAULT_IMAGE_TOPIC, 1, &ImageCallback,
                    image_transport_hint);
        info_subscriber = (*node_).subscribe(
                    DEFAULT_CAMERA_INFO_TOPIC, 10, &InfoCallback);
        running_ = true;
    }
}

void DisconnectHandler()
{
}

void DisconnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = marker_publisher_.getNumSubscribers()
            + ebt_publisher_.getNumSubscribers();
    ROS_DEBUG("Unsubscription detected! (%d subscribers)", subscribers);

    if(!subscribers && running_)
    {
        ROS_DEBUG("No Subscribers, Disconnecting from Input Image Topic.");
        image_subscriber.shutdown();
        info_subscriber.shutdown();
        running_ = false;
    }
}

void GetParameterValues()
{
    // Load node-wide configuration values.
    node_->param ("ebt_tracker_type", ebt_tracker_type_, std::string("irls"));
    node_->param ("ebt_num_particle", ebt_num_particle_, 1);
    node_->param ("ebt_min_keypoint", ebt_min_keypoint_, 20);
    node_->param ("ebt_th_cm", ebt_th_cm_, 0.2);
    node_->param ("ebt_obj_path", ebt_obj_path_, std::string("obj_name"));
    node_->param ("ebt_mesh_path", ebt_mesh_path_, std::string("mesh_path"));
    node_->param ("ebt_mesh_resource", ebt_mesh_resource_, std::string(""));
    node_->getParam("ebt_init_pose", ebt_init_pose_);
    node_->param ("ebt_dull_edge", ebt_dull_edge_, false);
    node_->param ("ebt_width", ebt_width_, 640);
    node_->param ("ebt_height", ebt_height_, 480);
    node_->param ("ebt_sample_step", ebt_sample_step_, 0.005);
    node_->param ("ebt_intrinsic", ebt_intrinsic_, std::string("Intrinsics_normal.xml"));
    node_->param ("ebt_distortion", ebt_distortion_, std::string("Distortion_normal.xml"));
    node_->param ("ebt_th_canny_l", ebt_th_canny_l_, 100);
    node_->param ("ebt_th_canny_h", ebt_th_canny_h_, 120);
    node_->param ("ebt_display", ebt_display_, false);
    node_->param("viewer", viewer_, true);
}

void SetupPublisher()
{
    // Add callbacks
    ros::SubscriberStatusCallback connect_callback = &ConnectCallback;
    ros::SubscriberStatusCallback disconnect_callback = &DisconnectCallback;

    // Publisher
    marker_publisher_ = node_->advertise<visualization_msgs::MarkerArray>(
                DEFAULT_MARKER_TOPIC, 1, connect_callback,
                disconnect_callback);
    ebt_publisher_ = node_->advertise<object_tracking_2d_ros::ObjectDetections>(
                DEFAULT_DETECTIONS_TOPIC, 1, connect_callback, disconnect_callback);
}

void InitializeTracker()
{
    // Create the desired tracker
    if(ebt_tracker_type_.compare("irls") == 0)
    {
        tracker_ = new IrlsTracker ();
        tracker_->setMinKeypointMatches(ebt_min_keypoint_);
    }
    else if(ebt_tracker_type_.compare("pf") == 0)
    {
        tracker_ = new TextureParticleFilterTracker ();
        ((TextureParticleFilterTracker*)tracker_)->setNumParticle(ebt_num_particle_);
        ((TextureParticleFilterTracker*)tracker_)->initParticleFilter();
    }
    else if(ebt_tracker_type_.compare("pf_textureless") == 0)
    {
        tracker_ = new TexturelessParticleFilterTracker ();
        ((TexturelessParticleFilterTracker*)tracker_)->setNumParticle(ebt_num_particle_);
        ((TexturelessParticleFilterTracker*)tracker_)->setThresholdCM(ebt_th_cm_);
        ((TexturelessParticleFilterTracker*)tracker_)->initParticleFilter();
    }
    else
    {
        ROS_ERROR_STREAM("Unknown tracker method name: " << ebt_tracker_type_);
    }

    // Configure the desired tracker
    tracker_->setSampleStep(ebt_sample_step_);
    tracker_->setDisplay(ebt_display_);
    tracker_->setNetworkMode(false);
    tracker_->setConsideringDullEdges(ebt_dull_edge_);
    tracker_->setTracking(true);
    std::string input = "ach";
    std::string ach_channel = "none";

    // oeverwrite pose_init
    CvMat* pose_init_ = cvCreateMat(4, 4, CV_32F);
    for (int i = 0; i < 16; i++){
        pose_init_->data.fl[i] = ebt_init_pose_[i];
    }

    // Initialize the desired tracker
    tracker_->initTracker(ebt_obj_path_, input, ebt_intrinsic_, ebt_distortion_, ebt_width_, ebt_height_, pose_init_ , ach_channel);
}

void InitializeROSNode(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking_2d_ros");
    node_ =  boost::make_shared<ros::NodeHandle>("~");
    image_ = boost::make_shared<image_transport::ImageTransport>(*node_);
}

int main(int argc, char **argv)
{
    // Initialize Node
    InitializeROSNode(argc,argv);
    GetParameterValues();
    SetupPublisher();
    InitializeTracker();

    if(viewer_){
        // Make a cv window for the viewer
        cvNamedWindow("ObjectTrackin2D");
        cvStartWindowThread();
    }

    // Start Node
    ROS_INFO("ObjectTrackin2D node started.");
    running_ = false;
    has_camera_info_ = false;
    ros::spin();
    ROS_INFO("ObjectTrackin2D node stopped.");

    //Destroying Stuff
    cvDestroyWindow("ObjectTrackin2D");
    delete tracker_;

    return EXIT_SUCCESS;
}
