#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <include/object_tracking_2D/tracker_base.h>
#include <include/object_tracking_2D/tracker_irls.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include "object_tracking_2d_ros.h"
#include <object_tracking_2d_ros/ObjectDetections.h>


using namespace std;

// Functions

Eigen::Matrix4d GetDetectionTransform()
{
//    double tag_size = 0;

//    std::vector<cv::Point3f> object_pts;
//    std::vector<cv::Point2f> image_pts;
//    double tag_radius = tag_size/2.;
    
//    object_pts.push_back(cv::Point3f(-tag_radius, -tag_radius, 0));
//    object_pts.push_back(cv::Point3f( tag_radius, -tag_radius, 0));
//    object_pts.push_back(cv::Point3f( tag_radius,  tag_radius, 0));
//    object_pts.push_back(cv::Point3f(-tag_radius,  tag_radius, 0));
    
//    image_pts.push_back(detection.p[0]);
//    image_pts.push_back(detection.p[1]);
//    image_pts.push_back(detection.p[2]);
//    image_pts.push_back(detection.p[3]);

//    cv::Matx33f intrinsics(camera_info_.K[0], 0, camera_info_.K[2],
//                           0, camera_info_.K[4], camera_info_.K[5],
//                           0, 0, 1);
    
//    cv::Mat rvec, tvec;
//    cv::Vec4f dist_param(0,0,0,0);
//    cv::solvePnP(object_pts, image_pts, intrinsics, dist_param,
//            rvec, tvec);
//    cv::Matx33d r;
//    cv::Rodrigues(rvec, r);
//    Eigen::Matrix3d rot;
//    rot << r(0,0), r(0,1), r(0,2),
//           r(1,0), r(1,1), r(1,2),
//           r(2,0), r(2,1), r(2,2);
    
//    Eigen::Matrix4d T;
//    T.topLeftCorner(3,3) = rot;
//    T.col(3).head(3) <<
//            tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
//    T.row(3) << 0,0,0,1;
    
//    return T;
}

// Callback for camera info
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    camera_info_ = (*camera_info);
    has_camera_info_ = true;
}

// Callback for image data
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//    if(!has_camera_info_){
//        ROS_WARN("No Camera Info Received Yet");
//        return;
//    }

//    // Get the image
//    cv_bridge::CvImagePtr subscribed_ptr;
//    try
//    {
//        subscribed_ptr = cv_bridge::toCvCopy(msg, "mono8");
//    }
//    catch(cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
    
//    cv::Mat subscribed_gray = subscribed_ptr->image;
//    cv::Mat tmp;
//    cv::Point2d opticalCenter(0.5*subscribed_gray.rows,
//                              0.5*subscribed_gray.cols);
//    TagDetectionArray detections;
//    detector_->process(subscribed_gray, opticalCenter, detections);
//    visualization_msgs::MarkerArray marker_transforms;
//    object_tracking_2d_ros::ObjectDetections object_detections;
//    object_detections.header.frame_id = msg->header.frame_id;
//    object_detections.header.stamp = msg->header.stamp;
    
//    if(viewer_)
//    {
//        subscribed_gray = family_->superimposeDetections(subscribed_gray,
//                                                         detections);
//    }
//    for(unsigned int i = 0; i < detections.size(); ++i)
//    {
//        // skip bad detections
//        if(!detections[i].good)
//        {
//            continue;
//        }
        
//        Eigen::Matrix4d pose = GetDetectionTransform(detections[i]);
        
//        // Get this info from earlier code, don't extract it again
//        Eigen::Matrix3d R = pose.block<3,3>(0,0);
//        Eigen::Quaternion<double> q(R);
        
//        double tag_size = 0;
//        cout << tag_size << " " << detections[i].id << endl;
        
//        visualization_msgs::Marker marker_transform;
//        marker_transform.header.frame_id = msg->header.frame_id;
//        marker_transform.header.stamp = msg->header.stamp;
//        stringstream convert;
//        convert << "tag" << detections[i].id;
//        marker_transform.ns = convert.str().c_str();
//        marker_transform.id = detections[i].id;
//        if(display_type_ == "ARROW"){
//            marker_transform.type = visualization_msgs::Marker::ARROW;
//            marker_transform.scale.x = tag_size;
//            marker_transform.scale.y = tag_size*10;
//            marker_transform.scale.z = tag_size*0.5;
//        }
//        else if(display_type_ == "CUBE"){
//            marker_transform.type = visualization_msgs::Marker::CUBE;
//            marker_transform.scale.x = tag_size;
//            marker_transform.scale.y = tag_size;
//            marker_transform.scale.z = 0.01 * tag_size;
//        }
//        marker_transform.action = visualization_msgs::Marker::ADD;
//        marker_transform.pose.position.x = pose(0,3);
//        marker_transform.pose.position.y = pose(1,3);
//        marker_transform.pose.position.z = pose(2,3);
//        marker_transform.pose.orientation.x = q.x();
//        marker_transform.pose.orientation.y = q.y();
//        marker_transform.pose.orientation.z = q.z();
//        marker_transform.pose.orientation.w = q.w();
        
//        marker_transform.color.r = 1.0;
//        marker_transform.color.g = 0.0;
//        marker_transform.color.b = 1.0;
//        marker_transform.color.a = 1.0;
//        marker_transforms.markers.push_back(marker_transform);
        
//        // Fill in AprilTag detection.
//        apriltags::AprilTagDetection apriltag_det;
//        apriltag_det.header = marker_transform.header;
//        apriltag_det.id = marker_transform.id;
//        apriltag_det.tag_size = tag_size;
//        apriltag_det.pose = marker_transform.pose;
//        const TagDetection &det = detections[i];
//        for(uint pt_i = 0; pt_i < 4; ++pt_i)
//        {
//            geometry_msgs::Point32 img_pt;
//            img_pt.x = det.p[pt_i].x;
//            img_pt.y = det.p[pt_i].y;
//            img_pt.z = 1;
//            apriltag_det.corners2d[pt_i] = img_pt;
//        }
//        apriltag_detections.detections.push_back(apriltag_det);
//    }
//    marker_publisher_.publish(marker_transforms);
//    ebt_publisher_.publish(object_detections);
    
//    if(viewer_)
//    {
//        cv::imshow("AprilTags", subscribed_gray);
//    }
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
    node_->param ("ebt_obj_name", ebt_obj_name_, std::string("obj_name"));
    node_->param ("ebt_mesh_resource", ebt_mesh_resource_, std::string(""));
    node_->param ("ebt_init_pose", ebt_init_pose_, std::string("1,0,0,0,0,1,0,0,0,0,1,0.5.0,0,0,0,1"));
    node_->param ("ebt_dull_edge", ebt_dull_edge_, false);
    // node_->param ("ebt_input", ebt_input_, std::string("normal"));
    // node_->param ("ebt_width", ebt_width_, 640);
    // node_->param ("ebt_height", ebt_height_, 480);
    node_->param ("ebt_sample_step", ebt_sample_step_, 0.005);
    // node_->param ("ebt_intrinsic", ebt_intrinsic_, std::string("Intrinsics_normal.xml"));
    // node_->param ("ebt_distortion", ebt_distortion_, std::string("Distortion_normal.xml"));
    node_->param ("ebt_display", ebt_display_, true);
    node_->param ("ebt_th_canny_l", ebt_th_canny_l_, 100);
    node_->param ("ebt_th_canny_h", ebt_th_canny_h_, 120);
    node_->param("viewer", viewer_, true);
}

void SetupPublisher()
{    
    ros::SubscriberStatusCallback connect_callback = &ConnectCallback;
    ros::SubscriberStatusCallback disconnect_callback = &DisconnectCallback;
    
    // Publisher
    marker_publisher_ = node_->advertise<visualization_msgs::MarkerArray>(
            DEFAULT_MARKER_TOPIC, 1, connect_callback,
            disconnect_callback);
    ebt_publisher_ = node_->advertise<object_tracking_2d_ros::ObjectDetections>(
            DEFAULT_DETECTIONS_TOPIC, 1, connect_callback, disconnect_callback);
}

void InitializeObjects()
{
    //TODO: Add in tracker
//    tracker_ = new ObjectTracker(@@@@@@@@@);
}

void InitializeROSNode(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking_2d_ros");
    node_ =  boost::make_shared<ros::NodeHandle>("~");
    image_ = boost::make_shared<image_transport::ImageTransport>(*node_);
}

int main(int argc, char **argv)
{
    InitializeROSNode(argc,argv);
    GetParameterValues();
    SetupPublisher();
    InitializeObjects();

    if(viewer_){
        cvNamedWindow("ObjectTrackin2D");
        cvStartWindowThread();
    }

    ROS_INFO("ObjectTrackin2D node started.");
    running_ = false;
    has_camera_info_ = false;
    ros::spin();
    ROS_INFO("ObjectTrackin2D node stopped.");

    //Destroying Stuff
    cvDestroyWindow("ObjectTrackin2D");
//    delete tracker_;

    return EXIT_SUCCESS;
}
