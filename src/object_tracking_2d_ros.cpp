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
    //    if(!has_camera_info_){
    //        ROS_WARN("No Camera Info Received Yet");
    //        return;
    //    }

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
    tracker_->setCannyHigh(ebt_th_canny_h_);
    tracker_->setCannyLow(ebt_th_canny_l_);
    tracker_->setImage(subscribed_gray);
    tracker_->tracking();

    // Convert pose matrix from CvMat to Eigen
    CvMat* pose_cv_;
    Eigen::Matrix4d pose_;
    pose_cv_ = tracker_->getPose();
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
        tracker_->renderResults();
        cv::Mat img_result = tracker_->getResultImage();
        cv::Mat img_edge   = tracker_->getEdgeImage();
        cv::imshow("ObjectTrackin2D:Result", img_result);
        cv::imshow("ObjectTrackin2D:Edges",  img_edge);
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

void ParameterCallback(object_tracking_2d_ros::object_tracking_2d_rosConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %d",
             config.ebt_th_canny_l,
             config.ebt_th_canny_h);
    ebt_th_canny_l_ = config.ebt_th_canny_l;
    ebt_th_canny_h_ = config.ebt_th_canny_h;
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

    tracker_->setCannyHigh(ebt_th_canny_h_);
    tracker_->setCannyLow(ebt_th_canny_l_);

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

void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z );
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
        cvNamedWindow("ObjectTrackin2D:Result");
        cvNamedWindow("ObjectTrackin2D:Edges");
        cvStartWindowThread();
    }

    // Start Node
    ROS_INFO("ObjectTrackin2D node started.");
    running_ = false;
    has_camera_info_ = false;

    dynamic_reconfigure::Server<Config> server;
    dynamic_reconfigure::Server<Config>::CallbackType f;
    f = boost::bind(&ParameterCallback, _1, _2);
    server.setCallback(f);

    // create an interactive marker server on the topic namespace simple_marker
    interactive_markers::InteractiveMarkerServer server1("object_tracking_2d_ros");

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/camera_link";
    int_marker.name = "my_marker";
    int_marker.description = "Simple 1-DOF Control";

    // create a grey marker_transform marker
    visualization_msgs::Marker box_marker;
    // Set the attributes for the marker
    box_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    box_marker.mesh_resource = ebt_mesh_path_;
    box_marker.scale.x = 1;
    box_marker.scale.y = 1;
    box_marker.scale.z = 1;
    box_marker.color.r = 0.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 0.0;
    box_marker.color.a = 0.5;

      // create a non-interactive control which contains the box
      visualization_msgs::InteractiveMarkerControl box_control;
      box_control.always_visible = true;
      box_control.markers.push_back( box_marker );

      // add the control to the interactive marker
      int_marker.controls.push_back( box_control );

      // create a control which will move the box
      // this control does not contain any markers,
      // which will cause RViz to insert two arrows
      visualization_msgs::InteractiveMarkerControl control;
      // make a control that rotates around the view axis
        control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
        control.orientation.w = 1;
        control.name = "rotate";

        int_marker.controls.push_back(control);

        // create a box in the center which should not be view facing,
        // but move in the camera plane.
        control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
        control.independent_marker_orientation = true;
        control.name = "move";

        control.markers.push_back( box_marker );
        control.always_visible = true;

        int_marker.controls.push_back(control);

      // add the interactive marker to our collection &
      // tell the server to call processFeedback() when feedback arrives for it
      server1.insert(int_marker, &processFeedback);

      // 'commit' changes and send to all clients
      server1.applyChanges();







    ros::spin();
    ROS_INFO("ObjectTrackin2D node stopped.");

    //Destroying Stuff
    cvDestroyWindow("ObjectTrackin2D:Result");
    cvDestroyWindow("ObjectTrackin2D:Edges");
    delete tracker_;

    return EXIT_SUCCESS;
}
