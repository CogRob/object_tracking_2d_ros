//Include the base of object_tracking_2d_ros
#include "object_tracking_2d_ros.h"

using namespace std;

Eigen::Matrix4d GetDetectionTransform(ObjectDetection detection)
{
    // Draft function for now
    return detection.pose;
}

void ProcessUserActions()
{
    char input = remote_input_.c_str()[0];
    remote_input_ = "";

    if(user_input_thread_ != NULL)
    {
        char console_input = user_input_thread_->getInput();
        if(console_input != 0)
            input = console_input;
    }

    switch(input)
    {
    case 'q':
        quit_ = true;
        ROS_INFO("ObjectTrackin2D user input: QUIT\n");
        break;
    case 'r':
        ebt_init_ = true;
        ROS_INFO("ObjectTrackin2D user input: RESET\n");
        break;
    case 'k':
        tracker_->saveKeyframe();
        ROS_INFO("ObjectTrackin2D user input: SAVE KEYFRAME\n");
        break;
//       case '1':
//        ebt_obj_id_ = '1';
//        break;
//    default:
//        ebt_obj_id_ = '0';
    }
}

void RemoteKeyCallback(const std_msgs::StringConstPtr& key_input)
{
    remote_input_ = key_input->data;
}

void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    camera_info_ = (*camera_info);
    has_camera_info_ = true;
}

void ApplyTrackerSettingsCallback(TrackerBase* tracker)
{
    tracker->setCannyHigh(ebt_th_canny_h_);
    tracker->setCannyLow(ebt_th_canny_l_);
    tracker->setValidSamplePointsRatio(ebt_th_valid_ratio_);
    tracker->setSampleStep(ebt_sample_step_);
    tracker->setMaxSearchDistance(ebt_maxd_);
    tracker->setConsideringDullEdges(ebt_dull_edge_);
    tracker->setMinKeypointMatches(ebt_min_keypoint_);
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

    // Convert pose matrix from Eigen to CvMat
    CvMat* pose_cv = cvCreateMat(4, 4, CV_32F);
    CvMat* cov_cv = cvCreateMat(6, 6, CV_32F);
    for(int r = 0; r < 4; r++){
        for(int c = 0; c < 4; c++){
            CV_MAT_ELEM(*pose_cv, float, r, c) = pose_(r,c);
        }
    }

    // Apply the tracker to the image
    ApplyTrackerSettingsCallback(tracker_);
    tracker_->setPose(pose_cv);
    ProcessUserActions();
    tracker_->setImage(subscribed_gray);
    if(ebt_reset_){
        tracker_->init_ = false;
        ROS_INFO("ObjectTrackin2D message input: RESET\n");
        ebt_reset_ = false;
    }
    if(ebt_auto_init_){
        ebt_init_ = tracker_->init_;
    }
    if(ebt_init_){
        tracker_->init_ = true;
        tracker_->initialize();
        ROS_INFO("ObjectTrackin2D message input: REINIT\n");
        ebt_init_ = tracker_->init_;
    }

    if(ebt_init_)
        return;

    tracker_->tracking();


    pose_cv = tracker_->getPose();
    cov_cv = tracker_->getCovariance();

    // Convert pose matrix from CvMat to Eigen
    for(int r = 0; r < 4; r++){
        for(int c = 0; c < 4; c++){
            pose_(r,c) = CV_MAT_ELEM(*pose_cv, float, r, c);
        }
    }

    // Convert cov matrix from CvMat to double array
    for(int r = 0; r < 6; r++){
        for(int c = 0; c < 6; c++){
            cov_[r*6+c] = CV_MAT_ELEM(*cov_cv, float, r, c);
        }
    }

    // Store the detection into an array struture
    ObjectDetection d;
    d.pose = pose_;
    d.good = !tracker_->init_;
    d.id = 0;
    d.ns = ebt_obj_id_;
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
//        // skip bad detections
//        if(!detections[i].good)
//        {
//            continue;
//        }

        // Get quaternion for the marker
        Eigen::Matrix4d pose = GetDetectionTransform(detections[i]);
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Quaternion<double> q(R);

        // Set the attributes for the marker
        visualization_msgs::Marker marker_transform;
        marker_transform.header.frame_id = msg->header.frame_id;
        marker_transform.header.stamp = msg->header.stamp;
//        stringstream convert;
//        convert << "tag" << detections[i].id;
        marker_transform.id = detections[i].id;
        marker_transform.ns = detections[i].ns;

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
        object_det.ns = marker_transform.ns;
        object_det.pose.pose = marker_transform.pose;
        object_det.pose.covariance = cov_;
        object_det.good = d.good;

        // Add the detection to detection array message
        object_detections.detections.push_back(object_det);
    }

    // Publish the marker and detection messages
    marker_publisher_.publish(marker_transforms);
    ebt_publisher_.publish(object_detections);

    // If viewing the tracking results
    if(viewer_ || viewing_)
    {
        // Render the results and edges
        tracker_->renderResults();
        cv::Mat img_result = tracker_->getResultImage();
        cv::Mat img_edge   = tracker_->getEdgeImage();

        // If using opencv viewer
        if(viewer_){
            cv::imshow("ObjectTrackin2D:Result", img_result);
            cv::imshow("ObjectTrackin2D:Edges",  img_edge);
        }

        // If viewing over message subscription
        if(viewing_){
            cv_bridge::CvImage cvi;
            cvi.header = msg->header;

            cvi.encoding = "rgb8";
            cvi.image = img_result;
            sensor_msgs::Image img_result_msg;
            cvi.toImageMsg(img_result_msg);
            img_result_publisher_.publish (img_result_msg);

            cvi.encoding = "bgr8";
            cvi.image = img_edge;
            sensor_msgs::Image img_edge_msg;
            cvi.toImageMsg(img_edge_msg);
            img_edge_publisher_.publish (img_edge_msg);
        }
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

void ViewerConnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = img_result_publisher_.getNumSubscribers()
            + img_edge_publisher_.getNumSubscribers();
    ROS_DEBUG("Viewer Subscription detected! (%d subscribers)", subscribers);

    if(subscribers && !viewing_)
    {
        ROS_DEBUG("New Subscribers, Enabling ROS viewer.");
        viewing_ = true;
    }
}

void ViewerDisconnectHandler()
{
}

void ViewerDisconnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = img_result_publisher_.getNumSubscribers()
            + img_edge_publisher_.getNumSubscribers();
    ROS_DEBUG("Viewer Unsubscription detected! (%d subscribers)", subscribers);

    if(!subscribers && viewing_)
    {
        ROS_DEBUG("No Subscribers, Disabling ROS viewer.");
        viewing_ = false;
    }
}

void GetParameterValues()
{
    // Load node-wide configuration values.
    node_->param ("ebt_tracker_type", ebt_tracker_type_, std::string("irls"));
    node_->param ("ebt_num_particle", ebt_num_particle_, 1);
    node_->param ("ebt_min_keypoint", ebt_min_keypoint_, 20);
    node_->param ("ebt_auto_init", ebt_auto_init_, true);
    node_->param ("ebt_init", ebt_init_, true);
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
    node_->param ("ebt_th_valid_ratio", ebt_th_valid_ratio_, 0.1);
    node_->param ("ebt_maxd", ebt_maxd_, 32);
    node_->param ("ebt_display", ebt_display_, false);
    node_->param ("user_input", user_input_, true);
    node_->param ("viewer", viewer_, false);

    boost::filesystem::path p(ebt_obj_path_);
    ebt_obj_id_ = p.filename().string();
}

void ParameterCallback(object_tracking_2d_ros::object_tracking_2d_rosConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request");
    ebt_th_canny_l_ = config.ebt_th_canny_l;
    ebt_th_canny_h_ = config.ebt_th_canny_h;
    ebt_th_valid_ratio_ = config.ebt_th_valid_ratio;
    ebt_sample_step_ = config.ebt_sample_step;
    ebt_maxd_ = config.ebt_maxd;
    ebt_dull_edge_ = config.ebt_dull_edge;
    ebt_min_keypoint_ = config.ebt_min_keypoint;
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


    // Add Viewing callbacks
    ros::SubscriberStatusCallback viewer_connect_callback = &ViewerConnectCallback;
    ros::SubscriberStatusCallback viewer_disconnect_callback = &ViewerDisconnectCallback;

    // Viewing Publisher
    img_result_publisher_ = node_->advertise<sensor_msgs::Image>(
                DEFAULT_IMAGE_RESULT_TOPIC, 1, viewer_connect_callback, viewer_disconnect_callback);
    img_edge_publisher_ = node_->advertise<sensor_msgs::Image>(
                DEFAULT_IMAGE_EDGE_TOPIC, 1, viewer_connect_callback, viewer_disconnect_callback);
}

void SetupSubscriber()
{
    // Subscribe to init pose messages
    init_poses_subscriber = (*node_).subscribe(
                DEFAULT_INIT_POSES_TOPIC, 1, &InitPosesCallback);

    // Subscribe to remote input
    if(user_input_){
        user_input_thread_ = boost::make_shared<vk::UserInputThread>();
    }
    sub_remote_key_ = (*node_).subscribe(
                DEFAULT_USER_INPUT_TOPIC, 5, &RemoteKeyCallback);
}

void InitPosesCallback(const object_tracking_2d_ros::ObjectDetections& msg)
{
    ROS_DEBUG("Init Poses Message of size %d Recived)", msg.detections.size());
    for(unsigned int i = 0; i < msg.detections.size(); ++i)
    {
        if(!ebt_obj_id_.compare(msg.detections[i].ns)){
            if(not msg.detections[i].good){
                geometry_msgs::Pose m = msg.detections[i].pose.pose;
                Eigen::Translation3d t(m.position.x,
                                       m.position.y,
                                       m.position.z);
                Eigen::Quaterniond r(m.orientation.w,
                                     m.orientation.x,
                                     m.orientation.y,
                                     m.orientation.z);
                pose_ = (t * r).matrix();
                ROS_DEBUG_STREAM("Init Pose Set: "<<pose_);
                ebt_reset_ = true;
            }
            ebt_init_ = msg.detections[i].init;
            ROS_DEBUG("Init SURF: %s", ebt_init_ ? "True" : "False");
        }
    }
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
        tracker_->setMinKeypointMatches(ebt_min_keypoint_);
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
    tracker_->setMaxSearchDistance(ebt_maxd_);
    tracker_->setDisplay(ebt_display_);
    tracker_->setNetworkMode(false);
    tracker_->setConsideringDullEdges(ebt_dull_edge_);
    tracker_->setTracking(true);

    tracker_->setCannyHigh(ebt_th_canny_h_);
    tracker_->setCannyLow(ebt_th_canny_l_);

    std::string input = "ach";
    std::string ach_channel = "none";

    // oeverwrite pose_init
    pose_init_ = cvCreateMat(4, 4, CV_32F);
    for (int i = 0; i < 16; i++){
        pose_init_->data.fl[i] = ebt_init_pose_[i];
    }

    // Convert pose matrix from CvMat to Eigen
    for(int r = 0; r < 4; r++){
        for(int c = 0; c < 4; c++){
            pose_(r,c) = CV_MAT_ELEM(*pose_init_, float, r, c);
        }
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
    SetupSubscriber();
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
    viewing_ = false;
    has_camera_info_ = false;

    dynamic_reconfigure::Server<Config> server;
    dynamic_reconfigure::Server<Config>::CallbackType f;
    f = boost::bind(&ParameterCallback, _1, _2);
    server.setCallback(f);

    // start processing callbacks
    while(ros::ok() && !quit_)
    {
        ros::spinOnce();
    }
    ROS_INFO("ObjectTrackin2D node stopped.");

    //Destroying Stuff
    cvDestroyWindow("ObjectTrackin2D:Result");
    cvDestroyWindow("ObjectTrackin2D:Edges");
    delete tracker_;
    if(user_input_thread_ != NULL)
    {
        user_input_thread_->stop();
    }

    return EXIT_SUCCESS;
}
