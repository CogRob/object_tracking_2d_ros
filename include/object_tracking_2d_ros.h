/***********************************************************************

Copyright (c) 2014, Georgia Tech Research Corporation
Atlanta, Georgia 30332-0415
All Rights Reserved

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice, 
      this list of conditions and the following disclaimer in the documentation 
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holders nor the names of its contributors 
      may be used to endorse or promote products derived from this software without 
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

const double SMALL_TAG_SIZE = 0.0358968;
const double MED_TAG_SIZE = 0.06096;
const double PAGE_TAG_SIZE = 0.165;

const std::string DEFAULT_TAG_FAMILY = "Tag36h11";
const std::string DEFAULT_IMAGE_TOPIC = "image";
const std::string DEFAULT_CAMERA_INFO_TOPIC = "camera_info";
const std::string DEFAULT_MARKER_TOPIC = "marker_array";
const std::string DEFAULT_DETECTIONS_TOPIC = "detections";
const double DEFAULT_TAG_SIZE = MED_TAG_SIZE;
const std::string DEFAULT_DISPLAY_TYPE = "CUBE";

// ROS parts
ros::NodeHandlePtr node_;
boost::shared_ptr<image_transport::ImageTransport> image_;
sensor_msgs::CameraInfo camera_info_;

ros::Publisher marker_publisher_;
ros::Publisher apriltag_publisher_;
ros::Subscriber info_subscriber;
image_transport::Subscriber image_subscriber;

// AprilTag parts
TagFamily* family_;
TagDetector* detector_;

TagDetectorParams tag_params;
std::string tag_data;
std::string tag_family_name_;

// Settings and local information
int viewer_;
double default_tag_size_;
boost::unordered_map<size_t, double> tag_sizes_;
bool running_;
bool has_camera_info_;
std::string display_type_;

Eigen::Matrix4d GetDetectionTransform(TagDetection detection);
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
void ConnectCallback(const ros::SingleSubscriberPublisher& info);
void DisconnectCallback(const ros::SingleSubscriberPublisher& info);
void DisconnectHandler();
void GetParameterValues();
void SetupPublisher();
void InitializeTags();
void InitializeROSNode();
