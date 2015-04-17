# Message storing raw 2D and 3D information about a single Object detection.
Header header
uint32 id	# id of Object (not necessarily unique)
string ns	# ns of Object (not necessarily unique)
bool good	# health status of detection and for command feedback
bool init	# init command for detection responce
geometry_msgs/Pose pose  # pose of the marker in the camera frame
