#ifndef _OBJECTDETECTION_H_
#define _OBJECTDETECTION_H_


/** Encapsulate the result of a object detection. */
class ObjectDetection {
public:

  /** Is the detection good enough? **/
  bool good;

  /** What was the ID of the detected object? **/
  size_t id;

  /** What was the pose of the detected object? **/
  Eigen::Matrix4d pose;
  
};

typedef std::vector<ObjectDetection> ObjectDetectionArray;

#endif

