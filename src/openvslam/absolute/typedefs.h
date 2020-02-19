
#ifndef ABSOLUTE_SLAM_TYPEDEFS_H
#define ABSOLUTE_SLAM_TYPEDEFS_H

#include <limits>
#include <Eigen/Dense>
#include "opencv2/core.hpp"

using namespace std;

using camera_t = uint32_t;
using image_t = uint32_t;

const camera_t kInvalidCameraId = std::numeric_limits<camera_t>::max();
const image_t kInvalidImageId = std::numeric_limits<image_t>::max();

using timestamp_t = unsigned long int;

enum PoseType : uint8_t
{
  INVALID = 0,
  LOST = 0,

  GPS_POSE = 1,
  PNP_POSE = 2,
  SLAM_POSE = 4,

  // A Pose is automatically a reference pose iff (PNP_POSE | SLAM_POSE | GPS_POSE) flags are set 2+4 = 6.
  REFERENCE_POSE = 4 + 2 + 1, // = 7

  // Optimized Pose is also a Reference Pose
  OPTIMIZED_POSE = 8 + 7 // = 15

};

// get opencv matrix type
// string ty =  CVTypeToStr( M.type() );
// printf("Matrix: %s %dx%d \n", ty.c_str(), M.cols, M.rows );
string CVTypeToStr(int type);




#endif //ABSOLUTE_SLAM_TYPEDEFS_H
