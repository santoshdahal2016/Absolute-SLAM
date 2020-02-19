#ifndef OPENVSLAM_ABSOLUTE_IMAGE_H
#define OPENVSLAM_ABSOLUTE_IMAGE_H


#include <Eigen/Dense>


#include "transform.h"
#include "typedefs.h"

#include "openvslam/system.h"

using namespace std;


class Image
{
public:
  //Constructor
  Image();
  Image(const Image&);
  Image(Image&&);
  virtual ~Image();

  //Getters
  Quaternion_t GetQuaternion() const;
  RotMat_t GetRotMatrix() const;
  Translation_t GetTranslation() const;

  /// @brief Set Camera Pose as Transformation Pose_cn.
  TF Pose_cn() const;

  // Makro because of eigen member
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Image& operator=(const Image&)=default;
  Image& operator=(Image&&)=default;

protected:
  //Setters
  /// @brief Set Camera Pose as Transformation Pose_cn.
  void SetPose(TF pose_cn);

private:
  // Pose of the image is transformation from normalized to camera Coordinate frame. T_cn
  TF pose_cn_;

protected:
  string name_;
  image_t image_id_;


};


// Derived Class for  dataset images
class DATASETImage: public Image
{
public:

  // Constructors
  DATASETImage();
  DATASETImage(const DATASETImage&);
  DATASETImage(DATASETImage&&);
  DATASETImage(const string& image_path , unsigned long int timestamp);
  ~DATASETImage();

  bool SetSlamPose(openvslam::system& openv_slam_system);


  /// @brief Get the Pose of the Image gathered py the SLAM

  TF SlamPose_co() const;

  bool SlamIsLost() const;

  // Get a bitmasked Composition of what pose types are available.
  uint8_t GetAvailablePoseTypes() const;
  std::string GetFilePath() const;
  timestamp_t GetTimestamp();


  // Operators
  /// @brief Copy assignment.
  DATASETImage& operator=(const DATASETImage&)=default;

  /// @brief Move assignment.
  DATASETImage& operator=(DATASETImage&&)=default;


private:
  static unsigned long count;
  static unsigned long count_alive;

  cv::Mat cv_img_;

  TF slam_pose_co_;

  // Bitmask from enum Available_Poses
  uint8_t available_poses_;

  bool slam_is_lost_;

  std::string path_;

  unsigned long int timestamp_;

};


#endif //OPENVSLAM_ABSOLUTE_IMAGE_H
