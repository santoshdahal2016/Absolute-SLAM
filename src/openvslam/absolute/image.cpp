//
// Created by Pascal Enderli on 03.10.18.
//

#include <Eigen/StdVector>
#include <fstream>

#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "image.h"


#include "string_manip.h"


#include "openvslam/system.h"

//Constructor
Image::Image()
{
  // Default value for pose.
  this->pose_cn_ = TFIdentity();
}

Image::Image(const Image&) = default;
Image::Image(Image&&) = default;
Image::~Image() {}


//Getters

Quaternion_t Image::GetQuaternion() const
{ return this->pose_cn_.GetQuaternion(); }


RotMat_t Image::GetRotMatrix() const
{ return this->pose_cn_.GetRotmatrix(); }


Translation_t Image::GetTranslation() const
{ return this->pose_cn_.GetTranslation(); }

/**
 * @details
 * @n Pose_cn describes the transformation from a normalized coordinate Frame to the camera Coordinate frame.
 * @n Meaning the how the Origin of the Normalized Coordinate Frame is seen from the Camera perpective.
 * @n Equivalent to the extrinsics representation.
 * @return TF Pose (orientation and position) of the camera expressed in camera coordinate frame.
 */
TF Image::Pose_cn() const
{ return std::move(this->pose_cn_); }

/**
 * @details
 * @n Pose_cn describes the transformation from a normalized coordinate Frame to the camera Coordinate frame.
 * @n Meaning the how the Origin of the Normalized Coordinate Frame is seen from the Camera perpective.
 * @n Equivalent to the extrinsics representation.
 * @param pose_cn Pose (orientation and position) of the camera expressed in camera coordinate frame.
 */
void Image::SetPose(TF pose_cn)
{ this->pose_cn_ = pose_cn; }




// Derived Class OXFORDImage
// Initialize static variable to count inserted Frames
unsigned long DATASETImage::count = 0;
unsigned long DATASETImage::count_alive = 0;

// Constructor
DATASETImage::DATASETImage()
{
  available_poses_ = LOST;
  this->count++;
  this->count_alive++;
}
DATASETImage::DATASETImage(const DATASETImage&) = default;

DATASETImage::DATASETImage(DATASETImage&&) = default;


DATASETImage::DATASETImage(const string& image_path ,unsigned long int timestamp)
{
  cv::TickMeter t;
  this->available_poses_ = LOST;
  string base_name = GetBaseName(image_path);
  this->name_ = base_name;
  this->path_ = image_path;
  this->timestamp_ = timestamp;
  this->image_id_ = count;

  // Read Image
  t.reset();
  t.start();
  try
  {
    this->cv_img_ = cv::imread( image_path, CV_LOAD_IMAGE_UNCHANGED);
  }

  catch(...)
  {
  }

  count++;
  count_alive++;
}


bool DATASETImage::SetSlamPose(openvslam::system& openv_slam_system)
{
  try
  {

    openvslam::Mat44_t cam_pose = openv_slam_system.feed_monocular_frame(this->cv_img_, this->timestamp_, cv::Mat{});
    
    RotMat_t r_co = cam_pose.block<3,3>(0,0);
    Translation_t t_co = cam_pose.block<3,1>(0, 3);
    TF T_co(r_co, t_co);
    // Get Transform from Orb slam.
                     
    this->slam_pose_co_ = T_co;
    this->slam_is_lost_ =  openv_slam_system.is_lost() || !openv_slam_system.is_ok();

    if(slam_is_lost_)
    {
      this->slam_pose_co_ = TFIdentity();
      return false;
    }

    else
    {
      this->available_poses_ = (this->available_poses_ | SLAM_POSE);
      return true;
    }
  }

  catch (const std::exception& e)
  {
    this->slam_is_lost_= true;
  }

  catch(...)
  {
    this->slam_is_lost_ = true;
  }
  return false;
}


DATASETImage::~DATASETImage()
{
  this->count_alive--;
}


std::string DATASETImage::GetFilePath() const
{
  return this->path_;
}
timestamp_t DATASETImage::GetTimestamp()
{
  return timestamp_;
}

TF DATASETImage::SlamPose_co() const
{
  return std::move(this->slam_pose_co_);
}

bool DATASETImage::SlamIsLost() const
{ return this->slam_is_lost_; }


uint8_t DATASETImage::GetAvailablePoseTypes() const
{ return available_poses_; }


