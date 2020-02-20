/**
 * @author Pascal Enderli
 * @date 01.11.2018
 * @file tranjectory,h
 * @brief datacontainer for the travelled trajectory.
 */

#ifndef OPENVSLAM_ABSOLUTE_TRAJECTORY_H
#define OPENVSLAM_ABSOLUTE_TRAJECTORY_H

#include "image.h"
#include "pose.h"


#include <vector>
#include <Eigen/StdVector>


/**
 * Stores Poses and takes care Reference is initialized correctly.
 */
class Trajectory
{
public:
  Trajectory();
  ~Trajectory();

  void ResetSlamState();


  bool AddSlamPose(DATASETImage& image);



  bool AddNonePose(DATASETImage& image);


  Pose GetLastPose() const;
  size_t Size() const;

 
  // i > 0 : behave like []
  // i < 0 : return from back: -1 is last element, -2 second before last etc.
  Pose at(int i) const;



  void Print(std::ostream& out) const;

private:

  void PushBack(Pose pose);


  // Major Data: vector of optimal chosen poses.
  std::vector<Pose, Eigen::aligned_allocator<Pose>> poses_;



};


// Print
std::ostream& operator<<(std::ostream& os, const Trajectory& trajectory);

#endif //OPENVSLAM_ABSOLUTE_TRAJECTORY_H
