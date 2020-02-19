#ifndef OPENVSLAM_ABSOLUTE_POSE_H
#define OPENVSLAM_ABSOLUTE_POSE_H


#include "transform.h"
#include "typedefs.h"


class Pose {

public:

  Pose();
  ~Pose();

  // Getter
  TF T_co() const;

  timestamp_t GetTimestamp() const;
  PoseType GetPoseType() const;
  string GetLabel() const;


  void Print(std::ostream& out) const;

  // Factory
  friend Pose CreateSlamPose(TF reference_T_co, TF current_T_co, timestamp_t time);
  friend Pose CreateReferencePose(TF T_co, timestamp_t time);
  friend Pose CreateNonePose(timestamp_t time);

private:


  // Reserved for Reference and optimized poses. used for optimization process.
  TF T_co_; // Pose from slam in orb slam frame.

  // Metadata for all kinds of poses
  PoseType pose_type_;
  timestamp_t time_;



};

// Factories

Pose CreateSlamPose(TF reference_T_co, TF current_T_co, timestamp_t time);
Pose CreateReferencePose(TF T_cn, TF T_co , timestamp_t time);
Pose CreateNonePose(timestamp_t time);

// Print
std::ostream& operator<<(std::ostream& os, const Pose& pose);

#endif //OPENVSLAM_ABSOLUTE_POSE_H