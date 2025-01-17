
#include "pose.h"
#include "string_manip.h"

Pose::Pose(){}

Pose::~Pose(){}

TF Pose::T_co() const
{
  return T_co_;
}


timestamp_t Pose::GetTimestamp() const
{ return time_; }



void Pose::Print(std::ostream& out) const
{
  out<<"Label: "<<this->GetLabel()<<endl;
  out<<"T_co: "<<endl<<this->T_co();

  if((this->pose_type_& SLAM_POSE) == SLAM_POSE)
  {
    out<<endl<<"T_co: "<<this->T_co();
  }


}

PoseType Pose::GetPoseType() const
{ return this->pose_type_; }

string Pose::GetLabel() const
{
  switch(this->pose_type_)
  {
    case LOST:
    {
      return "Lost";
    }

    case GPS_POSE:
    {
      return "GPS";
    }

    case SLAM_POSE:
    {
      return "SLAM";
    }

    case PNP_POSE:
    {
      return "PnP";
    }

    case OPTIMIZED_POSE:
    {
      return "Optimized";
    }

    case REFERENCE_POSE:
    {
      return "Reference";
    }

    default:
    {
      throw std::invalid_argument("Invalid Pose Type. No matching case.");
    }
  }
}

// Factories




Pose CreateSlamPose(TF current_T_co , timestamp_t time)
{
  Pose result;
  result.pose_type_ = SLAM_POSE;
  result.time_ = time;
  result.T_co_ = current_T_co;
  return result;
}


Pose CreateNonePose(timestamp_t time)
{
  Pose result;
  result.T_co_ = TFIdentity();
  result.pose_type_ = LOST;
  result.time_ = time;
  return result;
}

std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
  pose.Print(os);
  return os;
};