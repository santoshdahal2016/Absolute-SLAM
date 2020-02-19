
#include <glog/logging.h>
#include "pose.h"
#include "string_manip.h"

Pose::Pose(){}

Pose::~Pose(){}

TF Pose::T_co() const
{
  CHECK(((this->pose_type_& SLAM_POSE) == SLAM_POSE))<<"Pose Type value is: "<<this->pose_type_;
  return T_co_;
}


timestamp_t Pose::GetTimestamp() const
{ return time_; }


void Pose::Set_T_no(ScaledTF T_no)
{
  CHECK((this->pose_type_& REFERENCE_POSE) == REFERENCE_POSE)<<"Pose Type value is: "<<this->pose_type_;
  this->T_no_ = T_no;
}

void Pose::Print(std::ostream& out) const
{
  out<<"Label: "<<this->GetLabel()<<endl;
  out<<"T_cn: "<<endl<<this->T_cn();

  if((this->pose_type_& SLAM_POSE) == SLAM_POSE)
  {
    out<<endl<<"T_co: "<<this->T_co();
  }

  if((this->pose_type_& REFERENCE_POSE) == REFERENCE_POSE)
  {
    out<<endl<<"T_no: "<<this->T_no();
  }

  if((this->pose_type_& PNP_POSE) == PNP_POSE)
  {
    out<<endl<<"n_matches: "<<this->GetMatches().size();
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




Pose CreateSlamPose(TF reference_T_co, TF current_T_co , timestamp_t time)
{
  Pose result;
  result.pose_type_ = SLAM_POSE;
  result.time_ = time;

  // Find relative transform from reference pose to current pose.
  TF o_T_cRef_cCurrent = reference_T_co * current_T_co.Inv();

  TF T_nc = o_T_cRef_cCurrent;
  result.T_cn_ = T_nc.Inv();
  return result;
}

Pose CreateReferencePose(TF T_co, timestamp_t time)
{
  Pose result;
  result.pose_type_ = REFERENCE_POSE;
  result.time_ = time;
  result.T_co_ = T_co;
  return result;
}

Pose CreateNonePose(timestamp_t time)
{
  Pose result;
  result.T_cn_ = TFIdentity();
  result.pose_type_ = LOST;
  result.time_ = time;
  return result;
}

std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
  pose.Print(os);
  return os;
};