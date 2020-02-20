
#include "trajectory.h"

#include "string_manip.h"

Trajectory::Trajectory()
{


}

Trajectory::~Trajectory() {}



void Trajectory::ResetSlamState()
{

}


bool Trajectory::AddNonePose(DATASETImage& image)
{
  Pose pose = CreateNonePose(image.GetTimestamp());
  this->PushBack(pose);

  return true;
}

bool Trajectory::AddSlamPose(DATASETImage& image)
{
  if((image.GetAvailablePoseTypes() & SLAM_POSE) != SLAM_POSE)
  {
    return false;
  }

  Pose pose = CreateSlamPose(image.SlamPose_co(), image.GetTimestamp());
  this->PushBack(pose);

  return true;
}



void Trajectory::PushBack(Pose pose)
{
  poses_.push_back(pose);

 

}


Pose Trajectory::GetLastPose() const
{
  return this->poses_.back();
}

Pose Trajectory::at(int i) const
{
  if(i>=0)
  {
    return this->poses_[i];
  }
  else
  {
    return this->poses_.end()[i];
  }
}



size_t Trajectory::Size() const
{
  return this->poses_.size();
}

void Trajectory::Print(std::ostream& out) const
{

}



// Print
std::ostream& operator<<(std::ostream& os, const Trajectory& trajectory)
{
  trajectory.Print(os);
  return os;
};