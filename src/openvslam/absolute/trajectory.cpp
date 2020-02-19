//
// Created by Pascal Enderli on 02.11.18.
//

#include <glog/logging.h>

#include "trajectory.h"
#include "optimize.h"
#include "string_manip.h"

Trajectory::Trajectory()
{
  this->reference_is_set_ = false;

  Coordinate3D_t init_coordinate_zero; init_coordinate_zero<<0, 0, 0;

}

Trajectory::~Trajectory() {}



void Trajectory::ResetSlamState()
{
  LOG(WARNING)<<"[WARNING] ORB Slam Reseted its map. Absolute Slam will Follow.";
  this->reference_is_set_ = false;
  this->T_no_is_set = false;
}


bool Trajectory::AddNonePose(OXFORDImage& image)
{
  Pose pose = CreateNonePose(image.GetTimestamp());

  this->T_no_is_set = false;
  this->reference_is_set_ = false;

  this->PushBack(pose);

  LOG(INFO)<<"[Info] AddedNone Pose";
  return true;
}

bool Trajectory::AddSlamPose(OXFORDImage& image)
{
  if((image.GetAvailablePoseTypes() & SLAM_POSE) != SLAM_POSE)
  {
    LOG(INFO)<<"[Info] SLAM Pose Tag in image is not set.";
    return false;
  }

  if(!this->T_no_is_set)
  {
    LOG(INFO)<<"[Info] T_no_is_set is false";
    return false;
  }
  Pose pose = CreateSlamPose(this->GetReferencePose().T_co(), image.SlamPose_co(), this->T_no(), image.GetTimestamp());
  this->PushBack(pose);

  LOG(INFO)<<"[Info] Added Slam Pose ";
  return true;
}

bool Trajectory::AddOptimizedPose(OXFORDImage& image, Camera camera, ReferenceConditions& reference_conditions, OptimizationOptions& optimization_opts)
{

  // reference already available -> check if new pose is also reference
  // reference not available -> try to initialize Reference.
  if( !this->reference_is_set_ )
  {
    LOG(INFO)<<"[Info] [AddOptimizedPose] Reference is not set";
    return false;
  }

  // Check if at least PNP_POSE and SLAM_POSE are available
  if( (image.GetAvailablePoseTypes() & (PNP_POSE | SLAM_POSE)) !=  (PNP_POSE | SLAM_POSE))
  {
    LOG(INFO)<<"[Info] [AddOptimizedPose] PnP or Slam pose not available";
    return false;
  }


  if(image.GetMatches().size() < reference_conditions.min_n_matches)
  {
    LOG(INFO)<<"[Info] [AddOptimizedPose] Not enough n inlier matches: n_matches "<<image.GetMatches().size()<<" < "<<"threshold "<< reference_conditions.min_n_matches;
    return false;
  }

  // new pose is also a reference -> optimize
  // new pose is not a reference -> take next better pose estimation: 1)slam 2)pnp
  if( !IsReference(image.PnPPose_cn(), image.GetTimestamp(), reference_conditions))
  {
    return false;
  }

  // T_no is available -> optimize
  // T_no is not available -> initialize T_no first.
  if(!this->T_no_is_set)
  {
    double initialized_scale_no = InitializeScale_no(this->GetReferencePose().T_cn(), this->GetReferencePose().T_co(), image.PnPPose_cn(), image.SlamPose_co());
    initialized_scale_no = initialized_scale_no;
    this->UpdateT_no(initialized_scale_no);
  }


  TF T_pnp_nc = image.PnPPose_cn().Inv();
  TF o_T_cRef_cCurrent = this->GetReferencePose().T_co() * image.SlamPose_co().Inv();
  TF T_slam_nc = this->T_no() * o_T_cRef_cCurrent;


  // if slam and pnp drifted to much apart, it will not be optimized but a new reference is set to the pnp pose. since in optimization no valid transform could be found.
  // (here the first pose of a trinagle is resetted.)

  TF T_pnp_wc = global_map.T_wn() * T_pnp_nc;
  TF T_slam_wc = global_map.T_wn() * T_slam_nc;
  if(GetDistanceTranslation(T_pnp_wc.GetTranslation(), T_slam_wc.GetTranslation())>optimization_opts.max_pnp_slam_pose_distance)
  {
    ResetReferencePose(image);
    LOG(INFO)<<"[Info] Distance ("<<GetDistanceTranslation(T_pnp_wc.GetTranslation(), T_slam_wc.GetTranslation())<<") between pnp and slam as bigger than threshold ("<<optimization_opts.max_pnp_slam_pose_distance<<") : Reseted Reference Pose on new location with old scale s_no.";
    return true;
  }


  // Optimization

  std::tuple<TF, double, double> optim_result = Optimize(this->GetReferencePose().T_cn(), image.PnPPose_cn(), this->T_no().GetScale(), o_T_cRef_cCurrent, this->GetReferencePose().GetMatches(), image.GetMatches(), camera);

  if(std::get<2>(optim_result) > reference_conditions.max_reprojection_error)
  {
    LOG(INFO)<<"[Info] [AddOptimizedPose] Reprojection Error ( "<< std::get<2>(optim_result)<<" ) is bigger than threshold:( "<< reference_conditions.max_reprojection_error <<" )";
    //return false;
  }

  TF optimal_T_cn = std::get<0>(optim_result);

  double optimal_scale_no = std::get<1>(optim_result);
  if(GetReferencePose().GetPoseType()!=PoseType::REFERENCE_POSE)
  {
    optimal_scale_no = filter_.Filter(std::get<1>(optim_result), 1e-6 * image.GetTimestamp());
  }

  ScaledTF T_no = ComposeT_no(optimal_T_cn, optimal_scale_no);
  Pose pose = CreateOptimizedPose(optimal_T_cn, image.SlamPose_co(), T_no, image.GetMatches(), image.GetTimestamp());
  this->PushBack(pose);
  this->T_no_is_set = true;
  LOG(INFO)<<"[Info] Added Optimized Pose";
  return true;
}

/// Reference new reference location with old scale
void Trajectory::ResetReferencePose(OXFORDImage & image)
{
  double cache_s_no = GetReferencePose().T_no().GetScale();
  Pose pose = CreateReferencePose(image.PnPPose_cn(), image.SlamPose_co(), image.GetMatches(), image.GetTimestamp());
  this->PushBack(pose);

  this->UpdateT_no(cache_s_no);
}

bool Trajectory::InitializeReferencePose(OXFORDImage& image, ReferenceConditions& reference_conditions)
{
  // Check if at least PNP_POSE and SLAM_POSE are available
  if( (image.GetAvailablePoseTypes() & (PNP_POSE | SLAM_POSE)) !=  (PNP_POSE | SLAM_POSE))
  {
    return false;
  }

  if(this->reference_is_set_)
  {
    LOG(INFO)<<"[Info] Reference is already initialized.";
    return false;
  }

  LOG(INFO)<<"[Info] Reference Condition n inlier matches: "<<image.GetMatches().size();
  if(image.GetMatches().size() < reference_conditions.min_n_matches)
  {
    return false;
  }

  if(!this->IsReference(image.PnPPose_cn(), image.GetTimestamp(), reference_conditions))
  {
    return false;
  }

  // T_no will be set later in the Add Optimized Pose Function
  Pose pose = CreateReferencePose(image.PnPPose_cn(), image.SlamPose_co(), image.GetMatches(), image.GetTimestamp());
  this->PushBack(pose);

  LOG(INFO)<<"[Info] Initialized Reference without Scale s_no.";
  return true;
}

void Trajectory::PushBack(Pose pose)
{
  poses_.push_back(pose);

  if((pose.GetPoseType() & REFERENCE_POSE) == REFERENCE_POSE)
  {
    UpdateReferencePose(poses_.size() -1 );
  }

}

Pose Trajectory::GetReferencePose() const
{
  CHECK(this->reference_is_set_);
  return this->poses_.at(this->reference_pose_idx_);
}

ScaledTF Trajectory::T_no() const
{
  CHECK(this->T_no_is_set);
  return this->poses_[this->reference_pose_idx_].T_no();
}

bool Trajectory::TnoIsSet() const
{ return this->T_no_is_set;}

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

float Trajectory::GetSpeed() const
{
  if(this->Size() < 5) {return 0;}
  Pose p1 = this->at(-5);
  Pose p2 = this->at(-1);

  // time in seconds
  double delta_t = 1e-6 * 1/3600.0 * (double)(p2.GetTimestamp() - p1.GetTimestamp());
  // distance in normalized kilometers. (denormalization outside of function.)
  double distance = 1e-3 * GetDistanceTranslation(p1.T_cn().Inv().GetTranslation(), p2.T_cn().Inv().GetTranslation());

  return distance/delta_t;
}

size_t Trajectory::Size() const
{
  return this->poses_.size();
}

void Trajectory::UpdateT_no(double scale_no)
{
  // Transform orbslam Coordinate frame to the last Reference Point. T_current_co_ref

  ScaledTF T_current_no = ComposeT_no(this->GetReferencePose().T_cn(), scale_no);
  this->poses_[this->reference_pose_idx_].Set_T_no(T_current_no);
  this->T_no_is_set = true;
}

void Trajectory::UpdateReferencePose(unsigned int ref_idx)
{
  this->reference_pose_idx_ = ref_idx;
  this->reference_is_set_ = true;
}

bool Trajectory::PnpQueueIsFull() const
{
  for(int i=0; i<3; ++i)
  {
    if(this->pnp_queue_.at(i).second == 0) {return false;}
  }
  return true;
}

void Trajectory::UpdatePnPQueue(const TF& p_nc, const timestamp_t time)
{
  // most current pnp pose will always be at position 0.
  // Check if the incoming pnp pose is newer than the most current one in the queue. (return if not)
  if(time <= pnp_queue_.at(0).second) {return;}

  // Rotation to the right
  std::rotate(pnp_queue_.rbegin(), pnp_queue_.rbegin() + 1, pnp_queue_.rend());

  pnp_queue_.at(0).first = p_nc;
  pnp_queue_.at(0).second = time;
}

void Trajectory::Print(std::ostream& out) const
{
  out<<"Reference is set: "<<reference_is_set_<<endl;
  if(reference_is_set_)
  {
    out<<"Reference idx: "<<reference_pose_idx_<<endl;
  }

  out<<"T_no is set: "<<T_no_is_set;
}

bool Trajectory::IsReference(const TF& pnp_pose_cn, const timestamp_t time, const ReferenceConditions& reference_conditions)
{ // A Reference Pose shall not depend on relative poses, but rather on optimized and pnp ones. otherwise a a drift could distroy the condition and the robot is captured (will never find a reference anymore => no optimization anymore will only use slam pose for rest of lifetime.)
  // Whenever 3 PnP poses have almost same distance to each other: => const speed = pnp points does not come from different map parts-
  // And the moving angle between the tree points is small: no measurement outlier
  // it is considered as a Reference point. (reason this critical point must be of good quality)

  // Maintain pnp pose queue.
  UpdatePnPQueue(pnp_pose_cn.Inv(), time);

  if(!PnpQueueIsFull()) {return false;}

  // Estimate how much the camera moved currently.
  double travelled_distance1 = GetDistanceTranslation(pnp_queue_.at(1).first.GetTranslation() , pnp_queue_.at(2).first.GetTranslation());
  double travelled_distance2 = GetDistanceTranslation(pnp_queue_.at(0).first.GetTranslation() , pnp_queue_.at(1).first.GetTranslation());
  double ratio = travelled_distance1/travelled_distance2;
  double th = 1+reference_conditions.length_tolerance; // up to 1.2 times a slong is accepted.

  //Estimate moving angle
  double moving_angle = GetAngleBetweenVectors(pnp_queue_.at(1).first.GetTranslation() - pnp_queue_.at(2).first.GetTranslation(), pnp_queue_.at(0).first.GetTranslation() - pnp_queue_.at(1).first.GetTranslation());

  double heading_change = GetDistanceRotation(pnp_queue_.at(0).first.GetQuaternion(), pnp_queue_.at(1).first.GetQuaternion()) - GetDistanceRotation(pnp_queue_.at(1).first.GetQuaternion(), pnp_queue_.at(2).first.GetQuaternion());

  bool success;
  success = !(ratio > th || ratio < 1/th);
  success = success && (moving_angle < reference_conditions.moving_angle_tolerance);
  success = success && (std::abs(heading_change) < reference_conditions.heading_change_tolerance);

  LOG(INFO)<<"[Info]last 3 PnP poses:"<<endl
  <<"       "<<pnp_queue_.at(0).first.GetTranslation()[0] <<"; "<<pnp_queue_.at(1).first.GetTranslation()[0] << "; "<< pnp_queue_.at(2).first.GetTranslation()[0] <<endl
  <<"       "<<pnp_queue_.at(0).first.GetTranslation()[1] <<"; "<<pnp_queue_.at(1).first.GetTranslation()[1] << "; "<< pnp_queue_.at(2).first.GetTranslation()[1] <<endl
  <<"       "<<pnp_queue_.at(0).first.GetTranslation()[2] <<"; "<<pnp_queue_.at(1).first.GetTranslation()[2] << "; "<< pnp_queue_.at(2).first.GetTranslation()[2] <<endl;

  LOG(INFO)<<"[Info] Reference Condition Ratio: "<<ratio<<"\t\t"<<"angle: "<<moving_angle<<"\t\t"<<"heading_change: "<<heading_change<<"\t\t Success: "<<success<<endl;
  return success;
}

double InitializeScale_no(const TF& pnp_pose1_cn, const TF& slam_pose1_co, const TF& pnp_pose2_cn, const TF& slam_pose2_co)
{
  // Scale
  TF T_pnp_c1c2 = pnp_pose1_cn * pnp_pose2_cn.Inv();
  TF T_slam_c1c2 = slam_pose1_co * slam_pose2_co.Inv();
  double s_no = T_pnp_c1c2.GetTranslation().norm()/T_slam_c1c2.GetTranslation().norm();

  return s_no;
}

ScaledTF ComposeT_no(const TF& T_reference_cn, const double scale_no)
{
  return ScaledTF(T_reference_cn.Inv(), scale_no);
}

// Print
std::ostream& operator<<(std::ostream& os, const Trajectory& trajectory)
{
  trajectory.Print(os);
  return os;
};