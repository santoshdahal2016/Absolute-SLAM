
#ifndef OPENVSLAM_ABSOLUTE_LOGGER_H
#define OPENVSLAM_ABSOLUTE_LOGGER_H

#include <string>
#include <fstream>
#include "pose.h"

#include "yaml-cpp/yaml.h"
#include "transform.h"

class ResultLogger
{
public:
  ResultLogger();
  ResultLogger(string logfile_path, TF T_wn, string map_name, string images_name);
  ~ResultLogger();

  ResultLogger& operator=(const ResultLogger& other);

  void LogPose(unsigned int id, std::string image_name,Pose pose_opt, TF tf_co, int slam_map_nr,double s_no);

private:
  string file_path_;
  fstream out_file_;
};

YAML::Emitter& operator << (YAML::Emitter& out, const TF& tf);
#endif //OPENVSLAM_ABSOLUTE_LOGGER_H
