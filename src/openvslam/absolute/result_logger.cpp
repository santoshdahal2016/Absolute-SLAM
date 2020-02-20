
#include <ctime>
#include <sstream>
#include <iomanip>
#include "result_logger.h"
#include "filesystem.h"
#include "transform.h"

ResultLogger::ResultLogger() {}

ResultLogger::ResultLogger(string logfile_path, TF T_wn, string map_name, string images_name)
{
  file_path_ = logfile_path;

  if((GetFileExtension(logfile_path) == "yaml") || (GetFileExtension(logfile_path) == "yml"))
  {
    out_file_.open(logfile_path, ios::out|ios::trunc);
  }
  else
  {
  }

  std::time_t timestamp = std::time(nullptr);
  std::stringstream time_stream; time_stream << std::put_time(std::localtime(&timestamp), "%c %Z");

  YAML::Emitter out_yaml_node_;

  out_yaml_node_<<YAML::Comment("This Yaml file contains Results of Absolute Slam.")<<YAML::Newline
                <<YAML::Comment("The Structure is as follows:")<<YAML::Newline
                <<YAML::Comment("id: A unique identifier of the pose.")<<YAML::Newline
                <<YAML::Comment("image_name: The name of the corresponding image on Disk.")<<YAML::Newline
                <<YAML::Comment("Pose Contians: The type of the pose [GPS, Reference, SLAM, pnp, none].")<<YAML::Newline
                <<YAML::Comment("               The Position and orientation of that optimal pose.")<<YAML::Newline
                <<YAML::Comment("               The Position and orientation of the slam pose (in orbslam Coordinate frame extrinsics).")<<YAML::Newline
                <<YAML::Comment("               The Position and orientation of the gps pose (in normalized Coordinate frame extrinsics).")<<YAML::Newline
                <<YAML::Comment("               The Scalar between the orb slam map and the normalized map if this pose is optimized (new scale calculated).")<<YAML::Newline
                <<YAML::Comment("If OrbSlam Looses Track and reinitializes, slam_map_id coiunts +1 To signalize, that the sytem runs with a new orbslam coordinate frame.")<<YAML::Newline
                <<YAML::Comment("The first sequence entry (id = -1) contains metadata such as the transform between ecef coordinate frame and normalized coordinate frame.")<<YAML::Newline<<YAML::Newline;


  out_yaml_node_<<YAML::BeginSeq;
    out_yaml_node_<<YAML::BeginMap;
      out_yaml_node_<<YAML::Key<<"id";
      out_yaml_node_<<YAML::Value<<-1;

      out_yaml_node_<<YAML::Key<<"DateTime";
      out_yaml_node_<<YAML::Value<<time_stream.str();

      out_yaml_node_<<YAML::Key<<"global_map_name";
      out_yaml_node_<<YAML::Value<<map_name;

      out_yaml_node_<<YAML::Key<<"query_images_name";
      out_yaml_node_<<YAML::Value<<images_name;

      out_yaml_node_<<YAML::Key<<"T_wn";
      out_yaml_node_<<YAML::Value<<T_wn;

    out_yaml_node_<<YAML::EndMap;
  out_yaml_node_<<YAML::EndSeq<<YAML::Newline;

  out_file_<<"%YAML 1.2"<<endl<<"---"<<endl<<endl<<out_yaml_node_.c_str()<<endl;
  out_file_.flush();


}

ResultLogger::~ResultLogger()
{
  out_file_.close();
}

YAML::Emitter& operator << (YAML::Emitter& out, const TF& tf)
{
  out<<YAML::BeginMap;

  out<<YAML::Key<<"q";
  out<<YAML::Value<<YAML::Flow<<YAML::BeginSeq
    <<tf.GetQuaternion()[0]
    <<tf.GetQuaternion()[1]
    <<tf.GetQuaternion()[2]
    <<tf.GetQuaternion()[3]
  <<YAML::EndSeq;

  out<<YAML::Key<<"t";
  out<<YAML::Value<<YAML::Flow<<YAML::BeginSeq
    <<tf.GetTranslation()[0]
    <<tf.GetTranslation()[1]
    <<tf.GetTranslation()[2]
  <<YAML::EndSeq;

  out<<YAML::EndMap;
  return out;
}


void ResultLogger::LogPose(unsigned int id, std::string image_name, Pose pose_opt, TF tf_co, int slam_map_nr, double s_no)
{
  YAML::Emitter out_yaml_node_;

  out_yaml_node_<<YAML::BeginSeq;
    out_yaml_node_<<YAML::BeginMap;
      out_yaml_node_<<YAML::Key<<"id";
      out_yaml_node_<<YAML::Value<<id;

      out_yaml_node_<<YAML::Key<<"image_name";
      out_yaml_node_<<YAML::Value<<image_name;

      out_yaml_node_<<YAML::Key<<"label";
      out_yaml_node_<<YAML::Value<<pose_opt.GetLabel();


      out_yaml_node_<<YAML::Key<<"T_slam_co";
      out_yaml_node_<<YAML::Value<<tf_co;

      out_yaml_node_<<YAML::Key<<"slam_map_id";
      out_yaml_node_<<YAML::Value<<slam_map_nr;

      out_yaml_node_<<YAML::Key<<"s_no";
      out_yaml_node_<<YAML::Value<<s_no;

    out_yaml_node_<<YAML::EndMap;
  out_yaml_node_<<YAML::EndSeq;

  // Print the constructed YAML Sequence directly to a file.
  out_file_<<out_yaml_node_.c_str()<<endl<<endl;
  out_file_.flush();
}

ResultLogger& ResultLogger::operator=(const ResultLogger& other)
{
  this->file_path_ = other.file_path_;


  this->out_file_.open(other.file_path_, ios::out|ios::app);


  return *this;
}

