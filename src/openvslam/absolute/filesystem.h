#ifndef OPENVSLAM_ABSOLUTE_FILESYSTEM_H
#define OPENVSLAM_ABSOLUTE_FILESYSTEM_H


#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <iostream>


namespace fs = boost::filesystem;

void CheckPath(fs::path path);

/// @brief Return all filepaths of inside a directory
std::vector<std::string> GetDirContent(std::string path);

bool FileExist(const std::string& path);
bool DirExist(const std::string& path);

std::string GetFileExtension(std::string path);
#endif //ABSOLUTE_SLAM_FILESYSTEM_H
