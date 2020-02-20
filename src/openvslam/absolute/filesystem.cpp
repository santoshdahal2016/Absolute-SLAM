
#include "filesystem.h"
#include <algorithm>


std::vector<std::string> GetDirContent(std::string path)
{
  fs::path p(path);
  CheckPath(p);

  fs::recursive_directory_iterator begin(p), end;
  std::vector<fs::directory_entry> v(begin, end);
  size_t n_files = v.size();
  std::vector<std::string> paths(n_files);

  std::sort(v.begin(), v.end());
  for(unsigned int i=0; i<n_files; ++i)
  {
    paths.at(i) = v.at(i).path().string();
  }

return paths;
}

bool FileExist(const std::string& path)
{
  return boost::filesystem::is_regular_file(path);
}

bool DirExist(const std::string& path)
{
  return boost::filesystem::is_directory(path);
}

void CheckPath(fs::path path)
{
  fs::path p(path);
  if(!exists(p) || !is_directory(p))
  {
    throw std::invalid_argument("Path:\n"+path.string()+"\n is not valid.");
  }
}

std::string GetFileExtension(std::string path)
{
  return path.substr(path.find_last_of(".") + 1);
}