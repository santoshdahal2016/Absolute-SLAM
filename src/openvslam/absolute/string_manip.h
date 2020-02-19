
#ifndef STRING_MANIP_H_
#define STRING_MANIP_H_

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

bool IsNotWhiteSpace(const int character);

// Replace all occurrences of `old_str` with `new_str` in the given string.
std::string StringReplace(const std::string& str, const std::string& old_str,
                          const std::string& new_str);

// Split string into list of words using the given delimiters.
std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim);

// Check whether a string starts with a certain prefix.
bool StringStartsWith(const std::string& str, const std::string& prefix);

// Remove whitespace from string on both, left, or right sides.
void StringTrim(std::string* str);
void StringLeftTrim(std::string* str);
void StringRightTrim(std::string* str);

// Convert string to lower/upper case.
void StringToLower(std::string* str);
void StringToUpper(std::string* str);

// Check whether the sub-string is contained in the given string.
bool StringContains(const std::string& str, const std::string& sub_str);


// Author Pascal Enderli
string GetBaseName(const string& full_path);
string RemoveExtension(const string& base_name);
string GetFileExtension(const string& base_name);


// Pretty print
namespace pprint
{
  const string line = "-----------------------------------------";

  template<typename  T>
  void pprint(T var, string title = " "){cout<<title<<endl<<var<<endl<<line<<endl;}

  template<typename  T>
  void pprint(std::vector<T> vec, string title = " ")
  {
    using iterator = typename std::vector<T>::const_iterator ;
    cout<<title<<endl;
    for(iterator i = vec.begin(); i != vec.end(); ++i)
    {
      std::cout << *i << endl;
    }
    cout<<endl<<line<<endl;
  }

}

#endif  // STRING_MANIP_H_
