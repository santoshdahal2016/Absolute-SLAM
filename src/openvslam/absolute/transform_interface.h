
#ifndef OPENVSLAM_ABSOLUTE_TRANSFORM_INTERFACE_H
#define OPENVSLAM_ABSOLUTE_TRANSFORM_INTERFACE_H
#include <vector>


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <type_traits>

#include <iostream>

/// 4x4 [Homogenious Transformation Matrix](https://www.brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html")
using HomogeniousTF_t = Eigen::Matrix<double, 4, 4>;
/// 3x3 [Rotation Matrix](https://en.wikipedia.org/wiki/Rotation_matrix)
using RotMat_t        = Eigen::Matrix<double, 3, 3>;
/// 3x1 Vector [x,y,z]<SUP>T</SUP>
using Translation_t   = Eigen::Matrix<double, 3, 1>;
/// 4x1 Vector [w,x,y,z]<SUP>T</SUP>
using Quaternion_t    = Eigen::Matrix<double, 4, 1>;
/// 4x1 Vector [x,y,z,theta]<SUP>T</SUP>
using AxisAngle_t    = Eigen::Matrix<double, 4, 1>;
/// 2x1 Vector [x,y]<SUP>T</SUP>
using Coordinate2D_t  = Eigen::Matrix<double, 2, 1>;
/// 3x1 Vector [x,y,z]<SUP>T</SUP>
using Coordinate3D_t  = Eigen::Matrix<double, 3, 1>;
/// 3x1 Vector [lon,lat,h]<SUP>T</SUP>
using geodethic3D_t   = Eigen::Matrix<double, 3, 1>;

/// Pi constant
const double pi = 3.1415926535897932384626433832795028841971;

template<typename T>
class TransformInterface
{
public:

  /// @brief Set this Transform to Identity.
  virtual void SetIdentity() = 0;

  /// @brief Multiply two Transforms of same Type
  virtual T operator*(const T& other) const = 0;

  /// @brief Multiplication.
  virtual T& operator*=(const T& other) = 0;

  /// @brief Copy assignment.
  virtual T& operator=(const T&) = 0;

  /// @brief Move assignment.
  virtual T& operator=(T&&) = 0;

  /// @brief Eqiality comparison
  virtual bool operator==(const T& other) = 0;

  /// @brief ineqiality comparison
  virtual bool operator!=(const T& other) = 0;

  friend std::ostream& operator<<(std::ostream& os, const T& transform)
  {
    transform.Print(os);
    return os;
  };

private:
  virtual void Print( std::ostream& out ) const = 0;

};


/// @brief Multiplication if the right hand side is a vector of Transforms/Derivatives.
template<typename TL, typename TR>
std::vector<TR> operator*(const TL lhs, const std::vector<TR>& rhs)
{
  static_assert(std::is_base_of<TransformInterface<TL>, TL>::value, "Left hand side is not derived from TransformInterface!!");
  static_assert(std::is_base_of<TransformInterface<TR>, TR>::value, "Right hand side Vector Elements are not derived from TransformInterface!!");

  size_t n_elements = rhs.size();
  std::vector<TR> result(n_elements);
  for(unsigned int i=0; i<n_elements; ++i)
  {
    result[i] = lhs * rhs[i];
  }
  return result;
}
#endif //OPENVSLAM_ABSOLUTE_TRANSFORM_INTERFACE_H