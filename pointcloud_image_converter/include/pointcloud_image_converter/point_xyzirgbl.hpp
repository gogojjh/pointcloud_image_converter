#ifndef POINT_XYZIRGBL_HPP_
#define POINT_XYZIRGBL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

namespace pcl {
struct EIGEN_ALIGN16 PointXYZIRGBL {
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed
                    // using the point (which is float[4])
  PCL_ADD_INTENSITY;
  PCL_ADD_RGB;
  std::uint16_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRGBL,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, rgba, rgba)
                                 (std::uint16_t, label, label))
// clang-format on

#endif  // POINT_XYZIRGBL_HPP_