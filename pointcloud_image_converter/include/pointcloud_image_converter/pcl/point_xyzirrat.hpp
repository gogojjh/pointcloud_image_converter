/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/
/** \brief Metafunction to check if a given point type has a given field.
 *
 *  Example usage at run-time:
 *
 *  \code
 *  bool curvature_available = pcl::traits::has_field<PointT,
 * pcl::fields::curvature>::value; \endcode
 *
 *  Example usage at compile-time:
 *
 *  \code
 *  BOOST_MPL_ASSERT_MSG ((pcl::traits::has_field<PointT,
 * pcl::fields::label>::value), POINT_TYPE_SHOULD_HAVE_LABEL_FIELD, (PointT));
 *  \endcode
 */

#define PCL_NO_PRECOMPILE

#ifndef POINTXYZIRRAT_HPP_
#define POINTXYZIRRAT_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <iomanip>
#include <ostream>

// pcl point type:
// https://github.com/PointCloudLibrary/pcl/blob/pcl-1.8.0/common/include/pcl/impl/point_types.hpp
namespace pcl {
struct EIGEN_ALIGN16 _PointXYZIRRAT {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t reflectivity;
  std::uint8_t ring;
  std::uint16_t ambient;  // additional property of p.ouster
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

struct PointXYZIRRAT : public _PointXYZIRRAT {
  inline PointXYZIRRAT() {
    x = y = z = 0.0f;
    intensity = 0.0f;
    reflectivity = 0;
    ring = 0;
    ambient = 0;
    time = 0.0f;
  }

  inline PointXYZIRRAT(float _x, float _y, float _z, float _intensity,
                       std::uint16_t _reflectivity, std::uint8_t _ring,
                       std::uint16_t _ambient, float _time) {
    x = _x;
    y = _y;
    z = _z;
    intensity = _intensity;
    reflectivity = _reflectivity;
    ring = _ring;
    ambient = _ambient;
    time = _time;
  }

  inline PointXYZIRRAT(const _PointXYZIRRAT &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    intensity = p.intensity;
    reflectivity = p.reflectivity;
    ring = p.ring;
    ambient = p.ambient;
    time = p.time;
  }

  friend std::ostream &operator<<(std::ostream &out, const PointXYZIRRAT &p);
};

inline std::ostream &operator<<(std::ostream &out, const PointXYZIRRAT &p) {
  out << std::fixed;
  out << std::setprecision(3);
  out << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.intensity << ", "
      << ", " << p.reflectivity << ", " << p.ring << ", " << p.ambient << ", "
      << p.time << ")";
  return out;
}
}  // namespace pcl

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRRAT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (float, time, time))

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZIRRAT, pcl::_PointXYZIRRAT)
// clang-format on

#endif  // POINTXYZIRRAT_HPP_
