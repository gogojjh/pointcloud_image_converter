#ifndef PARAMS_HELPER_H_
#define PARAMS_HELPER_H_

#pragma once

#include <ros/ros.h>

#include <iostream>
#include <string>

namespace pc_img_conv {

template <typename T>
T get_ros_param(const ros::NodeHandle &nh, const std::string &name,
                const T &defaultValue) {
  if (nh.hasParam(name)) {
    T v;
    nh.param<T>(name, v, defaultValue);
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  std::cout << "Cannot find value for parameter: " << name
            << ", assigning default: " << defaultValue << std::endl;

  return defaultValue;
}
}  // namespace pc_img_conv

#endif  // PARAMS_HELPER_H_