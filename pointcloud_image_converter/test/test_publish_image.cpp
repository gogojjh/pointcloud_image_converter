#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <opencv2/opencv.hpp>

void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat depth_img =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
  cv::imwrite("/Spy/dataset/tmp/depth_image_pub.png", depth_img);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_publish_image");

  ros::NodeHandle nh;

  ros::Subscriber depth_image_sub =
      nh.subscribe("/pc2img/depth_image", 10, DepthImageCallback);

  ros::spin();

  return 0;
}