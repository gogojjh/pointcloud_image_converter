//
// Created by kevinlad on 23-1-2.
//

#include "pointcloud_image_converter/pointcloud_to_image.h"

#include "pointcloud_image_converter/TicToc.hpp"
#include "pointcloud_image_converter/ros_params_helper.h"

// #define DEBUG

namespace pc_img_conv {
PointCloudToImage::PointCloudToImage(ros::NodeHandle &nh) : nh_(nh) {
  pointcloud_sub_ = nh_.subscribe("input_cloud", 1,
                                  &PointCloudToImage::PointCloudCallback, this);
  it_ptr_.reset(new image_transport::ImageTransport(nh_));
  depth_pub_ = it_ptr_->advertiseCamera("depth_image", 1);
  height_pub_ = it_ptr_->advertiseCamera("height_image", 1);

  // Parameters:
  // assume that all points are higher than -10.0m
  // points at height within [-10m, 55m] can be stored
  PC2IMG_SCALE_FACTOR = get_ros_param(nh, "pc2img_scale_factor", 1000.0f);
  PC2IMG_SCALE_OFFSET = get_ros_param(nh, "pc2img_scale_offset", 10.0f);

  SCAN_MIN_RANGE = get_ros_param(nh, "scan_min_range", 1.0f);
  SCAN_MAX_RANGE = get_ros_param(nh, "scan_max_range", 200.0f);

  DATASET_TYPE =
      get_ros_param(nh, "dataset_type", std::string("FusionPortable"));

  // Parameters: lidar intrinsics
  lidar_intrinsics_.num_elevation_divisions_ =
      get_ros_param(nh, "num_elevation_divisions", 128);
  lidar_intrinsics_.num_azimuth_divisions_ =
      get_ros_param(nh, "num_azimuth_divisions", 2048);

  lidar_intrinsics_.start_elevation_rad_ =
      get_ros_param(nh, "start_elevation_rad", 65.0f);
  lidar_intrinsics_.end_elevation_rad_ =
      get_ros_param(nh, "end_elevation_rad", 115.0f);
  lidar_intrinsics_.vertical_fov_ = lidar_intrinsics_.end_elevation_rad_ -
                                    lidar_intrinsics_.start_elevation_rad_;

  lidar_intrinsics_.start_azimuth_rad_ =
      get_ros_param(nh, "start_azimuth_rad", 0.0f);
  lidar_intrinsics_.end_azimuth_rad_ =
      get_ros_param(nh, "end_azimuth_rad", 2 * M_PI);
  lidar_intrinsics_.horizontal_fov_ =
      lidar_intrinsics_.end_azimuth_rad_ - lidar_intrinsics_.start_azimuth_rad_;
}

bool PointCloudToImage::computeLidarIntrinsics(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    pcl::PointCloud<PointType>::Ptr &cloud_filter, LidarIntrinsics &intr) {
  cloud_filter->reserve(cloud_ptr->size());

  // NOTE(gogojjh): depend on the specific dataset
  if (!DATASET_TYPE.find("FusionPortable") != std::string::npos) {
    intr.start_elevation_rad_ = 0.0f;
    intr.end_elevation_rad_ = 0.0f;
    int min_ring = 1000, max_ring = -1000;
    for (const auto &pt : *cloud_ptr) {
      max_ring = std::max(max_ring, int(pt.ring));
      min_ring = std::min(min_ring, int(pt.ring));
    }
    int num_rings = max_ring - min_ring + 1;
#ifdef DEBUG
    std::cout << "min_ring: " << min_ring << " max_ring: " << max_ring
              << std::endl;
#endif

    // NOTE(gogojjh): if all points are removed, output the default values
    if (num_rings > intr.num_elevation_divisions_) {
      intr.rads_per_pixel_elevation_ =
          (intr.end_elevation_rad_ - intr.start_elevation_rad_) /
          (intr.num_elevation_divisions_ - 1);
      intr.rads_per_pixel_azimuth_ =
          intr.horizontal_fov_ / (intr.num_azimuth_divisions_ - 1);
    } else {
      int cnt_start = 0;
      int cnt_end = 0;
      for (const auto &pt : *cloud_ptr) {
        float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        // NOTE(gogojjh): Filter out points that are too close
        // Otherwise it will produce some NaN values
        if (r < SCAN_MIN_RANGE || r > SCAN_MAX_RANGE) continue;
        cloud_filter->push_back(pt);

        float elevation_angle_rad = acos(pt.z / r);
        if (int(pt.ring) == min_ring) {
          intr.start_elevation_rad_ += elevation_angle_rad;
          cnt_start++;
        }
        if (int(pt.ring) == max_ring) {
          intr.end_elevation_rad_ += elevation_angle_rad;
          cnt_end++;
        }
      }
      if (cnt_start == 0 || cnt_end == 0) return false;
      intr.start_elevation_rad_ /= cnt_start;
      intr.end_elevation_rad_ /= cnt_end;
#ifdef DEBUG
      std::cout << "start_elevation_rad: " << intr.start_elevation_rad_
                << std::endl;
      std::cout << "end_elevation_rad: " << intr.end_elevation_rad_
                << std::endl;
#endif

      intr.rads_per_pixel_elevation_ =
          (intr.end_elevation_rad_ - intr.start_elevation_rad_) /
          (num_rings - 1);
      intr.rads_per_pixel_azimuth_ =
          intr.horizontal_fov_ / (intr.num_azimuth_divisions_ - 1);
      intr.start_elevation_rad_ -= intr.rads_per_pixel_elevation_ * min_ring;
      intr.end_elevation_rad_ += intr.rads_per_pixel_elevation_ *
                                 (intr.num_elevation_divisions_ - 1 - max_ring);
      intr.vertical_fov_ = intr.end_elevation_rad_ - intr.start_elevation_rad_;
    }
  } else if (!DATASET_TYPE.find("KITTI") != std::string::npos) {
  } else if (!DATASET_TYPE.find("SemanticKITTI") != std::string::npos) {
  } else if (!DATASET_TYPE.find("SemanticUSL") != std::string::npos) {
  }
  intr.PrintIntrinsics();
  return true;
}

void PointCloudToImage::Pointcloud2DepthImage(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &depth_img) {
  // NOTE(gogojjh): convert range to into millimeter.
  for (const auto &pt : *cloud_ptr) {
    float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    float elevation_angle_rad = acos(pt.z / r);
    float azimuth_angle_rad = M_PI - atan2(pt.y, pt.x);

    int row_id = std::round((elevation_angle_rad - intr.start_elevation_rad_) /
                            intr.rads_per_pixel_elevation_);
    if (row_id < 0 || row_id > intr.num_elevation_divisions_ - 1) continue;

    int col_id = std::round(azimuth_angle_rad / intr.rads_per_pixel_azimuth_);
    if (col_id >= intr.num_azimuth_divisions_)
      col_id -= intr.num_azimuth_divisions_;

    float dep = r * PC2IMG_SCALE_FACTOR;
    if (dep > std::numeric_limits<uint16_t>::max()) {
      std::cout << "depth value is too large: " << dep << std::endl;
      continue;
    }
    if (dep < 0.0f) continue;
    depth_img.at<uint16_t>(row_id, col_id) = uint16_t(dep);
  }
}

void PointCloudToImage::Pointcloud2HeightImage(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &height_img) {
  // NOTE(gogojjh): convert range to into millimeter.
  for (const auto &pt : *cloud_ptr) {
    float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    float elevation_angle_rad = acos(pt.z / r);
    float azimuth_angle_rad = M_PI - atan2(pt.y, pt.x);

    int row_id = round((elevation_angle_rad - intr.start_elevation_rad_) /
                       intr.rads_per_pixel_elevation_);
    if (row_id < 0 || row_id > intr.num_elevation_divisions_ - 1) continue;

    int col_id = round(azimuth_angle_rad / intr.rads_per_pixel_azimuth_);
    if (col_id >= intr.num_azimuth_divisions_)
      col_id -= intr.num_azimuth_divisions_;

    float z = (pt.z + PC2IMG_SCALE_OFFSET) * PC2IMG_SCALE_FACTOR;
    if (z > std::numeric_limits<uint16_t>::max()) continue;
    if (z < 0.0f) continue;
    height_img.at<uint16_t>(row_id, col_id) = uint16_t(z);
  }
}

void PointCloudToImage::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  TicToc tt;

  // ******************* Initialize sensor data message
  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>());
  sensor_msgs::CameraInfoPtr lidar_info_msg_ptr(new sensor_msgs::CameraInfo());
  sensor_msgs::ImagePtr height_img_msg_ptr;
  sensor_msgs::ImagePtr depth_img_msg_ptr;

  pcl::fromROSMsg(*msg, *cloud_ptr);
  auto frame_id = msg->header.frame_id;
  auto stamp = msg->header.stamp;
#ifdef DEBUG
  std::cout << "Pointcloud received: got " << cloud_ptr->size() << " points"
            << std::endl;
#endif

  // NOTE(gogojjh): functions to convert pointcloud into image
  pcl::PointCloud<PointType>::Ptr cloud_filter_ptr(
      new pcl::PointCloud<PointType>());
  if (!computeLidarIntrinsics(cloud_ptr, cloud_filter_ptr, lidar_intrinsics_))
    return;
#ifdef DEBUG
  std::cout << "Raw cloud size: " << cloud_ptr->size() << std::endl;
  std::cout << "Filter cloud size: " << cloud_filter_ptr->size() << std::endl;
#endif

  cv::Mat depth_img(lidar_intrinsics_.num_elevation_divisions_,
                    lidar_intrinsics_.num_azimuth_divisions_, CV_16UC1,
                    cv::Scalar(0));
  Pointcloud2DepthImage(cloud_filter_ptr, lidar_intrinsics_, depth_img);
#ifdef DEBUG
  // cv::imwrite("/tmp/depth_image.png", depth_img);
#endif

  cv::Mat height_img(lidar_intrinsics_.num_elevation_divisions_,
                     lidar_intrinsics_.num_azimuth_divisions_, CV_16UC1,
                     cv::Scalar(0));
  Pointcloud2HeightImage(cloud_filter_ptr, lidar_intrinsics_, height_img);
#ifdef DEBUG
  cv::imwrite("/tmp/height_image.png", height_img);
#endif

  // ******************* Publish ROS message
  lidar_info_msg_ptr->header.frame_id = frame_id;
  lidar_info_msg_ptr->header.stamp = stamp;
  lidar_info_msg_ptr->height = lidar_intrinsics_.num_elevation_divisions_;
  lidar_info_msg_ptr->width = lidar_intrinsics_.num_azimuth_divisions_;
  lidar_info_msg_ptr->distortion_model = std::string("lidar_ouster");
  // TODO: Dirty code, using D to store lidar intrinsic
  lidar_info_msg_ptr->D = {
      static_cast<float>(lidar_intrinsics_.num_azimuth_divisions_),
      static_cast<float>(lidar_intrinsics_.num_elevation_divisions_),
      lidar_intrinsics_.horizontal_fov_,
      lidar_intrinsics_.vertical_fov_,
      lidar_intrinsics_.start_azimuth_rad_,
      lidar_intrinsics_.end_azimuth_rad_,
      lidar_intrinsics_.start_elevation_rad_,
      lidar_intrinsics_.end_elevation_rad_};

  depth_img_msg_ptr =
      cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_img).toImageMsg();
  depth_img_msg_ptr->header.frame_id = frame_id;
  depth_img_msg_ptr->header.stamp = stamp;
  depth_pub_.publish(depth_img_msg_ptr, lidar_info_msg_ptr);

  height_img_msg_ptr =
      cv_bridge::CvImage(std_msgs::Header(), "mono16", height_img).toImageMsg();
  height_img_msg_ptr->header.frame_id = frame_id;
  height_img_msg_ptr->header.stamp = stamp;
  height_pub_.publish(height_img_msg_ptr, lidar_info_msg_ptr);

  // output time: 3-5ms
  std::cout << "PointCloud to Image costs: " << tt.toc() << "ms" << std::endl;
}
}  // namespace pc_img_conv