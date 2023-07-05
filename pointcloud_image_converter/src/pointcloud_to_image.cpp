//
// Created by kevinlad on 23-1-2.
//

#include "pointcloud_image_converter/pointcloud_to_image.h"

#include "pointcloud_image_converter/TicToc.hpp"
#include "pointcloud_image_converter/ros_params_helper.h"

// #define DEBUG
// #define DEBUG_ALIGNMENT

namespace pc_img_conv {
PointCloudToImage::PointCloudToImage(ros::NodeHandle &nh) : nh_(nh) {
  // Parameters:
  // assume that all points are higher than -10.0m
  // points at height within [-10m, 55m] can be stored
  PC2IMG_SCALE_FACTOR = get_ros_param(nh, "pc2img_scale_factor", 1000.0f);
  PC2IMG_SCALE_OFFSET = get_ros_param(nh, "pc2img_scale_offset", 10.0f);

  SCAN_MIN_RANGE = get_ros_param(nh, "scan_min_range", 1.0f);
  SCAN_MAX_RANGE = get_ros_param(nh, "scan_max_range", 200.0f);

  DATASET_TYPE =
      get_ros_param(nh, "dataset_type", std::string("FusionPortable"));

  MIN_T_DATA = get_ros_param(nh, "min_t_data", 0.1);
  CLOUD_BUFF = get_ros_param(nh, "cloud_buff", 50);
  N_CAM = get_ros_param(nh, "n_cam", 1);
  K_.resize(N_CAM);
  v_image_queue_.resize(N_CAM);

  // Parameters: lidar intrinsics
  lidar_intrinsics_.distortion_model_ =
      get_ros_param(nh, "distortion_model", std::string("lidar_ouster"));

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

  // ROS subscriber and publisher
  pointcloud_sub_ = nh_.subscribe("input_cloud", 1,
                                  &PointCloudToImage::PointCloudCallback, this);
  depth_pub_ = nh.advertise<sensor_msgs::Image>("depth_image", 1);
  height_pub_ = nh.advertise<sensor_msgs::Image>("height_image", 1);
  semantic_pub_ = nh.advertise<sensor_msgs::Image>("semantic_image", 1);
  lidar_info_pub_ =
      nh.advertise<sensor_msgs::CameraInfo>("lidar_camera_info", 1);

#ifdef DEBUG_ALIGNMENT
  if (DATASET_TYPE.find("SemanticFusionPortable") != std::string::npos) {
    std::stringstream ss;
    for (int i = 0; i < N_CAM; i++) {
      ss << "input_semantic_image_frame_cam" << std::setw(2)
         << std::setfill('0') << i;
      if (i == 0) {
        v_image_sub_.push_back(nh_.subscribe(
            ss.str(), 1, &PointCloudToImage::ImageCallback_FrameCam00, this));
      } else {
        v_image_sub_.push_back(nh_.subscribe(
            ss.str(), 1, &PointCloudToImage::ImageCallback_FrameCam01, this));
      }
      ss.str("");
      ss << "input_camera_info_frame_cam" << std::setw(2) << std::setfill('0')
         << i;
      v_camera_info_sub_.push_back(nh_.subscribe(
          ss.str(), 1, &PointCloudToImage::CameraInfoCallback, this));
      K_[i].setIdentity();
    }
  }
#endif
}

void PointCloudToImage::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  TicToc tt;

  pcl::PointCloud<PointType>::Ptr cloud_ptr;
  cloud_ptr.reset(new pcl::PointCloud<PointType>());
  if ((DATASET_TYPE.find("FusionPortable") != std::string::npos) ||
      (DATASET_TYPE.find("SemanticFusionPortable") != std::string::npos)) {
    pcl::fromROSMsg(*msg, *cloud_ptr);
  } else if ((DATASET_TYPE.find("SemanticKITTI") != std::string::npos) ||
             (DATASET_TYPE.find("SemanticUSL") != std::string::npos)) {
    pcl::PointCloud<pcl::PointXYZIRGBL> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    for (size_t i = 0; i < tmp_cloud.size(); ++i) {
      // NOTE(gogojjh): only for SemanticKITTI and SemanticUSL dataset
      // Filter those outlier and dynamic objects
      // label <= 1 means unlabeled or outlier
      // label > 250 means moving (dynamic) objects
      if (tmp_cloud.points[i].label <= 1) continue;
      if (tmp_cloud.points[i].label > 250) continue;

      // NOTE(gogojjh): only for SemanticUSL dataset
      if (DATASET_TYPE.find("SemanticUSL") != std::string::npos) {
        if (tmp_cloud.points[i].label == 10) continue;  // person
        if (tmp_cloud.points[i].label == 30) continue;  // car
      }
      PointType pt;
      pt.x = tmp_cloud.points[i].x;
      pt.y = tmp_cloud.points[i].y;
      pt.z = tmp_cloud.points[i].z;
      pt.reflectivity = tmp_cloud.points[i].label;
      cloud_ptr->points.push_back(pt);
    }
  } else if (DATASET_TYPE.find("KITTI360") != std::string::npos) {
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    for (size_t i = 0; i < tmp_cloud.size(); ++i) {
      PointType pt;
      pt.x = tmp_cloud.points[i].x;
      pt.y = tmp_cloud.points[i].y;
      pt.z = tmp_cloud.points[i].z;
      cloud_ptr->points.push_back(pt);
    }
  } else if (DATASET_TYPE.find("KITTI") != std::string::npos) {
    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
    pcl::fromROSMsg(*msg, tmp_cloud);
    for (size_t i = 0; i < tmp_cloud.size(); ++i) {
      PointType pt;
      pt.x = tmp_cloud.points[i].x;
      pt.y = tmp_cloud.points[i].y;
      pt.z = tmp_cloud.points[i].z;
      // pt.intensity = tmp_cloud.points[i].intensity;
      cloud_ptr->points.push_back(pt);
    }
  }
#ifdef DEBUG
  std::cout << "Pointcloud received: got " << cloud_ptr->size() << " points"
            << std::endl;
#endif

  // NOTE(gogojjh): functions to convert pointcloud into image
  pcl::PointCloud<PointType>::Ptr cloud_filter_ptr;
  cloud_filter_ptr.reset(new pcl::PointCloud<PointType>());
  if (!computeLidarIntrinsics(cloud_ptr, cloud_filter_ptr, lidar_intrinsics_))
    return;
#ifdef DEBUG
  std::cout << "Raw cloud size: " << cloud_ptr->size() << std::endl;
  std::cout << "Filter cloud size: " << cloud_filter_ptr->size() << std::endl;
#endif

  cv::Mat depth_img(lidar_intrinsics_.num_elevation_divisions_,
                    lidar_intrinsics_.num_azimuth_divisions_, CV_16UC1,
                    cv::Scalar(0));
  cv::Mat height_img(lidar_intrinsics_.num_elevation_divisions_,
                     lidar_intrinsics_.num_azimuth_divisions_, CV_16UC1,
                     cv::Scalar(0));
  cv::Mat semantic_img;
  if ((DATASET_TYPE.find("SemanticKITTI") != std::string::npos) ||
      (DATASET_TYPE.find("SemanticUSL") != std::string::npos)) {
    semantic_img = cv::Mat(lidar_intrinsics_.num_elevation_divisions_,
                           lidar_intrinsics_.num_azimuth_divisions_, CV_16UC1,
                           cv::Scalar(0));
  }

  float r, elevation_angle_rad, azimuth_angle_rad;
  int row_id, col_id;
  for (const auto &pt : *cloud_filter_ptr) {
    r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    elevation_angle_rad = acos(pt.z / r);
    azimuth_angle_rad = M_PI - atan2(pt.y, pt.x);
    row_id =
        round((elevation_angle_rad - lidar_intrinsics_.start_elevation_rad_) /
              lidar_intrinsics_.rads_per_pixel_elevation_);
    if (row_id < 0 || row_id > lidar_intrinsics_.num_elevation_divisions_ - 1)
      continue;
    col_id =
        round(azimuth_angle_rad / lidar_intrinsics_.rads_per_pixel_azimuth_);
    if (col_id >= lidar_intrinsics_.num_azimuth_divisions_)
      col_id -= lidar_intrinsics_.num_azimuth_divisions_;

    ////////// DepthImage
    // The largest depth <= 2^16/1e3 = 65.536
    float dep = r * PC2IMG_SCALE_FACTOR;
    if (dep > std::numeric_limits<uint16_t>::max()) continue;
    if (dep < 0.0f) continue;
    depth_img.at<uint16_t>(row_id, col_id) = uint16_t(dep);
    ////////// HeightImage
    float z = (pt.z + PC2IMG_SCALE_OFFSET) * PC2IMG_SCALE_FACTOR;
    if (z > std::numeric_limits<uint16_t>::max()) continue;
    if (z < 0.0f) continue;
    height_img.at<uint16_t>(row_id, col_id) = uint16_t(z);
    ////////// SemanticImage
    if ((DATASET_TYPE.find("SemanticKITTI") != std::string::npos) ||
        (DATASET_TYPE.find("SemanticUSL") != std::string::npos)) {
      semantic_img.at<uint16_t>(row_id, col_id) = pt.reflectivity;  // label
    }
  }
#ifdef DEBUG
  cv::imwrite("/Spy/dataset/tmp/depth_image.png", depth_img);
  cv::imwrite("/Spy/dataset/tmp/height_image.png", height_img);
  cv::imwrite("/Spy/dataset/tmp/semantic_image.png", semantic_img);
#endif

  // ******************* Publish ROS message
  sensor_msgs::CameraInfoPtr lidar_info_msg_ptr(new sensor_msgs::CameraInfo());
  lidar_info_msg_ptr->header = msg->header;
  lidar_intrinsics_.ConvertToMsg(lidar_info_msg_ptr);

  sensor_msgs::ImagePtr depth_img_msg_ptr =
      cv_bridge::CvImage(msg->header, "mono16", depth_img).toImageMsg();
  depth_pub_.publish(*depth_img_msg_ptr);

  sensor_msgs::ImagePtr height_img_msg_ptr =
      cv_bridge::CvImage(msg->header, "mono16", height_img).toImageMsg();
  height_pub_.publish(*height_img_msg_ptr);

  lidar_info_pub_.publish(*lidar_info_msg_ptr);

  if ((DATASET_TYPE.find("SemanticKITTI") != std::string::npos) ||
      (DATASET_TYPE.find("SemanticUSL") != std::string::npos)) {
    sensor_msgs::ImagePtr semantic_img_msg_ptr =
        cv_bridge::CvImage(msg->header, "mono16", semantic_img).toImageMsg();
    semantic_pub_.publish(semantic_img_msg_ptr);
  }

#ifdef DEBUG
  // output time: 3-10ms
  std::cout << "PointCloud to Image costs: " << tt.toc() << "ms" << std::endl;
#endif

  // NOTE(gogojjh): Only work for SemanticFusionPortable data
  if (DATASET_TYPE.find("SemanticFusionPortable") != std::string::npos) {
    mutex_cloud_.lock();
    cloud_queue_.emplace(msg->header, cloud_ptr, lidar_intrinsics_);
    while (!cloud_queue_.empty() && cloud_queue_.size() > CLOUD_BUFF) {
      cloud_queue_.pop();
    }
    mutex_cloud_.unlock();
  }
}  // namespace pc_img_conv

// clang-format off
// NOTE(gogojjh): Only work for SemanticFusionPortable data
void PointCloudToImage::ImageCallback_FrameCam00(const sensor_msgs::ImageConstPtr &msg) {
  if (K_.size() < 1) return;
  if ((K_[0](0, 0) == 1.0f || K_[0](1, 1) == 1.0f)) return;
  TicToc tt;
  ProcessPointCloudImageAlignment(msg, K_[0]);
#ifdef DEBUG_ALIGNMENT
  std::cout << "PointCloud to Image Alignment costs: " << tt.toc() << "ms" << std::endl; // output time: 3-10ms
#endif
}

void PointCloudToImage::ImageCallback_FrameCam01(const sensor_msgs::ImageConstPtr &msg) {
  if (K_.size() < 2) return;
  if ((K_[1](0, 0) == 1.0f || K_[1](1, 1) == 1.0f)) return;
  TicToc tt;
  ProcessPointCloudImageAlignment(msg, K_[1]);
#ifdef DEBUG_ALIGNMENT
  std::cout << "PointCloud to Image Alignment costs: " << tt.toc() << "ms" << std::endl; // output time: 3-10ms
#endif
}
// clang-format on

// clang-format off
// NOTE(gogojjh): Only work for SemanticFusionPortable data
void PointCloudToImage::CameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr &msg) {
  if ((msg->header.frame_id.compare("frame_cam00") == 0) && (K_.size() >= 1)) {
    if (K_[0](0, 0) == 1.0f || K_[0](1, 1) == 1.0f) {
      K_[0] << msg->K[0], msg->K[1], msg->K[2], 
               msg->K[3], msg->K[4], msg->K[5], 
               msg->K[6], msg->K[7], msg->K[8];
    }
  } else if ((msg->header.frame_id.compare("frame_cam01") == 0) && (K_.size() >= 2)) {
    if (K_[1](0, 0) == 1.0f || K_[1](1, 1) == 1.0f) {
      K_[1] << msg->K[0], msg->K[1], msg->K[2], 
               msg->K[3], msg->K[4], msg->K[5], 
               msg->K[6], msg->K[7], msg->K[8];
    }
  } else {
    return;
  }
}
// clang-format on

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
bool PointCloudToImage::computeLidarIntrinsics(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    pcl::PointCloud<PointType>::Ptr &cloud_filter, LidarIntrinsics &intr) {
  cloud_filter->reserve(cloud_ptr->size());

  // NOTE(gogojjh): depend on the specific dataset
  if (intr.distortion_model_.find("lidar_ouster") != std::string::npos) {
    intr.start_elevation_rad_ = 0.0f;
    intr.end_elevation_rad_ = 0.0f;
    int min_ring = 1000, max_ring = -1000;
    for (const auto &pt : *cloud_ptr) {
      max_ring = std::max(max_ring, int(pt.ring));
      min_ring = std::min(min_ring, int(pt.ring));
    }
    int num_rings = max_ring - min_ring + 1;

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
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
            !std::isfinite(pt.z))
          continue;
        // NOTE(gogojjh): Filter out points that are too close
        // Otherwise it will produce some NaN values
        float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
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
  } else if (intr.distortion_model_.find("lidar_vlp") != std::string::npos) {
    intr.start_elevation_rad_ = 2 * M_PI;
    intr.end_elevation_rad_ = 0.0f;
    int cnt_start = 0, cnt_end = 0;
    for (const auto &pt : *cloud_ptr) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;
      float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r < SCAN_MIN_RANGE || r > SCAN_MAX_RANGE) continue;
      cloud_filter->push_back(pt);

      float elevation_angle_rad = acos(pt.z / r);
      intr.start_elevation_rad_ =
          intr.start_elevation_rad_ < elevation_angle_rad
              ? intr.start_elevation_rad_
              : elevation_angle_rad;
      intr.end_elevation_rad_ = intr.end_elevation_rad_ > elevation_angle_rad
                                    ? intr.end_elevation_rad_
                                    : elevation_angle_rad;
    }
    intr.rads_per_pixel_elevation_ =
        (intr.end_elevation_rad_ - intr.start_elevation_rad_) /
        (intr.num_elevation_divisions_ - 1);
    intr.rads_per_pixel_azimuth_ =
        intr.horizontal_fov_ / (intr.num_azimuth_divisions_ - 1);
    intr.vertical_fov_ = intr.end_elevation_rad_ - intr.start_elevation_rad_;
  }
#ifdef DEBUG
  intr.PrintIntrinsics();
#endif
  return true;
}

void PointCloudToImage::Pointcloud2DepthImage(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &img) {
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
    // the largest depth <= 2^16/1e3 = 65.536
    if (dep > std::numeric_limits<uint16_t>::max()) {
#ifdef DEBUG
      std::cout << "depth value is too large: " << dep << std::endl;
#endif
      continue;
    }
    if (dep < 0.0f) continue;
    img.at<uint16_t>(row_id, col_id) = uint16_t(dep);
  }
}

void PointCloudToImage::Pointcloud2HeightImage(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &img) {
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
    img.at<uint16_t>(row_id, col_id) = uint16_t(z);
  }
}

void PointCloudToImage::Pointcloud2SemanticImage(
    const pcl::PointCloud<PointType>::Ptr &cloud_ptr,
    const LidarIntrinsics &intr, cv::Mat &img) {
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
    img.at<uint16_t>(row_id, col_id) = pt.reflectivity;  // label
  }
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
// NOTE(gogojjh): Only work for SemanticFusionPortable data
//                Not used, under development
void PointCloudToImage::ProcessPointCloudImageAlignment(
    const sensor_msgs::ImageConstPtr &msg, const Eigen::Matrix3d &K) {
  mutex_cloud_.lock();
  // TODO(gogojjh): improve the code logistics here
  // Remove elements from the queue if their timestamps are too old < 0.1s
  while (!cloud_queue_.empty() &&
         std::get<0>(cloud_queue_.front()).stamp.toSec() <
             msg->header.stamp.toSec() - MIN_T_DATA) {
#ifdef DEBUG_ALIGNMENT
    std::cout << "Remove cloud from the queue: "
              << std::get<0>(cloud_queue_.front()).stamp << " "
              << abs(std::get<0>(cloud_queue_.front()).stamp.toSec() -
                     msg->header.stamp.toSec())
              << std::endl;
#endif
    cloud_queue_.pop();
  }

  // NOTE(gogojjh): no nearest lidar data
  if (cloud_queue_.empty()) {
    mutex_cloud_.unlock();
    return;
  }

  double t_msg = msg->header.stamp.toSec();

  bool select_t1 = false;
  bool select_t2 = false;
  const auto data_tuple_1 = cloud_queue_.front();
  double t1 = std::get<0>(data_tuple_1).stamp.toSec();
  cloud_queue_.pop();

  const auto data_tuple_2 = cloud_queue_.front();
  double t2 = std::get<0>(data_tuple_2).stamp.toSec();

  // Handle the case1:
  // Image:       |
  // Pointcloud:    |
  if (cloud_queue_.size() == 1) {
    if (abs(t1 - t_msg) < MIN_T_DATA) {
      select_t1 = true;
    }
  }
  // Handle the case:
  // Image:        |
  // Pointcloud: |    |
  else {
    if ((abs(t1 - t_msg) < MIN_T_DATA) && (abs(t1 - t_msg) < abs(t2 - t_msg))) {
      select_t1 = true;
    } else if ((abs(t2 - t_msg) < MIN_T_DATA) &&
               (abs(t2 - t_msg) < abs(t1 - t_msg))) {
      select_t2 = true;
    }
  }

  if (select_t1 || select_t2) {
    // Retrieve sensor data
    std_msgs::Header header;
    pcl::PointCloud<PointType>::Ptr cloud_ptr;
    LidarIntrinsics lidar_intrinsics;

    if (select_t1) {
      std::tie(header, cloud_ptr, lidar_intrinsics) = data_tuple_1;
    } else {
      std::tie(header, cloud_ptr, lidar_intrinsics) = data_tuple_2;
      cloud_queue_.pop();
    }
    mutex_cloud_.unlock();
#ifdef DEBUG_ALIGNMENT
    std::cout << "Accept cloud from the queue: " << header.stamp << " "
              << abs(header.stamp.toSec() - t_msg) << std::endl;
#endif

#ifdef DEBUG_ALIGNMENT
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image = cv_ptr->image;
#else
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "mono16");
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image = cv_ptr->image;
#endif

    // RetrieveTF
    Eigen::Matrix4d T_cam_lidar;
    tf::StampedTransform TF_cam_lidar;
    std::string world_frame = msg->header.frame_id;
    std::string source_frame = header.frame_id;
    std::string *err_msg = new std::string();
    if (!listener_.canTransform(world_frame, source_frame, msg->header.stamp,
                                err_msg)) {
#ifdef DEBUG_ALIGNMENT
      std::cout << "No transform available: " << *err_msg << std::endl;
#endif
      delete err_msg;
    } else {
      listener_.lookupTransform(world_frame, source_frame, msg->header.stamp,
                                TF_cam_lidar);
      Eigen::Affine3d pose_cam_lidar;
      tf::transformTFToEigen(TF_cam_lidar, pose_cam_lidar);
      T_cam_lidar = pose_cam_lidar.matrix();
#ifdef DEBUG_ALIGNMENT
      std::cout << "T_cam_lidar: " << std::endl << T_cam_lidar << std::endl;
#endif
    }

    // Pointcloud image alignment
#ifdef DEBUG_ALIGNMENT
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_label(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_label->reserve(cloud_ptr->size() / 2);
    cv::Vec3b color;
    pcl::PointXYZRGB p;
    for (size_t i = 0; i < cloud_ptr->points.size(); i++) {
      p.x = cloud_ptr->points[i].x;
      p.y = cloud_ptr->points[i].y;
      p.z = cloud_ptr->points[i].z;

      Eigen::Vector3d pt(p.x, p.y, p.z);
      Eigen::Vector3d pt_cam =
          T_cam_lidar.block<3, 3>(0, 0) * pt + T_cam_lidar.block<3, 1>(0, 3);
      Eigen::Vector3d pt_pixel = K * pt_cam;
      Eigen::Vector2d uv(pt_pixel[0] / pt_pixel[2], pt_pixel[1] / pt_pixel[2]);
      if ((pt_cam.z() <= 0.0) || (uv[0] < 0 || uv[0] >= image.cols ||
                                  uv[1] < 0 || uv[1] >= image.rows)) {
        continue;
      }
      color = image.at<cv::Vec3b>(uv[1], uv[0]);
      p.r = color[2];
      p.g = color[1];
      p.b = color[0];
      cloud_label->push_back(p);
    }
#else
    pcl::PointCloud<PointType>::Ptr cloud_label(new pcl::PointCloud<PointType>);
    cloud_label->reserve(cloud_ptr->size() / 2);
    PointType p;
    for (size_t i = 0; i < cloud_ptr->points.size(); i++) {
      p.x = cloud_ptr->points[i].x;
      p.y = cloud_ptr->points[i].y;
      p.z = cloud_ptr->points[i].z;
      Eigen::Vector3d pt(p.x, p.y, p.z);
      Eigen::Vector3d pt_cam =
          T_cam_lidar.block<3, 3>(0, 0) * pt + T_cam_lidar.block<3, 1>(0, 3);
      Eigen::Vector3d pt_pixel = K * pt_cam;
      Eigen::Vector2d uv(pt_pixel[0] / pt_pixel[2], pt_pixel[1] / pt_pixel[2]);
      // Remove points that are not observed by the camera
      if ((pt_cam.z() <= 0.0) || (uv[0] < 0 || uv[0] >= image.cols ||
                                  uv[1] < 0 || uv[1] >= image.rows)) {
        continue;
      }
      p.reflectivity = image.at<std::uint16_t>(uv[1], uv[0]);  // get the
      cloud_label->push_back(p);
    }
#endif

    cv::Mat semantic_img = cv::Mat(lidar_intrinsics.num_elevation_divisions_,
                                   lidar_intrinsics.num_azimuth_divisions_,
                                   CV_16UC1, cv::Scalar(0));
    Pointcloud2SemanticImage(cloud_label, lidar_intrinsics, semantic_img);

    sensor_msgs::CameraInfoPtr lidar_info_msg_ptr(
        new sensor_msgs::CameraInfo());
    lidar_info_msg_ptr->header = header;
    lidar_intrinsics.ConvertToMsg(lidar_info_msg_ptr);

    sensor_msgs::ImagePtr semantic_img_msg_ptr =
        cv_bridge::CvImage(header, "mono16", semantic_img).toImageMsg();
    semantic_pub_.publish(semantic_img_msg_ptr);
  } else {
    mutex_cloud_.unlock();
  }
}  // namespace pc_img_conv
}  // namespace pc_img_conv