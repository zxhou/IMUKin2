

/**
 * Copyright 2018 Nanjing University of Science and Technology
 * Author: Zhixing Hou <zxhou@njust.edu.cn>

 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef RECEIVER_H
#define RECEIVER_H

#include <memory>

#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "Mpu6050.h"

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;
  int count;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;

  bool save_seq;  //!< hzx

  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

  //! hzx
  std::string baseName;
  std::string timeStampColor;
  std::string timeStampDepth;
  std::ofstream fs;
  std::mutex lockTimestamp;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed, const std::string writePath);
  ~Receiver();
  void run(const Mode mode, MPU* mpu6050);
private:
  void start(const Mode mode, MPU* mpu6050);
  void stop();
  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);
  void imageViewer(MPU* mpu6050);
  void cloudViewer(MPU* mpu6050);
  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *);
  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;
  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;
  void readTimestamp(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, uint32_t& sec, uint32_t& nsec) const;
  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out);
  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored);
  void createLookup(size_t width, size_t height);


};

#endif