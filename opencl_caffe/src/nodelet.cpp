/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <pluginlib/class_list_macros.h>
#include "opencl_caffe/detector_gpu.h"
#include "opencl_caffe/nodelet.h"

PLUGINLIB_EXPORT_CLASS(opencl_caffe::Nodelet, nodelet::Nodelet)

namespace opencl_caffe
{
void Nodelet::onInit()
{
  ros::NodeHandle pnh = getPrivateNodeHandle();
  std::string net_config_path, weights_path, labels_path;
  // 载入配置文件　
  if (!pnh.getParam("net_config_path", net_config_path))// 模型文件
  {
    ROS_WARN("param net_cfg_path not set, use default");
  }
  if (!pnh.getParam("weights_path", weights_path))// 权重
  {
    ROS_WARN("param weights_path not set, use default");
  }
  if (!pnh.getParam("labels_path", labels_path))// 标签
  {
    ROS_WARN("param labels_path not set, use default");
  }

  loadResources(net_config_path, weights_path, labels_path);// 检测 
  pub_ = getNodeHandle().advertise<object_msgs::ObjectsInBoxes>("inference", 1);
}
// 话题回调函数
void Nodelet::cbImage(const sensor_msgs::ImagePtr image_msg)
{
  object_msgs::ObjectsInBoxes objects;
  if (detector_->runInference(image_msg, objects))// 前向推理
  {
    pub_.publish(objects);//　发布检测结果
  }
  else
  {
    ROS_ERROR("Inference failed.");
  }
}

void Nodelet::loadResources(const std::string net_config_path, const std::string weights_path,
                            const std::string labels_path)
{
  detector_.reset(new DetectorGpu());
  sub_.shutdown();

  if (detector_->loadResources(net_config_path, weights_path, labels_path))
  {
    sub_ = getNodeHandle().subscribe("/usb_cam/image_raw", 1, &Nodelet::cbImage, this);
  }
  else
  {
    ROS_FATAL("Load resource failed.");
    ros::shutdown();
  }
}
}  // namespace opencl_caffe
