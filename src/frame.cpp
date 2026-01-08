/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include <boost/bind.hpp>
#include <iostream>
#include "feature.h"
#include "frame.h"
#include "visual_point.h"
#include <stdexcept>
#include <vikit/math_utils.h>
#include <vikit/performance_monitor.h>
#include <vikit/vision.h>

int Frame::frame_counter_ = 0;

// 【核心功能】构造帧对象并初始化图像缓存
// 【输入/输出】输入: 相机模型与灰度图像; 输出: 生成帧ID并完成合法性校验
Frame::Frame(vk::AbstractCamera *cam, const cv::Mat &img)
    : id_(frame_counter_++), 
      cam_(cam)
{
  initFrame(img);
}

// 【核心功能】析构帧对象并释放特征点资源
// 【输入/输出】输入: 挂载的特征点列表; 输出: 完成内存清理
Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature *i) { delete i; });
}

// 【核心功能】校验输入图像并保存到帧对象
// 【输入/输出】输入: 灰度图像; 输出: 成功写入成员或抛出异常
void Frame::initFrame(const cv::Mat &img)
{
  if (img.empty()) { throw std::runtime_error("Frame: provided image is empty"); }

  if (img.cols != cam_->width() || img.rows != cam_->height())
  {
    std::cerr << "Frame: image size differs from camera model, continue with scaled input." << std::endl;
  }

  if (img.type() != CV_8UC1) { throw std::runtime_error("Frame: provided image is not grayscale"); }

  img_ = img;
}

/// Utility functions for the Frame class
namespace frame_utils
{

// 【核心功能】构建金字塔图像用于多尺度处理
// 【输入/输出】输入: 原始图像与层数; 输出: 填充图像金字塔
void createImgPyramid(const cv::Mat &img_level_0, int n_levels, ImgPyr &pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;
  for (int i = 1; i < n_levels; ++i)
  {
    pyr[i] = cv::Mat(pyr[i - 1].rows / 2, pyr[i - 1].cols / 2, CV_8U);
    vk::halfSample(pyr[i - 1], pyr[i]);
  }
}

} // namespace frame_utils
