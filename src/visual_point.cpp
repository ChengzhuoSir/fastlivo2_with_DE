/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "visual_point.h"
#include "feature.h"
#include <stdexcept>
#include <vikit/math_utils.h>

// 【核心功能】创建视觉点并初始化位置与参考信息
// 【输入/输出】输入: 世界坐标位置; 输出: 初始化状态与观测列表
VisualPoint::VisualPoint(const Vector3d &pos)
    : pos_(pos), previous_normal_(Vector3d::Zero()), normal_(Vector3d::Zero()),
      is_converged_(false), is_normal_initialized_(false), has_ref_patch_(false)
{
}

// 【核心功能】析构视觉点并清理关联特征
// 【输入/输出】输入: 观测特征列表; 输出: 释放资源
VisualPoint::~VisualPoint() 
{
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    delete(*it);
  }
  obs_.clear();
  ref_patch = nullptr;
}

// 【核心功能】添加帧内观测引用
// 【输入/输出】输入: 特征指针; 输出: 追加观测并更新统计
void VisualPoint::addFrameRef(Feature *ftr)
{
  obs_.push_front(ftr);
}

// 【核心功能】移除指定特征引用
// 【输入/输出】输入: 特征指针; 输出: 更新观测列表
void VisualPoint::deleteFeatureRef(Feature *ftr)
{
  if (ref_patch == ftr)
  {
    ref_patch = nullptr;
    has_ref_patch_ = false;
  }
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    if ((*it) == ftr)
    {
      delete((*it));
      obs_.erase(it);
      return;
    }
  }
}

// 【核心功能】寻找视角接近的观测特征
// 【输入/输出】输入: 当前位姿与像素; 输出: 返回是否命中并给出特征
bool VisualPoint::getCloseViewObs(const Vector3d &framepos, Feature *&ftr, const Vector2d &cur_px) const
{
  // TODO: get frame with same point of view AND same pyramid level!
  if (obs_.size() <= 0) return false;

  Vector3d obs_dir(framepos - pos_);
  obs_dir.normalize();
  auto min_it = obs_.begin();
  double min_cos_angle = 0;
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    Vector3d dir((*it)->T_f_w_.inverse().translation() - pos_);
    dir.normalize();
    double cos_angle = obs_dir.dot(dir);
    if (cos_angle > min_cos_angle)
    {
      min_cos_angle = cos_angle;
      min_it = it;
    }
  }
  ftr = *min_it;

  // Vector2d ftr_px = ftr->px_;
  // double pixel_dist = (cur_px-ftr_px).norm();

  // if(pixel_dist > 200)
  // {
  //   ROS_ERROR("The pixel dist exceeds 200.");
  //   return false;
  // }

  if (min_cos_angle < 0.5) // assume that observations larger than 60° are useless 0.5
  {
    // ROS_ERROR("The obseved angle is larger than 60°.");
    return false;
  }

  return true;
}

// 【核心功能】根据评分选取最优特征观测
// 【输入/输出】输入: 当前位姿; 输出: 返回最佳特征指针
void VisualPoint::findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const
{
  auto min_it = obs_.begin();
  float min_score = std::numeric_limits<float>::max();

  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    if ((*it)->score_ < min_score)
    {
      min_score = (*it)->score_;
      min_it = it;
    }
  }
  ftr = *min_it;
}

// 【核心功能】清理非参考 patch 的观测特征
// 【输入/输出】输入: 观测列表; 输出: 删除无效特征
void VisualPoint::deleteNonRefPatchFeatures()
{
  for (auto it = obs_.begin(); it != obs_.end();)
  {
    if (*it != ref_patch)
    {
      delete *it;
      it = obs_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}
