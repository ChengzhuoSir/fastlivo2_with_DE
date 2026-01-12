# fastlivo2_with_DE
# 更轻量级的 FAST-LIVO（定稿方案）

本方案将论文优化点与轻量化诉求融合为**固定、可落地的工程方案**，不使用“可选”逻辑。所有机制均为默认启用，便于统一实现与验证。

## 1) 设计目标（明确约束）
- **CPU 占用降低**：视觉更新平均频率降到原来的 1/3。
- **内存稳定**：视觉点数量上线 + 超龄淘汰，避免长时间增长。
- **鲁棒性保留**：LiDAR 退化时自动提高视觉更新频率。

## 2) 模块级修改方案（确定版）

### A. `src/voxel_map.cpp` + `include/voxel_map.h`
**目标：输出统一的退化状态与指标，供视觉更新调度使用。**

1. 新增成员字段（`VoxelMapManager`）：
   - `double sigma_min_`
   - `double valid_plane_ratio_`
   - `double avg_residual_`
   - `int degen_count_`
   - `bool lidar_degenerate_`

2. 在 `VoxelMapManager::StateEstimation` 中计算指标：
   - `valid_plane_ratio_ = effct_feat_num_ / max(1, feats_down_size_)`
   - `avg_residual_ = total_residual / max(1, effct_feat_num_)`
   - `sigma_min_` 计算：
     - 构造 `N = Σ (n_i · n_i^T)`，`n_i` 为 `ptpl_list_` 中平面法向。
     - `sigma = SVD(N)`，`sigma_min_ = sigma(2) / sigma(0)`。

3. 固定退化判据（连续帧）：
   - `degenerate_now = (sigma_min_ < 0.07) || (valid_plane_ratio_ < 0.15) || (avg_residual_ > 0.12)`
   - `degen_count_ = degenerate_now ? degen_count_ + 1 : max(0, degen_count_ - 1)`
   - `lidar_degenerate_ = (degen_count_ >= 3)`

4. 提供只读接口：
   - `bool IsLidarDegenerate() const;`
   - `double GetSigmaMin() const;`

### B. `src/LIVMapper.cpp` + `include/LIVMapper.h`
**目标：严格控制视觉更新频率，并在退化时提频。**

1. 新增参数（读取 ROS 参数，默认值如下）：
   - `visual/image_stride_normal = 3`
   - `visual/image_stride_degenerate = 1`
   - `visual/keyframe_trans_thresh = 1.0`（米）
   - `visual/keyframe_rot_thresh = 30.0`（度）
   - `visual/keyframe_scale_min = 0.3`

2. 新增状态：
   - `int img_counter_`
   - `StatesGroup last_keyframe_state_`
   - `bool has_keyframe_state_`

3. 在 `img_cbk` 中执行帧选择（直接丢弃非选中帧）：
   - `img_counter_++`
   - 计算 `scale = clamp(3 * sigma_min_, keyframe_scale_min, 1.0)`
   - `trans_thresh = scale * keyframe_trans_thresh`
   - `rot_thresh = scale * keyframe_rot_thresh`
   - `is_keyframe = (Δpos > trans_thresh) || (Δrot > rot_thresh)`
   - `stride = IsLidarDegenerate() ? image_stride_degenerate : image_stride_normal`
   - `stride_hit = (img_counter_ % stride == 0)`
   - **使用规则（固定）：** `use_image = IsLidarDegenerate() || stride_hit || is_keyframe`
   - 若 `use_image == false`，直接 `return`（不入 buffer）。
   - 若 `is_keyframe == true`，更新 `last_keyframe_state_`。

4. 地图滑窗联动：
   - `mapSliding()` 完成后调用 `vio_manager->TrimVisualMap(center, radius)`。

### C. `src/vio.cpp` + `include/vio.h`
**目标：视觉点数量可控、图像处理更轻量。**

1. 新增参数（默认值）：
   - `visual/max_points_per_voxel = 30`
   - `visual/point_max_age = 50`（帧）
   - `visual/downsample_ratio = 0.5`
   - `visual/patch_pyrimid_level = 2`
   - `visual/patch_size = 6`

2. 视觉点上限与淘汰（`insertPointIntoVoxelMap`）：
   - 若 `voxel_points.size() >= max_points_per_voxel`：
     - 淘汰 `obs_.size()` 最小的点（并释放其 `Feature` 资源）。

3. 超龄淘汰（`updateVisualMapPoints`）：
   - `current_id - pt->obs_.back()->id_ > point_max_age` → 删除点。

4. 图像降采样（`processFrame`）：
   - `cv::resize(img, img, Size(), downsample_ratio, downsample_ratio)`。
   - 同步更新相机内参与 `image_resize_factor`。

5. 新增 `TrimVisualMap(center, radius)`：
   - 将超出局部地图半径的视觉点转移到长期视觉地图。

6. 新增长期视觉地图：
   - `long_term_feat_map` 存储稀疏历史点。
   - `UpdateLongTermMapSliding` 按更大尺度滑动裁剪长期地图。

## 3) 参数落地（`config/*.yaml` 新增项）
```
visual:
  image_stride_normal: 3
  image_stride_degenerate: 1
  keyframe_trans_thresh: 1.0
  keyframe_rot_thresh: 30.0
  keyframe_scale_min: 0.3
  max_points_per_voxel: 30
  long_term_max_points_per_voxel: 10
  point_max_age: 50
  downsample_ratio: 0.5
  patch_pyrimid_level: 2
  patch_size: 6

mapping:
  local_map_half_size: 60
  sliding_thresh: 20

long_term_map:
  map_sliding_en: true
  half_map_size: 400
  sliding_thresh: 40
```

## 4) 实施顺序（固定顺序）
1. `VoxelMapManager` 输出退化指标。
2. `LIVMapper` 按退化/关键帧/降频规则过滤图像。
3. `VIOManager` 加入视觉点上限与超龄淘汰。
4. 图像降采样 + patch 缩减。
5. 滑窗触发时裁剪视觉点。

## 5) 预期效果（明确）
- **CPU**：视觉部分至少下降 40% 左右（以 1/3 帧率 + 小 patch 为基础）。
- **内存**：视觉点规模固定，长时间运行稳定。
- **鲁棒性**：退化时强制提频，避免视觉约束不足。
