# cu_opencv

ROS 2 Humble 下的固定翼投弹视觉节点示例。

目标：

- 订阅 `/image_raw`
- 基于 OpenCV 做颜色分割与轮廓筛选
- 输出投弹目标中心像素坐标

## 依赖边界

- Python 3.10
- ROS 2 Humble
- 系统已安装的 `rclpy` / `cv_bridge`
- 虚拟环境仅用于隔离本项目 Python 包，不影响原有用户环境

## 推荐环境

先让 ROS 2 进入当前 shell：

```bash
source /opt/ros/humble/setup.bash
```

再创建虚拟环境：

```bash
python3.10 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

本项目默认依赖系统已安装的：

- `numpy==1.21.5`
- `opencv-python` 对应系统 OpenCV 4.5.4

如果你的系统里 `cv2` 已可导入，就不要再额外装一套 `opencv-python`，避免 ABI 冲突。

## 构建

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --symlink-install
```

## 运行

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 run cu_vision target_detector
```

如果要实时预览处理结果：

```bash
ros2 launch cu_vision target_detector.launch.py display:=true preview_scale:=0.5
```

## 参数

- `image_topic`: 输入图像话题，默认 `/image_raw`
- `target_color`: `red` 或 `blue`
- `min_area`: 最小轮廓面积
- `display`: 是否打开调试窗口
- `preview_scale`: 预览缩放比例，默认 `1.0`
- `shape_epsilon_ratio`: 多边形近似精度，默认 `0.03`
- `hsv_s_min`: HSV 饱和度下限，默认 `45`
- `hsv_v_min`: HSV 亮度下限，默认 `35`
- `channel_delta`: 主颜色通道需要比其他通道高出的差值，默认 `35`
- `close_kernel_size`: 连接断裂色块的闭运算核大小，默认 `7`
- `close_iterations`: 闭运算次数，默认 `1`
- `open_kernel_size`: 去除小噪点的开运算核大小，默认 `3`
- `min_vertices`: 形状筛选允许的最少多边形顶点数，默认 `4`
- `max_vertices`: 形状筛选允许的最多多边形顶点数，默认 `7`
- `min_solidity`: 轮廓面积 / 凸包面积下限，默认 `0.65`

现场调参建议：

- `target_color_mask` 太多噪声：提高 `hsv_s_min`、`hsv_v_min` 或 `channel_delta`
- `target_color_mask` 靶标缺块：降低 `hsv_s_min`、`hsv_v_min` 或 `channel_delta`
- 阴影把靶标切开：增大 `close_kernel_size` 或 `close_iterations`
- 小噪点太多：增大 `open_kernel_size`
- `target_shape_mask` 经常为空：增大 `max_vertices`、减小 `min_vertices` 或降低 `min_solidity`
- 多边形顶点数不稳定：调 `shape_epsilon_ratio`，值越大，轮廓越简化

调试图像：

- `target_mask` / OpenCV 窗口 `target_color_mask`: 颜色候选区域
- `target_shape_mask`: 通过 5 边形筛选后的目标区域
- `target_debug`: 原图上叠加外接框、中心点和多边形轮廓

输出话题：

- `target_center_px`: 单帧原始检测结果，格式 `[cx, cy, area, vertex_count]`
- `target_center_px_filtered`: 多帧确认和平滑后的结果，格式 `[cx, cy, area, vertex_count, hit_count]`
- `target_pose_camera`: PnP 解算出的目标位姿，`geometry_msgs/PoseStamped`

滤波参数：

- `filter_enable`: 是否启用滤波，默认 `true`
- `min_confirm_frames`: 连续命中多少帧才发布滤波结果，默认 `3`
- `max_missed_frames`: 允许丢失多少帧后重置跟踪，默认 `5`
- `filter_alpha`: 指数平滑系数，默认 `0.35`
- `max_center_jump_px`: 已确认目标允许的最大跳变像素，默认 `220.0`

PnP 参数：

- `enable_pnp`: 是否启用 PnP，默认 `true`
- `target_side_m`: 靶标边长，默认 `1.0`
- `max_reprojection_error_px`: PnP 最大重投影误差，默认 `8.0`
- `camera_info_topic`: 相机内参话题，默认 `/camera_info`
- `camera_fx` / `camera_fy` / `camera_cx` / `camera_cy`: 手动内参，默认 `0.0`

PnP 需要相机内参。如果相机节点没有发布 `/camera_info`，就从标定文件里读出 `fx/fy/cx/cy`，通过 launch 参数手动传入。

## QoS

图像订阅使用 `sensor_data` QoS，也就是 `best_effort` 低延迟模式，适配相机节点这类高频图像流。
