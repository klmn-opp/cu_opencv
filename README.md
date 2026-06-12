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

## QoS

图像订阅使用 `sensor_data` QoS，也就是 `best_effort` 低延迟模式，适配相机节点这类高频图像流。
