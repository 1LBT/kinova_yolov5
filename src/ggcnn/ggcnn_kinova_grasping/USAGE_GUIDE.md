# Kinova机械臂开环抓取程序使用指南

## 快速开始

### 1. 环境准备
```bash
# 进入工作空间
cd ~/kinova_ws

# 编译工作空间（如果需要）
catkin_make

# 设置环境变量
source devel/setup.bash
```

### 2. 启动系统组件

#### 启动机械臂驱动
```bash
# 终端1: 启动机械臂驱动
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n6s200
```

#### 启动相机和TF
```bash
# 终端2: 启动相机和手眼标定TF
roslaunch ggcnn_kinova_grasping handeye_static_tf.launch
```

#### 启动GGCNN视觉处理
```bash
# 终端3: 启动GGCNN
rosrun ggcnn_kinova_grasping run_ggcnn.py
```

### 3. 运行抓取程序

#### 方法1: 使用测试脚本（推荐）
```bash
# 终端4: 运行测试脚本
./src/kinova-ros/ggcnn/ggcnn_kinova_grasping/scripts/run_tests.sh
```

#### 方法2: 直接运行程序
```bash
# 运行修改后的开环抓取程序
rosrun ggcnn_kinova_grasping kinova_open_loop_grasp.py

# 或运行XY位置控制测试
rosrun ggcnn_kinova_grasping test_xy_control.py
```

## 程序功能说明

### 修改后的开环抓取程序特点

1. **TF变换链调试**
   - 启动时自动检查TF连接
   - 打印完整的TF树结构
   - 验证坐标变换的正确性

2. **分步抓取流程**
   - 步骤1: 移动到目标XY位置（保持当前高度）
   - 步骤2: 下降到接近位置
   - 步骤3: 速度控制精确下降
   - 步骤4: 闭合夹爪
   - 步骤5: 抬升到安全高度

3. **增强的可视化**
   - 在RViz中显示抓取点标记
   - 显示中间移动点
   - 完整的TF坐标系显示

4. **错误处理和重试**
   - 位置精度验证
   - 自动重试机制
   - 详细的错误日志

### XY位置控制测试程序

提供以下测试功能：
- 手动测试点验证
- 坐标变换测试
- TF调试信息显示

## RViz可视化设置

### 启动RViz
```bash
# 终端5: 启动RViz
rosrun rviz rviz
```

### 添加显示项
1. **TF显示**
   - Add → TF
   - 显示所有坐标系关系

2. **标记显示**
   - Add → Marker
   - Topic: /visualization_marker
   - 显示抓取点和中间点

3. **机械臂模型**
   - Add → RobotModel
   - 显示机械臂当前状态

## 故障排除

### 常见问题及解决方案

#### 1. TF变换失败
```bash
# 检查TF树
rosrun tf view_frames
evince frames.pdf

# 监控特定变换
rosrun tf tf_echo m1n6s200_link_base camera_depth_optical_frame
```

**可能原因:**
- robot_state_publisher未运行
- 多个TF发布器冲突
- 坐标系名称不匹配

#### 2. 机械臂不移动
```bash
# 检查机械臂话题
rostopic list | grep m1n6s200

# 检查位置控制器状态
rostopic echo /m1n6s200_driver/out/tool_pose
```

**可能原因:**
- 机械臂驱动未启动
- 控制器未激活
- 目标位置超出工作空间

#### 3. GGCNN无输出
```bash
# 检查GGCNN话题
rostopic echo /ggcnn/out/command

# 检查相机话题
rostopic list | grep camera
```

**可能原因:**
- 相机未启动
- 深度图像质量差
- GGCNN模型加载失败

### 调试命令

```bash
# 查看所有话题
rostopic list

# 查看节点状态
rosnode list

# 检查TF变换
rosrun tf tf_monitor

# 查看系统日志
rosout_agg
```

## 参数调整

### 位置控制参数
在`kinova_open_loop_grasp.py`中可以调整：
```python
# XY位置控制精度
tolerance = 0.01  # 米

# 最大重试次数
max_attempts = 3

# 接近位置偏移
approach_offset = 0.15  # 米
```

### 工作空间限制
```python
# X范围: -0.5 到 0.7 米
# Y范围: ±0.9 米
# Z范围: -0.05 到 0.7 米
# 距离基座: 0.15 到 0.85 米
```

### TF超时设置
```python
# TF查找超时
timeout = 5.0  # 秒

# TF连接检查重试
max_retries = 3
```

## 性能优化建议

1. **提高抓取精度**
   - 减小tolerance值（如0.005米）
   - 增加重试次数
   - 优化相机标定

2. **提高运行速度**
   - 减少sleep时间
   - 优化移动路径
   - 并行处理部分操作

3. **增强稳定性**
   - 增加异常处理
   - 添加更多验证步骤
   - 优化力控制参数

## 日志分析

### 正常运行日志示例
```
[INFO] TF变换链验证成功!
[INFO] 步骤1: 移动到目标XY位置
[INFO] XY位置控制成功! 误差: 0.008米 (< 0.010)
[INFO] 步骤2: 下降到接近位置
[INFO] 步骤3: 速度控制下降到目标位置
[INFO] 抓取操作完成!
```

### 错误日志分析
- `TF变换失败`: 检查TF发布器和坐标系名称
- `位置精度不足`: 调整tolerance参数或检查机械臂控制器
- `移动超时`: 检查目标位置是否在工作空间内

## 联系和支持

如果遇到问题，请：
1. 查看详细的日志输出
2. 使用测试程序验证各个组件
3. 检查TF树结构和话题状态
4. 参考MODIFICATION_SUMMARY.md了解修改详情