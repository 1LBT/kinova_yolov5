# 手眼标定结果应用指南

## 手眼标定结果 (2024-12-19)

### 变换矩阵
```
从末端执行器到相机链接的变换矩阵:
[[-0.97450433  0.22284222  0.02612782 -0.00688656]
 [-0.22153716 -0.97410183  0.04524275  0.06518315]
 [ 0.03553315  0.03830097  0.99863428 -0.0858317 ]
 [ 0.          0.          0.          1.        ]]
```

### 平移向量
```
x: -0.006886560877924761 米
y:  0.06518314780922749  米
z: -0.08583169582135934 米
```

### 旋转四元数
```
x: -0.01551792345147061
y: -0.02102504775817704
z: -0.9933831640608743
w:  0.11183483705414576
```

## 已更新的文件

### 1. Launch文件

#### `handeye_static_tf.launch`
- 发布从末端执行器到相机的静态TF
- 使用手眼标定的精确四元数
- 同时发布camera_link到camera_depth_optical_frame的变换

#### `camera_tf_broadcaster.launch`
- 与handeye_static_tf.launch功能相同
- 可以根据需要选择使用

#### `ggcnn_grasp_with_calibration.launch`
- 完整的抓取系统启动文件
- 自动加载手眼标定TF

### 2. URDF文件

#### `m1n6s200_with_depth_cam.xacro`
- 更新了相机安装参数
- 注意：建议使用launch文件中的static_transform_publisher而不是URDF中的joint

### 3. 抓取程序

#### `kinova_open_loop_grasp.py`
- 修复了坐标变换逻辑
- 直接使用TF树进行变换，不再手动应用手眼标定矩阵
- 增加了详细的调试日志

## 使用方法

### 方法1：使用完整的launch文件（推荐）

```bash
# 终端1：启动机械臂驱动（如果还没启动）
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n6s200

# 终端2：启动手眼标定TF和相机
roslaunch ggcnn_kinova_grasping handeye_static_tf.launch

# 终端3：启动GGCNN检测
rosrun ggcnn_kinova_grasping run_ggcnn.py

# 终端4：启动抓取程序
rosrun ggcnn_kinova_grasping kinova_open_loop_grasp.py
```

### 方法2：单独启动各个组件

```bash
# 1. 启动手眼标定TF
roslaunch ggcnn_kinova_grasping handeye_static_tf.launch

# 2. 启动相机（如果需要）
roslaunch ggcnn_kinova_grasping wrist_camera.launch

# 3. 启动抓取程序
rosrun ggcnn_kinova_grasping kinova_open_loop_grasp.py
```

## 验证TF变换

### 使用调试工具
```bash
# 运行TF调试脚本
rosrun ggcnn_kinova_grasping debug_tf_chain.py

# 查看TF树
rosrun tf view_frames
evince frames.pdf

# 实时查看TF变换
rosrun tf tf_echo m1n6s200_link_base camera_depth_optical_frame
```

### 检查TF是否正确

正确的TF链应该是：
```
m1n6s200_link_base
  -> m1n6s200_link_1
    -> ... (其他关节)
      -> m1n6s200_end_effector
        -> camera_link (手眼标定)
          -> camera_depth_optical_frame (ROS相机约定)
```

## 预期结果

使用正确的手眼标定后：

1. **TF变换验证**
   - 末端执行器到相机的距离约为0.11米
   - 相机主要在末端执行器的下方（z=-0.086）和侧面（y=0.065）

2. **抓取位置验证**
   - 相机坐标系中的点(0, 0, 0.5)应该变换到基座坐标系中合理的位置
   - Z坐标应该在0.05-0.5米之间（桌面以上，工作空间内）

3. **抓取成功率**
   - 抓取位置应该准确对应物体位置
   - 不应该出现"Z坐标过低"的警告

## 故障排除

### 问题1：TF变换链不完整
**症状**: 运行debug_tf_chain.py时报错"无法获取变换"

**解决方案**:
```bash
# 检查是否启动了handeye_static_tf.launch
roslaunch ggcnn_kinova_grasping handeye_static_tf.launch

# 检查TF是否发布
rostopic echo /tf_static
```

### 问题2：抓取位置Z坐标过低
**症状**: 日志显示"抓取Z坐标不合理: 0.033米"

**解决方案**:
- 确认使用了正确的手眼标定参数
- 运行debug_tf_chain.py验证TF变换
- 检查是否有其他TF发布器冲突

### 问题3：多个TF发布器冲突
**症状**: TF树中有重复的变换

**解决方案**:
```bash
# 查找所有static_transform_publisher节点
rosnode list | grep static

# 关闭冲突的节点
rosnode kill /camera_tf_broadcaster  # 如果有的话
```

## 重新标定

如果需要重新进行手眼标定：

```bash
# 1. 启动标定程序
roslaunch easy_handeye calibrate.launch

# 2. 按照提示移动机械臂到不同位置采集数据

# 3. 计算标定结果

# 4. 更新以下文件中的参数：
#    - handeye_static_tf.launch
#    - camera_tf_broadcaster.launch
#    - m1n6s200_with_depth_cam.xacro (可选)
```

## 参考资料

- [ROS TF教程](http://wiki.ros.org/tf/Tutorials)
- [REP-103: 标准坐标系约定](https://www.ros.org/reps/rep-0103.html)
- [easy_handeye手眼标定工具](https://github.com/IFL-CAMP/easy_handeye)

## 更新日志

- **2024-12-19**: 应用新的手眼标定结果
  - Translation: (-0.0069, 0.0652, -0.0858)
  - Rotation: (-0.0155, -0.0210, -0.9934, 0.1118)
  - 修复了坐标变换逻辑
  - 更新了所有相关文件
