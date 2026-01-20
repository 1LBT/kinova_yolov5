#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf.transformations as tft
import numpy as np
import sys
import time
import json
import cv2
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import CameraInfo, Image

# 导入助手函数
try:
    from helpers.gripper_action_client import set_finger_positions
    # 修改导入方式以便访问 action client 对象
    import helpers.position_action_client as pac
    from helpers.position_action_client import move_to_position
    from helpers.transforms import convert_pose
except ImportError as e:
    rospy.logerr("导入助手模块失败: %s", e)
    sys.exit(1)

# 全局变量
LATEST_YOLO_DETECTIONS = []
LATEST_YOLO_TIME = None
LATEST_DEPTH_IMG = None
CAMERA_INTRINSICS = {'fx': None, 'fy': None, 'cx': None, 'cy': None}

MOVING = False
CURR_Z = 0.0
CURRENT_POSE = None

# 参数
CAMERA_FRAME = rospy.get_param('~camera_frame', 'yolo_frame') # 【关键修正】使用自定义的专用Frame，避开驱动TF冲突，确保方向正确
BASE_FRAME = rospy.get_param('~base_frame', 'm1n6s200_link_base')
YOLO_TOPIC = rospy.get_param('~yolo_topic', '/yolov5/detections')
DEPTH_TOPIC = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw')
CAMERA_INFO_TOPIC = rospy.get_param('~camera_info_topic', '/camera/color/camera_info') # 【关键修正】使用RGB内参，确保比例正确
POSE_TOPIC = rospy.get_param('~pose_topic', '/m1n6s200_driver/out/tool_pose')
WRENCH_TOPIC = rospy.get_param('~wrench_topic', '/m1n6s200_driver/out/tool_wrench')

# 固定的Home位置
FIXED_HOME_POSITION = [0.0535, -0.3381, 0.2688]
FIXED_HOME_ORIENTATION = [-0.9948, -0.0999, 0.0036, 0.0189]

# 夹爪长度补偿 (已根据要求设置为 0)
TOOL_LENGTH_OFFSET = -0.03

# 1. 目标白名单
TARGET_CLASSES = ['sports ball']

# 2. 手动坐标补偿 (单位: 米)
MANUAL_OFFSET_X = 0
MANUAL_OFFSET_Y = 0
MANUAL_OFFSET_Z = 0

# 3. 坐标轴方向修正 (1.0 或 -1.0)
X_SIGN = 1.0
Y_SIGN = 1.0

# 4. 手动抓取姿态修正 (单位: 度)
# GRASP_TILT_X = -25.0  # (已弃用，改为自适应列表)
# 候选俯仰角列表 (相对于垂直向下 180 度的偏移)
# 优先级: 垂直(0) -> 微倾斜(-15/15) -> 大倾斜(-25/25)
# 注意: +15 = Pitch 195度, -15 = Pitch 165度
GRASP_PITCH_CANDIDATES = [0.0, -15.0, 15.0, -25.0, 25.0]
GRASP_YAW = 0.0      # 水平旋转角 (Yaw)

# 5. 精度控制阈值 (单位: 米)
POSITION_TOLERANCE = 0.005 # XY 位置允许的最大偏差 (5mm)
BLIND_ZONE_MIN_DIST = 0.01 # 视觉伺服最小距离 (小于此距离进入盲区，避免相机失焦/遮挡)

# 6. 夹爪几何参数 (设为 0 以消除额外偏移)
GRIPPER_LENGTH = 0.0 

bridge = CvBridge()
tf_buffer = None
tf_listener = None
# 新增 TF 广播器
tf_broadcaster = None

class PoseFilter:
    """坐标平滑滤波器 (移动平均)"""
    def __init__(self, window_size=10): # 增加窗口大小到 10 (约1秒数据)
        self.window_size = window_size
        self.history_x = deque(maxlen=window_size)
        self.history_y = deque(maxlen=window_size)
    
    def update(self, x, y):
        # 异常值剔除：如果历史不为空，且新点距离均值太远(>5cm)，则视为误检或噪声，忽略该点
        # 这样做可以防止机械臂因为偶尔的识别错误而剧烈抖动
        if len(self.history_x) > 0:
            avg_x = np.mean(self.history_x)
            avg_y = np.mean(self.history_y)
            dist = np.sqrt((x - avg_x)**2 + (y - avg_y)**2)
            if dist > 0.05:
                rospy.logwarn(f"检测到异常坐标跳变 ({dist:.3f}m)，已忽略该点(可能是噪声)。")
                return avg_x, avg_y # 返回旧的平均值，不更新历史
        
        self.history_x.append(x)
        self.history_y.append(y)
        
        return np.mean(self.history_x), np.mean(self.history_y)

def calculate_wrist_target(object_pos, orientation_quat, offset_dist=0.0):
    """
    根据物体位置和进近姿态，反推手腕应该所在的位置。
    补偿因倾斜 (Tilt) 导致的 X/Y/Z 偏移。
    """
    # 1. 将四元数转换为旋转矩阵
    rot_matrix = tft.quaternion_matrix(orientation_quat)
    
    # 2. 提取进近向量 (假设夹爪沿自身 Z 轴伸出)
    approach_vector = rot_matrix[:3, 2] # 第三列是 Z 轴向量
    
    # 3. 计算手腕位置
    # Wrist = Object - (Vector * (GripperLength + Offset))
    total_dist = GRIPPER_LENGTH + offset_dist
    
    wrist_x = object_pos[0] - approach_vector[0] * total_dist
    wrist_y = object_pos[1] - approach_vector[1] * total_dist
    wrist_z = object_pos[2] - approach_vector[2] * total_dist
    
    return [wrist_x, wrist_y, wrist_z]

def camera_info_callback(msg):
    global CAMERA_INTRINSICS
    if CAMERA_INTRINSICS['fx'] is None:
        CAMERA_INTRINSICS['fx'] = msg.K[0]
        CAMERA_INTRINSICS['cx'] = msg.K[2]
        CAMERA_INTRINSICS['fy'] = msg.K[4]
        CAMERA_INTRINSICS['cy'] = msg.K[5]
        rospy.loginfo(f"接收到相机内参: fx={CAMERA_INTRINSICS['fx']}, fy={CAMERA_INTRINSICS['fy']}")

def yolo_callback(msg):
    global LATEST_YOLO_DETECTIONS, LATEST_YOLO_TIME
    try:
        raw_detections = json.loads(msg.data)
        
        # 过滤: 只保留白名单内的物体
        if TARGET_CLASSES:
            filtered_dets = [d for d in raw_detections if d['name'] in TARGET_CLASSES]
        else:
            filtered_dets = raw_detections
            
        LATEST_YOLO_DETECTIONS = filtered_dets
        LATEST_YOLO_TIME = rospy.Time.now()
        
    except Exception as e:
        rospy.logerr(f"YOLO 解析错误: {e}")

def depth_callback(msg):
    global LATEST_DEPTH_IMG
    try:
        LATEST_DEPTH_IMG = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.logerr(f"深度图转换错误: {e}")

def robot_position_callback(msg):
    """监控机器人位置"""
    global CURR_Z, CURRENT_POSE
    CURR_Z = msg.pose.position.z
    CURRENT_POSE = msg.pose

def robot_wrench_callback(msg):
    """监控力矩以在碰撞时停止运动 (简化版)"""
    global MOVING
    if MOVING and msg.wrench.force.z < -5.0: 
        rospy.logwarn(f'检测到碰撞力 (Z={msg.wrench.force.z:.2f}N)，可能已接触物体')
        pass

def get_target_object():
    """选择最佳抓取目标 (置信度最高)"""
    if not LATEST_YOLO_DETECTIONS:
        return None
    sorted_dets = sorted(LATEST_YOLO_DETECTIONS, key=lambda x: x['confidence'], reverse=True)
    return sorted_dets[0]

def check_reached(target_pos, tolerance=0.01, xy_only=False):
    """检查是否到达目标位置"""
    if CURRENT_POSE is None:
        rospy.logwarn("无法获取当前机械臂位置 (CURRENT_POSE is None)")
        return False
    
    dx = CURRENT_POSE.position.x - target_pos[0]
    dy = CURRENT_POSE.position.y - target_pos[1]
    dz = CURRENT_POSE.position.z - target_pos[2]
    
    if xy_only:
        error = np.sqrt(dx**2 + dy**2)
        error_type = "XY"
    else:
        error = np.sqrt(dx**2 + dy**2 + dz**2)
        error_type = "Euclidean"
        
    if error <= tolerance:
        return True
    else:
        rospy.logwarn(f"{error_type} 位置误差过大: {error:.4f}m (阈值: {tolerance}m)")
        return False

def pixel_to_3d(u, v, z_mm):
    """像素坐标 + 深度(mm) -> 相机坐标系(m)"""
    if CAMERA_INTRINSICS['fx'] is None:
        return None
    z = z_mm / 1000.0
    if z <= 0.05 or z > 1.5: # 放宽最小距离到 5cm
        return None
    
    x = ((u - CAMERA_INTRINSICS['cx']) / CAMERA_INTRINSICS['fx'] * z) * X_SIGN
    y = ((v - CAMERA_INTRINSICS['cy']) / CAMERA_INTRINSICS['fy'] * z) * Y_SIGN
    return (x, y, z)

def validate_grasp_position(pose_base):
    """验证抓取位置是否安全"""
    if pose_base.position.z < -0.10: 
        rospy.logwarn(f"抓取点过低 (Z={pose_base.position.z:.3f})，可能撞击桌面")
        return False
    if pose_base.position.z > 0.8:
        rospy.logwarn("抓取点过高")
        return False
        
    r = np.sqrt(pose_base.position.x**2 + pose_base.position.y**2)
    if r > 0.7:
        rospy.logwarn(f"抓取点超出工作半径 (R={r:.3f})")
        return False
    if r < 0.15:
        rospy.logwarn(f"抓取点过近 (R={r:.3f})")
        return False
        
    return True

def move_to_pose_with_retry(pose_list, orient_list, max_attempts=3):
    """带重试的移动函数"""
    for attempt in range(max_attempts):
        success = move_to_position(pose_list, orient_list)
        if success:
            return True
        rospy.logwarn(f"移动失败，重试中 ({attempt+1}/{max_attempts})...")
        rospy.sleep(1.0)
    return False

def move_to_fixed_home():
    """移动到Home位置"""
    rospy.loginfo("正在移动到 Home 位置...")
    return move_to_pose_with_retry(FIXED_HOME_POSITION, FIXED_HOME_ORIENTATION)

def send_pose_goal_non_blocking(pos, orient):
    """发送非阻塞移动指令 (Fire and Forget)"""
    if pac.position_client is None:
        pac.init_action()
        
    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=BASE_FRAME)
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.pose.position = geometry_msgs.msg.Point(x=pos[0], y=pos[1], z=pos[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(x=orient[0], y=orient[1], z=orient[2], w=orient[3])
    
    pac.position_client.send_goal(goal)

def visual_servo_approach(orientation, target_z_final, fallback_xy=None):
    """
    视觉伺服逼近：在移动过程中持续更新目标坐标 (Closed-Loop)
    停止条件：达到 target_z_final (预抓取点高度)
    """
    global tf_broadcaster # 引用全局广播器
    rospy.loginfo("启动视觉伺服逼近 (Visual Servo Approach)...")
    
    # 初始化平滑滤波器
    pose_filter = PoseFilter(window_size=10) # 增大窗口大小到10
    
    # 0. 如果有 fallback_xy (来自高处的可靠检测)，先用它预热滤波器
    if fallback_xy:
        rospy.loginfo(f"使用高处检测结果预热滤波器: {fallback_xy}")
        # 预填入几次，增加其初始权重
        for _ in range(3):
            pose_filter.update(fallback_xy[0], fallback_xy[1])

    # 1. 获取初始目标 (带重试机制，防止因数据过期导致失败)
    initial_target_pose = None
    last_valid_z = None
    for _ in range(10): # 尝试最多 2.0 秒 (10 * 0.2s)
        initial_target_pose = get_target_pose_base()
        if initial_target_pose:
            last_valid_z = initial_target_pose.position.z
            break
        rospy.loginfo("等待有效的 YOLO 检测数据...")
        rospy.sleep(0.2)
        
    if not initial_target_pose:
        if fallback_xy:
            rospy.logwarn("视觉伺服启动失败（视野遮挡？），降级为开环模式 (Open-Loop)，使用最后已知坐标。" )
            # 直接使用传入的 fallback_xy，不做伺服，直接下降
            send_pose_goal_non_blocking([fallback_xy[0], fallback_xy[1], target_z_final], orientation)
            # 等待移动完成
            while not rospy.is_shutdown():
                if CURR_Z <= target_z_final + 0.005: break
                rospy.sleep(0.1)
            return True, fallback_xy
        else:
            rospy.logwarn("视觉伺服启动失败：长时间未获取到有效目标且无备用坐标")
            return False, None
        
    # 初始化滤波器并获取第一个平滑坐标
    current_target_xy = list(pose_filter.update(initial_target_pose.position.x, initial_target_pose.position.y))
    last_processed_time = LATEST_YOLO_TIME # 记录最后处理的时间戳
    
    # 设定终点高度 (预抓取点)
    servo_end_z = target_z_final
    rospy.loginfo(f"伺服目标高度 (预抓取点): {servo_end_z:.3f}m")
    
    # 启动移动 (朝向当前物体 XY 下降)
    send_pose_goal_non_blocking([current_target_xy[0], current_target_xy[1], servo_end_z], orientation)
    
    # 初始化 Z 轴参考
    if last_valid_z is None: last_valid_z = initial_target_pose.position.z

    rate = rospy.Rate(10) # 10Hz 监控频率
    while not rospy.is_shutdown():
        # 停止条件：距离目标高度小于 5mm
        if CURR_Z <= servo_end_z + 0.005: 
            break
            
        # --- 持续发布 TF (即使没有新检测) ---
        if tf_broadcaster and last_valid_z is not None:
            tf_broadcaster.sendTransform(
                (current_target_xy[0], current_target_xy[1], last_valid_z), 
                (0, 0, 0, 1), # 无旋转
                rospy.Time.now(),
                "filtered_target_pose",
                BASE_FRAME
            )
        # ---------------------------------

        # 仅当有 *新* 的 YOLO 数据时才进行滤波和更新
        if LATEST_YOLO_TIME and LATEST_YOLO_TIME > last_processed_time:
            last_processed_time = LATEST_YOLO_TIME # 更新时间戳
            
            # 尝试获取更新的目标坐标
            raw_pose = get_target_pose_base()
            if raw_pose:
                last_valid_z = raw_pose.position.z # 更新 Z 值
                # 使用滤波器平滑坐标
                smoothed_x, smoothed_y = pose_filter.update(raw_pose.position.x, raw_pose.position.y)
                
                # 计算新平滑目标与当前目标之间的距离
                dist_change = np.sqrt((smoothed_x - current_target_xy[0])**2 + (smoothed_y - current_target_xy[1])**2)
                
                # 只有当变化超过 5mm 时才更新指令
                if dist_change > 0.005:
                    rospy.loginfo(f"坐标更新 (d={dist_change:.3f}m) -> [{smoothed_x:.3f}, {smoothed_y:.3f}]")
                    current_target_xy = [smoothed_x, smoothed_y]
                    send_pose_goal_non_blocking([smoothed_x, smoothed_y, servo_end_z], orientation)
        
        rate.sleep()
        
    rospy.loginfo(f"视觉伺服在预抓取高度停止。最后确认物体位置: XY=[{current_target_xy[0]:.3f}, {current_target_xy[1]:.3f}]")
    return True, current_target_xy


def get_target_pose_base():
    """获取当前目标在基座下的 Pose (封装了检测+转换逻辑)"""
    if LATEST_YOLO_TIME is None or (rospy.Time.now() - LATEST_YOLO_TIME).to_sec() > 1.0:
        rospy.logwarn("get_target_pose_base: YOLO 数据过期")
        return None # 数据过期
        
    target = get_target_object()
    if not target:
        rospy.logwarn("get_target_pose_base: 无检测目标")
        return None
    if LATEST_DEPTH_IMG is None: 
        rospy.logwarn("get_target_pose_base: 无深度图像")
        return None
    
    u = int((target['xmin'] + target['xmax']) / 2)
    v = int((target['ymin'] + target['ymax']) / 2)
    
    try:
        d_crop = LATEST_DEPTH_IMG[v-5:v+6, u-5:u+6]
        valid = d_crop[d_crop > 0]
        if valid.size == 0: 
            rospy.logwarn("get_target_pose_base: 目标区域深度无效 (全0/NaN)")
            return None
        z = np.median(valid)
    except:
        return None
    
    cam_xyz = pixel_to_3d(u, v, z)
    if not cam_xyz: 
        rospy.logwarn(f"get_target_pose_base: 3D转换失败 (Z={z/1000.0:.3f}m 超出范围)")
        return None
    
    p_cam = geometry_msgs.msg.Pose()
    p_cam.position.x, p_cam.position.y, p_cam.position.z = cam_xyz
    p_cam.orientation.w = 1.0
    
    p_base = convert_pose(p_cam, CAMERA_FRAME, BASE_FRAME)
    if p_base:
        p_base.position.x += MANUAL_OFFSET_X
        p_base.position.y += MANUAL_OFFSET_Y
        p_base.position.z += MANUAL_OFFSET_Z
    else:
        rospy.logwarn("get_target_pose_base: 坐标系转换失败 (TF问题)")
        
    return p_base

def execute_grasp_sequence():
    global MOVING
    
    # 初次检测
    pose_base = get_target_pose_base()
    if not pose_base:
        rospy.logwarn("未检测到目标或无法计算坐标")
        return False
        
    if not validate_grasp_position(pose_base):
        return False

    MOVING = True
    
    # === 自适应进近策略 (Adaptive Approach) ===
    selected_orientation = None
    yaw_rad = np.radians(GRASP_YAW)
    entry_success = False

    # 目标物体坐标 (按下回车时的可靠坐标)
    object_pos_initial = [pose_base.position.x, pose_base.position.y, pose_base.position.z]

    for pitch_offset in GRASP_PITCH_CANDIDATES:
        rospy.loginfo(f"尝试进近角度: 垂直偏移 {pitch_offset} 度...")
        tilt_rad = np.radians(pitch_offset)
        q = tft.quaternion_from_euler(np.pi + tilt_rad, 0, yaw_rad)
        candidate_orient = [q[0], q[1], q[2], q[3]]
        
        # 计算该角度下的手腕测试点 (15cm 处)
        wrist_test = calculate_wrist_target(object_pos_initial, candidate_orient, offset_dist=0.15)
        
        move_result = move_to_position(wrist_test, candidate_orient)
        rospy.sleep(0.5)
        
        if move_result and check_reached(wrist_test, tolerance=POSITION_TOLERANCE, xy_only=True):
            rospy.loginfo(f"角度 {pitch_offset} 度 可用。")
            selected_orientation = candidate_orient
            entry_success = True
            break
            
    if not entry_success:
        rospy.logerr("所有进近角度均失败。" )
        MOVING = False
        return False

    # === [Phase 1] 视觉伺服逼近 (Visual Servo Approach) ===
    # 计算预抓取点 (10cm 处) 的高度
    pre_grasp_z_target = calculate_wrist_target(object_pos_initial, selected_orientation, offset_dist=0.10)[2]
    
    # 传入 object_pos_initial[0:2] 作为 fallback_xy
    success_servo, last_xy = visual_servo_approach(selected_orientation, pre_grasp_z_target, fallback_xy=object_pos_initial[0:2])
    if not success_servo:
        MOVING = False
        return False
        
    # === [Phase 2] 闭环位置修正 (Closed-loop Correction) ===
    object_pos_final = [last_xy[0], last_xy[1], pose_base.position.z]
    pre_grasp_pos = calculate_wrist_target(object_pos_final, selected_orientation, offset_dist=0.10)
    
    rospy.loginfo("执行预抓取点最终修正...")
    correction_attempts = 2
    for i in range(correction_attempts + 1):
        move_to_pose_with_retry(pre_grasp_pos, selected_orientation, max_attempts=1)
        rospy.sleep(0.3)
        if check_reached(pre_grasp_pos, tolerance=POSITION_TOLERANCE, xy_only=True):
            break

    # === [Phase 3] 盲区下降与执行 ===
    rospy.loginfo("打开夹爪...")
    set_finger_positions([0.0, 0.0])
    rospy.sleep(0.5)
    
    grasp_pos = calculate_wrist_target(object_pos_final, selected_orientation, offset_dist=-0.01)
    rospy.loginfo(f"下降抓取 (目标 XY: {object_pos_final[0]:.3f}, {object_pos_final[1]:.3f})...")
    move_to_position(grasp_pos, selected_orientation)
    rospy.sleep(0.5)
    
    rospy.loginfo("闭合夹爪...")
    set_finger_positions([6000.0, 6000.0])
    rospy.sleep(1.0)
    
    rospy.loginfo("提起物体...")
    move_to_position(pre_grasp_pos, selected_orientation)
    
    MOVING = False
    return True

def main():
    rospy.init_node('kinova_yolo_grasp')
    
    rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, camera_info_callback)
    rospy.Subscriber(YOLO_TOPIC, std_msgs.msg.String, yolo_callback)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback)
    rospy.Subscriber(POSE_TOPIC, geometry_msgs.msg.PoseStamped, robot_position_callback)
    rospy.Subscriber(WRENCH_TOPIC, geometry_msgs.msg.WrenchStamped, robot_wrench_callback)
    
    global tf_buffer, tf_listener, tf_broadcaster
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf.TransformBroadcaster() # 初始化广播器
    rospy.loginfo(f"TF Broadcaster initialized: {tf_broadcaster}")
    
    rospy.loginfo("等待相机内参...")
    while CAMERA_INTRINSICS['fx'] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
        
    rospy.loginfo("系统初始化完成。" )
    move_to_fixed_home()
    
    while not rospy.is_shutdown():
        user_input = input("\n按 Enter 开始检测并抓取, 输入 'h' 或 ' '(空格)+Enter 返回 Home (q 退出): ")
        if user_input.lower() == 'q':
            break
        elif user_input.lower() == 'h' or user_input == ' ':
            move_to_fixed_home()
            continue
            
        rospy.loginfo("正在寻找目标...")
        rospy.sleep(1.0)
        
        if execute_grasp_sequence():
            rospy.loginfo("抓取流程完成。" )
            move_to_fixed_home()
            rospy.loginfo("放下物体...")
            set_finger_positions([0.0, 0.0])
            rospy.sleep(1.0)
        else:
            rospy.logwarn("抓取流程中断或未发现目标，正在返回 Home...")
            move_to_fixed_home()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass