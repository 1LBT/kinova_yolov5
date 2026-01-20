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
from cv_bridge import CvBridge, CvBridgeError

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import CameraInfo, Image

# 导入助手函数
try:
    from helpers.gripper_action_client import set_finger_positions
    from helpers.position_action_client import position_client, move_to_position
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
GRASP_TILT_X = -15.0  # 俯仰角 (Pitch-like tilt)
GRASP_YAW = 45.0      # 水平旋转角 (Yaw)

# 5. 精度控制阈值 (单位: 米)
POSITION_TOLERANCE = 0.01 # XY 位置允许的最大偏差 (5mm)

bridge = CvBridge()
tf_buffer = None
tf_listener = None

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
    if z <= 0.1 or z > 1.5: 
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

def execute_grasp_sequence():
    global MOVING
    
    if LATEST_YOLO_TIME is None or (rospy.Time.now() - LATEST_YOLO_TIME).to_sec() > 2.0:
        rospy.logwarn("未检测到物体 (数据过期)")
        return False

    target = get_target_object()
    if not target:
        return False
        
    rospy.loginfo(f"发现目标: {target['name']} (置信度: {target['confidence']:.2f})")
    
    u_center = int((target['xmin'] + target['xmax']) / 2)
    v_center = int((target['ymin'] + target['ymax']) / 2)
    
    rospy.loginfo(f"目标像素中心: ({u_center}, {v_center})")

    if LATEST_DEPTH_IMG is None:
        rospy.logwarn("无深度图")
        return False
        
    try:
        search_radius = 5
        depth_crop = LATEST_DEPTH_IMG[v_center-search_radius:v_center+search_radius+1, u_center-search_radius:u_center+search_radius+1]
        valid_depths = depth_crop[depth_crop > 0]
        
        if valid_depths.size == 0:
            rospy.logwarn(f"目标点 ({u_center},{v_center}) 深度无效")
            return False
            
        z_depth = np.median(valid_depths)
    except IndexError:
        return False

    cam_coords = pixel_to_3d(u_center, v_center, z_depth)
    if not cam_coords:
        rospy.logwarn("3D 坐标转换失败")
        return False
    
    x, y, z = cam_coords
    rospy.loginfo(f"相机坐标系 (Camera Frame): x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    pose_cam = geometry_msgs.msg.Pose()
    pose_cam.position.x, pose_cam.position.y, pose_cam.position.z = x, y, z
    pose_cam.orientation.w = 1.0
    
    pose_base = convert_pose(pose_cam, CAMERA_FRAME, BASE_FRAME)
    if not pose_base:
        rospy.logerr("坐标变换失败")
        return False

    if not validate_grasp_position(pose_base):
        return False

    pose_base.position.x += MANUAL_OFFSET_X
    pose_base.position.y += MANUAL_OFFSET_Y
    pose_base.position.z += MANUAL_OFFSET_Z

    rospy.loginfo(f"基座坐标系 (Base Frame): x={pose_base.position.x:.3f}, y={pose_base.position.y:.3f}, z={pose_base.position.z:.3f}")
    
    # --- DEBUG TF ---
    try:
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = BASE_FRAME
        t.child_frame_id = "debug_grasp_target"
        t.transform.translation.x = pose_base.position.x
        t.transform.translation.y = pose_base.position.y
        t.transform.translation.z = pose_base.position.z
        t.transform.rotation = pose_base.orientation
        for _ in range(10): 
            t.header.stamp = rospy.Time.now()
            br.sendTransform(t)
            rospy.sleep(0.05)
    except: pass

    MOVING = True
    
    pre_grasp_z = pose_base.position.z + TOOL_LENGTH_OFFSET + 0.10
    pre_grasp_pos = [pose_base.position.x, pose_base.position.y, pre_grasp_z]
    
    # === 抓取姿态计算 (Manual Orientation) ===
    # 使用手动配置的 Tilt 和 Yaw
    tilt_rad = np.radians(GRASP_TILT_X)
    yaw_rad = np.radians(GRASP_YAW)
    
    # 构造姿态: Roll=180(向下) + Tilt(俯仰) + Yaw(旋转)
    q = tft.quaternion_from_euler(np.pi + tilt_rad, 0, yaw_rad)
    orientation = [q[0], q[1], q[2], q[3]]

    rospy.loginfo(f"移动到预抓取点 (Z_wrist={pre_grasp_z:.3f}, Tilt={GRASP_TILT_X}°, Yaw={GRASP_YAW}°)...")
    
    # === 闭环修正逻辑 (Closed-loop Correction) ===
    correction_attempts = 3
    arrived_successfully = False

    for i in range(correction_attempts + 1):
        # 尝试移动
        if not move_to_pose_with_retry(pre_grasp_pos, orientation, max_attempts=1):
            rospy.logwarn("移动指令执行失败")
        
        rospy.sleep(0.5) # 等待物理稳定
        
        # 检查是否达标
        if check_reached(pre_grasp_pos, tolerance=POSITION_TOLERANCE, xy_only=True):
            rospy.loginfo(f"精确到达预抓取点 (尝试次数: {i+1})")
            arrived_successfully = True
            break
        
        if i < correction_attempts:
            rospy.logwarn(f"位置存在偏差，正在执行第 {i+1} 次修正...")
    
    if not arrived_successfully:
        rospy.logwarn("多次修正后仍无法精确到达目标，放弃本次抓取以防碰撞。")
        MOVING = False
        return False
    # ============================================

    rospy.loginfo("打开夹爪...")
    set_finger_positions([0.0, 0.0])
    rospy.sleep(0.5)
    
    grasp_z = pose_base.position.z + TOOL_LENGTH_OFFSET
    grasp_pos = [pose_base.position.x, pose_base.position.y, grasp_z]
    
    rospy.loginfo(f"下降抓取 (Z_wrist={grasp_z:.3f})...")
    # 使用配置好的姿态下降
    move_to_position(grasp_pos, orientation)
    rospy.sleep(0.5)
    
    rospy.loginfo("闭合夹爪...")
    set_finger_positions([6000.0, 6000.0])
    rospy.sleep(1.0)
    
    lift_pos = [pose_base.position.x, pose_base.position.y, pre_grasp_z]
    rospy.loginfo("提起物体...")
    move_to_position(lift_pos, orientation)
    
    MOVING = False
    return True

def main():
    rospy.init_node('kinova_yolo_grasp')
    
    rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, camera_info_callback)
    rospy.Subscriber(YOLO_TOPIC, std_msgs.msg.String, yolo_callback)
    rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback)
    rospy.Subscriber(POSE_TOPIC, geometry_msgs.msg.PoseStamped, robot_position_callback)
    rospy.Subscriber(WRENCH_TOPIC, geometry_msgs.msg.WrenchStamped, robot_wrench_callback)
    
    global tf_buffer, tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.loginfo("等待相机内参...")
    while CAMERA_INTRINSICS['fx'] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
        
    rospy.loginfo("系统初始化完成。")
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
            rospy.loginfo("抓取流程完成。")
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
