#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf.transformations as tft
import numpy as np
import sys
import time

import kinova_msgs.msg
import kinova_msgs.srv
import std_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import CameraInfo
import actionlib

# 导入助手函数
try:
    from helpers.gripper_action_client import set_finger_positions
    from helpers.position_action_client import position_client, move_to_position
    from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
    from helpers.covariance import generate_cartesian_covariance
except ImportError as e:
    rospy.logerr("导入助手模块失败: %s", e)
    rospy.logerr("请确保helpers模块在PYTHONPATH中")
    sys.exit(1)

# 全局变量
MOVING = False  # 机械臂是否在速度控制下运动
CURR_Z = 0.0   # 当前末端执行器Z高度
CURRENT_POSE = None  # 存储当前位姿
LATEST_GGCNN_CMD = None # 最新GGCNN命令
LATEST_GGCNN_TIME = None # 最新命令接收时间

# 相机内参全局变量
CAMERA_INTRINSICS = {'fx': None, 'fy': None, 'cx': None, 'cy': None}

# 参数（坐标系和话题）
CAMERA_FRAME = rospy.get_param('~camera_frame', 'camera_depth_optical_frame')
BASE_FRAME = rospy.get_param('~base_frame', 'm1n6s200_link_base')
EE_FRAME = rospy.get_param('~ee_frame', 'm1n6s200_end_effector')
WRENCH_TOPIC = rospy.get_param('~wrench_topic', '/m1n6s200_driver/out/tool_wrench')
POSE_TOPIC = rospy.get_param('~pose_topic', '/m1n6s200_driver/out/tool_pose')
CMD_TOPIC = rospy.get_param('~cmd_topic', '/ggcnn/out/command')
CAMERA_INFO_TOPIC = rospy.get_param('~camera_info_topic', '/camera/depth/camera_info')
CART_VELO_TOPIC = rospy.get_param('~cartesian_velocity_topic', '/m1n6s200_driver/in/cartesian_velocity')
START_FORCE_SRV = rospy.get_param('~start_force_srv', '/m1n6s200_driver/in/start_force_control')
STOP_FORCE_SRV = rospy.get_param('~stop_force_srv', '/m1n6s200_driver/in/stop_force_control')

# TF相关变量
tf_buffer = None
tf_listener = None

# 固定的Home位置（根据您的设置调整这些值）
FIXED_HOME_POSITION = [0.0, -0.38, 0.35]  # x, y, z
FIXED_HOME_ORIENTATION = [0.99, 0.0, 0.0, np.sqrt(1 - 0.99**2)]  # x, y, z, w

def camera_info_callback(msg):
    """获取相机内参的回调函数"""
    global CAMERA_INTRINSICS
    if CAMERA_INTRINSICS['fx'] is None:
        CAMERA_INTRINSICS['fx'] = msg.K[0]
        CAMERA_INTRINSICS['cx'] = msg.K[2]
        CAMERA_INTRINSICS['fy'] = msg.K[4]
        CAMERA_INTRINSICS['cy'] = msg.K[5]
        rospy.loginfo(f"接收到相机内参: fx={CAMERA_INTRINSICS['fx']}, fy={CAMERA_INTRINSICS['fy']}, cx={CAMERA_INTRINSICS['cx']}, cy={CAMERA_INTRINSICS['cy']}")

def ggcnn_command_callback(msg):
    """GGCNN命令回调函数"""
    global LATEST_GGCNN_CMD, LATEST_GGCNN_TIME
    LATEST_GGCNN_CMD = list(msg.data)
    LATEST_GGCNN_TIME = rospy.Time.now()

def check_tf_connection():
    """检查TF连接是否正常"""
    global tf_buffer, tf_listener
    
    if tf_buffer is None:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    try:
        # 等待TF变换可用，最多等待5秒
        transform = tf_buffer.lookup_transform(BASE_FRAME, CAMERA_FRAME, rospy.Time(0), rospy.Duration(5.0))
        rospy.loginfo("TF连接正常: %s -> %s", BASE_FRAME, CAMERA_FRAME)
        rospy.loginfo("变换参数: 位置(%.6f, %.6f, %.6f)", 
                     transform.transform.translation.x,
                     transform.transform.translation.y, 
                     transform.transform.translation.z)
        return True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF连接失败: %s", str(e))
        rospy.logerr("请确保已发布从 %s 到 %s 的静态变换", BASE_FRAME, CAMERA_FRAME)
        return False

def wait_for_service(service_name, timeout=10.0):
    """等待服务可用"""
    try:
        rospy.loginfo("等待服务: %s", service_name)
        rospy.wait_for_service(service_name, timeout=timeout)
        rospy.loginfo("服务已就绪: %s", service_name)
        return True
    except rospy.ROSException:
        rospy.logerr("服务等待超时: %s", service_name)
        return False

def robot_wrench_callback(msg):
    """监控力矩以在碰撞时停止运动"""
    global MOVING
    if MOVING and msg.wrench.force.z < -2.0:
        MOVING = False
        rospy.logerr('检测到力过大，停止运动')

def robot_position_callback(msg):
    """监控机器人位置"""
    global CURR_Z, CURRENT_POSE
    CURR_Z = msg.pose.position.z
    CURRENT_POSE = msg.pose  # 保存当前位置

def validate_grasp_position(grasp_pose):
    """验证抓取位置是否在合理范围内"""
    # 检查Z坐标是否合理（不能低于桌面，不能过高）
    if grasp_pose.position.z < 0.05 or grasp_pose.position.z > 0.8:
        rospy.logwarn("抓取Z坐标不合理: %.3f米", grasp_pose.position.z)
        return False
    
    # 检查X,Y坐标是否在工作空间内
    if abs(grasp_pose.position.x) > 0.6 or abs(grasp_pose.position.y) > 0.6:
        rospy.logwarn("抓取位置超出工作空间: (%.3f, %.3f)", 
                     grasp_pose.position.x, grasp_pose.position.y)
        return False
    
    rospy.loginfo("抓取位置验证通过: (%.3f, %.3f, %.3f)", 
                 grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z)
    return True

def move_to_pose_with_retry(pose, max_attempts=3):
    """带重试机制的移动函数"""
    for attempt in range(max_attempts):
        rospy.loginfo("移动尝试 %d/%d", attempt + 1, max_attempts)
        
        try:
            p = pose.position
            o = pose.orientation
            rospy.loginfo("移动到位置: (%.3f, %.3f, %.3f)", p.x, p.y, p.z)
            
            # 调用移动函数（不使用timeout参数）
            success = move_to_position([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
            
            if success:
                rospy.loginfo("移动成功")
                return True
            else:
                rospy.logwarn("移动失败，尝试重试...")
                rospy.sleep(1.0)  # 短暂延迟后重试
                
        except Exception as e:
            rospy.logerr("移动过程中发生错误: %s", str(e))
            rospy.sleep(1.0)
    
    rospy.logerr("所有移动尝试均失败")
    return False

def move_to_fixed_home():
    """移动到固定的Home位置"""
    home_pose = geometry_msgs.msg.Pose()
    home_pose.position.x = FIXED_HOME_POSITION[0]
    home_pose.position.y = FIXED_HOME_POSITION[1]
    home_pose.position.z = FIXED_HOME_POSITION[2]
    home_pose.orientation.x = FIXED_HOME_ORIENTATION[0]
    home_pose.orientation.y = FIXED_HOME_ORIENTATION[1]
    home_pose.orientation.z = FIXED_HOME_ORIENTATION[2]
    home_pose.orientation.w = FIXED_HOME_ORIENTATION[3]
    
    rospy.loginfo("移动到固定Home位置: (%.3f, %.3f, %.3f)", 
                 home_pose.position.x, home_pose.position.y, home_pose.position.z)
    return move_to_pose_with_retry(home_pose)

def calculate_gripper_position(grip_width, curr_z):
    """计算夹爪位置"""
    try:
        # 转换宽度计算（更精确的公式）
        # 假设视角约为60度，像素到角度的转换
        fov_degrees = 60.0  # 相机视野角度
        image_width_pixels = 300.0  # GGCNN输出图像宽度
        
        # 计算每个像素对应的角度
        degrees_per_pixel = fov_degrees / image_width_pixels
        
        # 计算抓取角度（弧度）
        grip_angle_rad = grip_width * degrees_per_pixel * np.pi / 180.0
        
        # 计算抓取宽度（米）
        # 假设抓取点距离相机约 curr_z + 0.07 米
        grasp_distance = curr_z + 0.07
        physical_width = 2 * grasp_distance * np.tan(grip_angle_rad / 2.0)
        
        # 转换为夹爪电机位置（0-6800，其中0=完全打开，6800=完全关闭）
        # 假设最大抓取宽度为0.07米（70mm）
        max_width = 0.07
        width_ratio = min(physical_width, max_width) / max_width
        
        # 计算电机位置（4000-6800范围）
        motor_position = 4000 + (1 - width_ratio) * 2800
        motor_position = min(max(motor_position, 4000), 6800)
        
        rospy.loginfo("夹爪计算: 宽度=%.1fpx, 物理宽度=%.3fm, 电机位置=%.0f", 
                     grip_width, physical_width, motor_position)
        return motor_position
    except Exception as e:
        rospy.logerr("计算夹爪位置失败: %s", str(e))
        return 4000  # 默认打开位置

def execute_grasp():
    """执行抓取操作"""
    global MOVING, CURR_Z, CAMERA_INTRINSICS
    
    # 0. 检查相机内参
    if CAMERA_INTRINSICS['fx'] is None:
        rospy.logerr("尚未获取相机内参，无法执行抓取")
        return False
        
    # 1. 检查TF连接
    if not check_tf_connection():
        rospy.logerr("TF连接检查失败，无法执行抓取")
        return False
    
    # 2. 检查服务可用性
    if not (wait_for_service(START_FORCE_SRV) and wait_for_service(STOP_FORCE_SRV)):
        rospy.logerr("必要的服务不可用，无法执行抓取")
        return False
    
    # 3. 获取抓取命令
    try:
        rospy.loginfo("等待新鲜的GGCNN抓取命令...")
        
        # 等待最多5秒以获取新鲜数据（2秒内的数据视为有效）
        wait_start = rospy.Time.now()
        got_fresh_data = False
        
        while (rospy.Time.now() - wait_start).to_sec() < 5.0:
            if LATEST_GGCNN_TIME is not None:
                age = (rospy.Time.now() - LATEST_GGCNN_TIME).to_sec()
                if age < 2.0:
                    got_fresh_data = True
                    break
            rospy.sleep(0.1)
            
        if not got_fresh_data:
            if LATEST_GGCNN_TIME is None:
                rospy.logerr("从未接收到GGCNN命令")
            else:
                rospy.logerr("GGCNN命令数据陈旧 (延迟: %.2fs)，相机可能卡住或检测失败", 
                             (rospy.Time.now() - LATEST_GGCNN_TIME).to_sec())
            return False

        d = LATEST_GGCNN_CMD
        # 数据格式变换: [u, v, z_depth, ang, width, depth_center]
        # 执行坐标转换：像素坐标 -> 相机坐标
        # d[0]=u, d[1]=v, d[2]=z
        u = d[0]
        v = d[1]
        z_depth = d[2]
        
        fx = CAMERA_INTRINSICS['fx']
        fy = CAMERA_INTRINSICS['fy']
        cx = CAMERA_INTRINSICS['cx']
        cy = CAMERA_INTRINSICS['cy']
        
        # 坐标变换公式
        x = (u - cx) / fx * z_depth
        y = (v - cy) / fy * z_depth
        z = z_depth
        
        # 更新数据用于后续处理
        # 原始数据包含 [u, v, z, ang, width, depth_center]
        # 我们现在有了 [x, y, z]，ang(d[3]), width(d[4])
        
        rospy.loginfo("坐标转换: Pixel(u=%.1f, v=%.1f) -> Camera(x=%.3f, y=%.3f, z=%.3f) using Intrin(fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f)",
                      u, v, x, y, z, fx, fy, cx, cy)
        
        rospy.loginfo("接收到抓取命令 (延迟: %.3fs): [x=%.3f, y=%.3f, z=%.3f, ang=%.3f, width=%.3f]", 
                     (rospy.Time.now() - LATEST_GGCNN_TIME).to_sec(),
                     x, y, z, d[3], d[4])
                     
        # 调试：检查数据是否变化
        rospy.loginfo("RAW GGCNN DATA: %s", str(d))
        
    except Exception as e:
        rospy.logerr("获取GGCNN命令异常: %s", str(e))
        return False
    
    # 4. 设置夹爪到预抓取宽度
    try:
        grip_width = d[4]
        motor_pos = calculate_gripper_position(grip_width, CURR_Z)
        rospy.loginfo("设置夹爪到预抓取位置: %.0f", motor_pos)
        set_finger_positions([motor_pos, motor_pos])
        rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr("设置夹爪失败: %s", str(e))
        return False
    
    # 5. 计算抓取位姿
    try:
        # 相机坐标系中的抓取位姿
        gp_camera = geometry_msgs.msg.Pose()
        gp_camera.position.x = x
        gp_camera.position.y = y
        gp_camera.position.z = z
        gp_camera.orientation.w = 1.0

        # 转换到基坐标系
        try:
            gp_base = convert_pose(gp_camera, CAMERA_FRAME, BASE_FRAME)
            if gp_base is None:
                rospy.logerr("坐标变换失败")
                return False
            
            rospy.loginfo("基坐标系下的抓取位姿: x=%.3f, y=%.3f, z=%.3f", 
                         gp_base.position.x, gp_base.position.y, gp_base.position.z)
        except Exception as e:
            rospy.logerr("坐标变换异常: %s", str(e))
            return False

        # 设置抓取方向
        q = tft.quaternion_from_euler(np.pi, 0, d[3])  # 翻转并添加抓取角度
        gp_base.orientation.x = q[0]
        gp_base.orientation.y = q[1]
        gp_base.orientation.z = q[2]
        gp_base.orientation.w = q[3]

        # 验证抓取位置
        if not validate_grasp_position(gp_base):
            rospy.logerr("抓取位置验证失败")
            return False

        publish_pose_as_transform(gp_base, BASE_FRAME, 'grasp_point', 0.5)
        
    except Exception as e:
        rospy.logerr("计算抓取位姿失败: %s", str(e))
        return False
    
    # 6. 执行抓取运动
    try:
        # 初始偏移（安全高度）
        approach_offset = 0.15
        approach_pose = geometry_msgs.msg.Pose()
        approach_pose.position.x = gp_base.position.x
        approach_pose.position.y = gp_base.position.y
        approach_pose.position.z = gp_base.position.z + approach_offset
        approach_pose.orientation = gp_base.orientation

        # 停止力控制以提高精度
        stop_force_srv = rospy.ServiceProxy(STOP_FORCE_SRV, kinova_msgs.srv.Stop)
        if wait_for_service(STOP_FORCE_SRV):
            stop_force_srv.call(kinova_msgs.srv.StopRequest())
            rospy.loginfo("已停止力控制")

        # 移动到接近位置
        rospy.loginfo("移动到接近位置")
        if not move_to_pose_with_retry(approach_pose):
            rospy.logerr("移动到接近位置失败")
            return False
        
        rospy.sleep(0.5)

        # 开始力控制以帮助防止碰撞
        start_force_srv = rospy.ServiceProxy(START_FORCE_SRV, kinova_msgs.srv.Start)
        if wait_for_service(START_FORCE_SRV):
            start_force_srv.call(kinova_msgs.srv.StartRequest())
            rospy.loginfo("已启动力控制")

        rospy.sleep(0.25)

        # 设置运动标志
        MOVING = True

        # 生成非线性控制
        cart_cov = generate_cartesian_covariance(0)

        # 速度控制下降
        velo_pub = rospy.Publisher(CART_VELO_TOPIC, kinova_msgs.msg.PoseVelocity, queue_size=1)
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(10.0)  # 10秒超时
        
        rospy.loginfo("开始下降抓取")
        while (MOVING and 
               CURR_Z - 0.02 > gp_base.position.z and 
               (rospy.Time.now() - start_time) < timeout_duration):
            
            # 计算下降速度
            dz = gp_base.position.z - CURR_Z - 0.03  # 考虑夹爪长度
            MAX_VELO_Z = 0.05  # 降低最大速度以提高安全性
            dz = max(min(dz, MAX_VELO_Z), -1.0 * MAX_VELO_Z)

            # 应用非线性控制
            v = np.array([0, 0, dz])
            vc = list(np.dot(v, cart_cov)) + [0, 0, 0]
            
            # 发布速度命令
            velo_msg = kinova_msgs.msg.PoseVelocity()
            velo_msg.twist_linear_x = vc[0]
            velo_msg.twist_linear_y = vc[1]
            velo_msg.twist_linear_z = vc[2]
            velo_msg.twist_angular_x = vc[3]
            velo_msg.twist_angular_y = vc[4]
            velo_msg.twist_angular_z = vc[5]
            
            velo_pub.publish(velo_msg)
            rospy.sleep(0.01)  # 10ms控制周期

        MOVING = False
        
        # 检查是否超时
        if (rospy.Time.now() - start_time) >= timeout_duration:
            rospy.logwarn("下降运动超时")
        
    except Exception as e:
        rospy.logerr("运动控制失败: %s", str(e))
        MOVING = False
        return False
    
    # 7. 闭合夹爪
    try:
        rospy.sleep(0.1)
        rospy.loginfo("闭合夹爪")
        set_finger_positions([6800, 6800])  # 完全闭合
        rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr("闭合夹爪失败: %s", str(e))
        return False
    
    # 8. 抬升到安全高度
    try:
        # 停止力控制，必须在位置控制前执行！
        if wait_for_service(STOP_FORCE_SRV):
            stop_force_srv.call(kinova_msgs.srv.StopRequest())
            rospy.loginfo("已停止力控制 (准备抬升)")

        # 创建抬升位姿（保持当前位置，只增加Z高度）
        lift_pose = geometry_msgs.msg.Pose()
        lift_pose.position.x = gp_base.position.x
        lift_pose.position.y = gp_base.position.y
        lift_pose.position.z = gp_base.position.z + approach_offset
        lift_pose.orientation = gp_base.orientation
        
        rospy.loginfo("抬升机械臂到安全高度")
        if not move_to_pose_with_retry(lift_pose):
            rospy.logwarn("抬升运动失败")
            
    except Exception as e:
        rospy.logerr("抬升运动失败: %s", str(e))
        return False
    
    rospy.loginfo("抓取操作完成")
    return True

def main():
    """主函数"""
    rospy.init_node('kinova_open_loop_grasp')
    
    # 初始化TF
    global tf_buffer, tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 机器人状态监控
    wrench_sub = rospy.Subscriber(WRENCH_TOPIC, geometry_msgs.msg.WrenchStamped, robot_wrench_callback, queue_size=1)
    position_sub = rospy.Subscriber(POSE_TOPIC, geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)
    cmd_sub = rospy.Subscriber(CMD_TOPIC, std_msgs.msg.Float32MultiArray, ggcnn_command_callback, queue_size=1)
    
    # 添加相机内参订阅
    camera_info_sub = rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, camera_info_callback, queue_size=1)
    
    rospy.loginfo("GGCNN开环抓取节点已启动")
    rospy.loginfo("基坐标系: %s", BASE_FRAME)
    rospy.loginfo("相机坐标系: %s", CAMERA_FRAME)
    rospy.loginfo("末端执行器坐标系: %s", EE_FRAME)
    
    # 等待机械臂状态topic
    try:
        rospy.loginfo("等待机械臂状态topic...")
        rospy.wait_for_message(POSE_TOPIC, geometry_msgs.msg.PoseStamped, timeout=10.0)
        rospy.loginfo("机械臂状态topic已就绪: %s", POSE_TOPIC)
        
        # 等待几秒确保当前位置被正确记录
        rospy.sleep(2.0)
    except rospy.ROSException:
        rospy.logerr("等待机械臂状态topic超时: %s", POSE_TOPIC)
        return
    
    # 主循环
    while not rospy.is_shutdown():
        try:
            # 打开夹爪
            try:
                rospy.loginfo("打开夹爪")
                set_finger_positions([4000, 4000])  # 打开夹爪
                rospy.sleep(0.5)
            except Exception as e:
                rospy.logwarn("设置夹爪位置失败: %s，继续执行", str(e))
            
            # 移动到固定Home位置
            rospy.loginfo("移动到固定Home位置")
            if not move_to_fixed_home():
                rospy.logerr("移动到Home位置失败，跳过本次抓取")
                continue
            
            # 显示当前位置
            rospy.loginfo("当前位置: Z=%.3f", CURR_Z)
            
            # 等待用户输入
            try:
                input('按Enter开始抓取，或Ctrl+C退出...')
            except EOFError:
                # 在非交互式环境中继续
                rospy.loginfo("非交互式模式，5秒后开始抓取...")
                rospy.sleep(5)
            
            # 执行抓取
            grasp_success = execute_grasp()
            
            if grasp_success:
                rospy.loginfo("抓取成功!")
            else:
                rospy.logerr("抓取失败!")
            
            # 返回Home位置
            try:
                rospy.loginfo("返回Home位置")
                move_to_fixed_home()
                rospy.sleep(0.5)
            except Exception as e:
                rospy.logwarn("返回Home位置失败: %s", str(e))
            
            # 等待用户确认
            try:
                input('按Enter继续下一轮抓取，或Ctrl+C退出...')
            except EOFError:
                rospy.loginfo("非交互式模式，5秒后继续...")
                rospy.sleep(5)
                
        except rospy.ROSInterruptException:
            rospy.loginfo("接收到中断信号，退出...")
            break
        except KeyboardInterrupt:
            rospy.loginfo("用户中断，退出...")
            break
        except Exception as e:
            rospy.logerr("主循环中发生未预期错误: %s", str(e))
            rospy.sleep(1)  # 防止快速循环

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("程序执行失败: %s", str(e))