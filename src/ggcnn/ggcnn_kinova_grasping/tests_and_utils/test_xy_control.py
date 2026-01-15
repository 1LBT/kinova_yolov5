#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试XY位置控制功能

这个脚本用于测试修改后的机械臂XY位置控制功能，
可以手动指定测试点，验证机械臂能否准确移动到指定位置。
"""

import rospy
import geometry_msgs.msg
import numpy as np
import sys

# 导入修改后的抓取程序中的函数
import sys
import os

# 获取当前脚本的目录
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    from kinova_open_loop_grasp import (
        init_parameters, check_tf_connection, move_to_target_xy, 
        move_to_fixed_home, debug_tf_tree
    )
    from helpers.position_action_client import move_to_position
except ImportError as e:
    rospy.logerr("导入失败: %s", str(e))
    rospy.logerr("请确保在正确的工作空间中运行此脚本")
    sys.exit(1)

def test_manual_points():
    """测试手动指定的测试点"""
    
    # 定义测试点（基座坐标系）
    test_points = [
        (0.2, -0.3, "前方左侧"),
        (0.3, 0.0, "正前方"),
        (0.2, 0.3, "前方右侧"),
        (0.4, -0.2, "远前方左侧"),
        (0.1, -0.4, "近左侧"),
    ]
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("开始手动测试点验证")
    rospy.loginfo("=" * 60)
    
    for i, (x, y, desc) in enumerate(test_points):
        rospy.loginfo("测试点 %d/%d: %s (%.3f, %.3f)", i+1, len(test_points), desc, x, y)
        
        try:
            input(f'按Enter测试点 {i+1}: {desc} ({x}, {y})，或Ctrl+C跳过...')
        except EOFError:
            rospy.loginfo("非交互式模式，自动测试...")
        except KeyboardInterrupt:
            rospy.loginfo("跳过测试点 %d", i+1)
            continue
        
        # 执行XY移动测试
        success = move_to_target_xy(x, y, tolerance=0.02)
        
        if success:
            rospy.loginfo("✓ 测试点 %d 成功", i+1)
        else:
            rospy.logerr("✗ 测试点 %d 失败", i+1)
        
        rospy.sleep(1.0)
        
        # 返回Home位置
        rospy.loginfo("返回Home位置...")
        move_to_fixed_home()
        rospy.sleep(1.0)

def test_coordinate_transform():
    """测试坐标变换功能"""
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("测试坐标变换功能")
    rospy.loginfo("=" * 60)
    
    # 模拟GGCNN输出的相机坐标系位置
    camera_positions = [
        (0.0, 0.0, 0.4, "相机正前方40cm"),
        (0.1, 0.0, 0.3, "相机右侧10cm，前方30cm"),
        (-0.1, 0.0, 0.5, "相机左侧10cm，前方50cm"),
        (0.0, 0.1, 0.35, "相机下方10cm，前方35cm"),
    ]
    
    for i, (cx, cy, cz, desc) in enumerate(camera_positions):
        rospy.loginfo("变换测试 %d/%d: %s", i+1, len(camera_positions), desc)
        rospy.loginfo("  相机坐标: (%.3f, %.3f, %.3f)", cx, cy, cz)
        
        # 创建相机坐标系位姿
        pose_camera = geometry_msgs.msg.Pose()
        pose_camera.position.x = cx
        pose_camera.position.y = cy
        pose_camera.position.z = cz
        pose_camera.orientation.w = 1.0
        
        # 导入变换函数
        from kinova_open_loop_grasp import camera_to_base_transform
        
        # 执行坐标变换
        pose_base = camera_to_base_transform(pose_camera, grasp_angle=0.0)
        
        if pose_base:
            rospy.loginfo("  基座坐标: (%.3f, %.3f, %.3f)", 
                         pose_base.position.x, pose_base.position.y, pose_base.position.z)
            
            # 验证变换是否合理
            distance_camera = np.sqrt(cx**2 + cy**2 + cz**2)
            distance_base = np.sqrt(pose_base.position.x**2 + 
                                   pose_base.position.y**2 + 
                                   pose_base.position.z**2)
            rospy.loginfo("  距离验证: 相机%.3fm -> 基座%.3fm", distance_camera, distance_base)
        else:
            rospy.logerr("  坐标变换失败")
        
        rospy.loginfo("")

def main():
    """主函数"""
    rospy.init_node('test_xy_control')
    
    # 初始化参数
    init_parameters()
    
    # 初始化TF
    import tf2_ros
    global tf_buffer, tf_listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.loginfo("等待TF缓冲区初始化...")
    rospy.sleep(2.0)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("XY位置控制测试程序")
    rospy.loginfo("=" * 60)
    
    # 检查TF连接
    if not check_tf_connection():
        rospy.logerr("TF连接检查失败，退出测试")
        return
    
    # 显示TF调试信息
    debug_tf_tree()
    
    # 移动到Home位置
    rospy.loginfo("移动到Home位置...")
    if not move_to_fixed_home():
        rospy.logerr("移动到Home位置失败")
        return
    
    while not rospy.is_shutdown():
        try:
            print("\n选择测试项目:")
            print("1. 手动测试点验证")
            print("2. 坐标变换测试")
            print("3. TF调试信息")
            print("4. 退出")
            
            try:
                choice = input("请输入选择 (1-4): ").strip()
            except EOFError:
                choice = "4"  # 非交互式模式下退出
            
            if choice == "1":
                test_manual_points()
            elif choice == "2":
                test_coordinate_transform()
            elif choice == "3":
                debug_tf_tree()
            elif choice == "4":
                rospy.loginfo("退出测试程序")
                break
            else:
                rospy.logwarn("无效选择，请重新输入")
                
        except KeyboardInterrupt:
            rospy.loginfo("用户中断，退出测试")
            break
        except Exception as e:
            rospy.logerr("测试过程中发生错误: %s", str(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass