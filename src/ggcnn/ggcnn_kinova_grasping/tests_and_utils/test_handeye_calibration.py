#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
手眼标定验证脚本

用于验证手眼标定结果是否正确应用。
测试从相机坐标系到基座坐标系的变换是否合理。

使用方法:
    # 先启动手眼标定TF
    roslaunch ggcnn_kinova_grasping handeye_static_tf.launch
    
    # 然后运行此脚本
    rosrun ggcnn_kinova_grasping test_handeye_calibration.py
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft


def print_separator(title=""):
    print("\n" + "=" * 70)
    if title:
        print(title)
        print("=" * 70)


def test_handeye_calibration():
    """测试手眼标定"""
    rospy.init_node('test_handeye_calibration', anonymous=True)
    
    # 坐标系
    base_frame = 'm1n6s200_link_base'
    ee_frame = 'm1n6s200_end_effector'
    camera_link = 'camera_link'
    camera_optical = 'camera_depth_optical_frame'
    
    # TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    print_separator("手眼标定验证工具")
    print("等待TF缓冲区初始化...")
    rospy.sleep(2.0)
    
    # 1. 验证TF链
    print_separator("1. TF变换链验证")
    
    transforms_to_check = [
        (ee_frame, camera_link, "末端执行器 -> 相机链接 (手眼标定)"),
        (camera_link, camera_optical, "相机链接 -> 光学帧 (ROS约定)"),
        (base_frame, camera_optical, "基座 -> 光学帧 (完整链)"),
    ]
    
    all_ok = True
    for from_frame, to_frame, desc in transforms_to_check:
        try:
            transform = tf_buffer.lookup_transform(
                to_frame, from_frame,
                rospy.Time(0),
                rospy.Duration(3.0)
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            euler = tft.euler_from_quaternion([r.x, r.y, r.z, r.w])
            
            print("\n[OK] %s" % desc)
            print("  %s -> %s" % (from_frame, to_frame))
            print("  位置: x=%.4f, y=%.4f, z=%.4f (米)" % (t.x, t.y, t.z))
            print("  四元数: x=%.4f, y=%.4f, z=%.4f, w=%.4f" % (r.x, r.y, r.z, r.w))
            print("  欧拉角: roll=%.2f°, pitch=%.2f°, yaw=%.2f°" % 
                  (np.degrees(euler[0]), np.degrees(euler[1]), np.degrees(euler[2])))
            
            # 验证手眼标定参数
            if from_frame == ee_frame and to_frame == camera_link:
                expected_t = (-0.006887, 0.065183, -0.085832)
                expected_r = (-0.015518, -0.021025, -0.993383, 0.111835)
                
                t_error = np.sqrt((t.x - expected_t[0])**2 + 
                                 (t.y - expected_t[1])**2 + 
                                 (t.z - expected_t[2])**2)
                
                if t_error < 0.001:  # 1mm误差
                    print("  [验证通过] 手眼标定参数正确")
                else:
                    print("  [警告] 手眼标定参数可能不正确，误差: %.4f米" % t_error)
                    print("  期望位置: x=%.4f, y=%.4f, z=%.4f" % expected_t)
                    all_ok = False
                    
        except Exception as e:
            print("\n[FAIL] %s" % desc)
            print("  错误: %s" % str(e))
            all_ok = False
    
    if not all_ok:
        print_separator("TF验证失败")
        print("请检查:")
        print("  1. 是否启动了 handeye_static_tf.launch")
        print("  2. 是否有其他TF发布器冲突")
        print("  3. 手眼标定参数是否正确")
        return False
    
    # 2. 测试坐标变换
    print_separator("2. 坐标变换测试")
    
    # 模拟GGCNN输出的抓取点
    test_cases = [
        {
            'name': '相机正前方0.5米',
            'camera_pos': (0.0, 0.0, 0.5),
            'expected_z_range': (0.15, 0.45),  # 期望在基座坐标系中的Z范围
        },
        {
            'name': '相机右侧0.1米，前方0.4米',
            'camera_pos': (0.1, 0.0, 0.4),
            'expected_z_range': (0.15, 0.45),
        },
        {
            'name': 'GGCNN实际输出示例',
            'camera_pos': (0.0789, -0.1196, 0.42),
            'expected_z_range': (0.10, 0.50),
        },
    ]
    
    for test in test_cases:
        print("\n测试: %s" % test['name'])
        x, y, z = test['camera_pos']
        print("  相机光学坐标系 (x右,y下,z前): (%.4f, %.4f, %.4f)" % (x, y, z))
        
        # 创建位姿
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = camera_optical
        pose_stamped.header.stamp = rospy.Time(0)
        
        try:
            transform = tf_buffer.lookup_transform(
                base_frame,
                camera_optical,
                rospy.Time(0),
                rospy.Duration(3.0)
            )
            
            transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            p = transformed.pose.position
            
            print("  基座坐标系: (%.4f, %.4f, %.4f)" % (p.x, p.y, p.z))
            
            # 验证Z坐标
            z_min, z_max = test['expected_z_range']
            if z_min <= p.z <= z_max:
                print("  [OK] Z坐标在合理范围内 (%.3f ∈ [%.3f, %.3f])" % (p.z, z_min, z_max))
            else:
                print("  [警告] Z坐标超出预期范围!")
                print("  期望: %.3f ∈ [%.3f, %.3f]" % (p.z, z_min, z_max))
                if p.z < 0.05:
                    print("  [错误] Z坐标过低，可能低于桌面!")
                    all_ok = False
                    
        except Exception as e:
            print("  [FAIL] 变换失败: %s" % str(e))
            all_ok = False
    
    # 3. 总结
    print_separator("验证结果")
    
    if all_ok:
        print("✓ 手眼标定验证通过!")
        print("\n可以开始使用抓取程序:")
        print("  rosrun ggcnn_kinova_grasping kinova_open_loop_grasp.py")
    else:
        print("✗ 手眼标定验证失败!")
        print("\n请检查:")
        print("  1. 确认已启动 handeye_static_tf.launch")
        print("  2. 检查TF树: rosrun tf view_frames")
        print("  3. 查看TF: rosrun tf tf_echo m1n6s200_link_base camera_depth_optical_frame")
        print("  4. 运行调试工具: rosrun ggcnn_kinova_grasping debug_tf_chain.py")
    
    print_separator()
    
    return all_ok


if __name__ == '__main__':
    try:
        success = test_handeye_calibration()
        if not success:
            exit(1)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("\n[错误] 测试过程中发生异常: %s" % str(e))
        import traceback
        traceback.print_exc()
        exit(1)
