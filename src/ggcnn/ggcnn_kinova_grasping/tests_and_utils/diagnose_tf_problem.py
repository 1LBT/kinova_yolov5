#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TF问题诊断脚本

检查是否有多个TF发布器冲突，以及手眼标定是否正确应用
"""

import rospy
import tf2_ros
import numpy as np
import tf.transformations as tft


def diagnose_tf():
    rospy.init_node('diagnose_tf', anonymous=True)
    
    print("=" * 70)
    print("TF问题诊断工具")
    print("=" * 70)
    
    # 等待TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2.0)
    
    # 期望的手眼标定值
    expected = {
        'x': -0.006886560877924761,
        'y': 0.06518314780922749,
        'z': -0.08583169582135934,
        'qx': -0.01551792345147061,
        'qy': -0.02102504775817704,
        'qz': -0.9933831640608743,
        'qw': 0.11183483705414576,
    }
    
    print("\n1. 期望的手眼标定值:")
    print("   位置: x=%.6f, y=%.6f, z=%.6f" % (expected['x'], expected['y'], expected['z']))
    print("   四元数: x=%.6f, y=%.6f, z=%.6f, w=%.6f" % 
          (expected['qx'], expected['qy'], expected['qz'], expected['qw']))
    
    # 获取实际TF
    try:
        transform = tf_buffer.lookup_transform(
            'camera_link',
            'm1n6s200_end_effector',
            rospy.Time(0),
            rospy.Duration(3.0)
        )
        
        t = transform.transform.translation
        r = transform.transform.rotation
        
        print("\n2. 实际TF值 (末端执行器 -> 相机链接):")
        print("   位置: x=%.6f, y=%.6f, z=%.6f" % (t.x, t.y, t.z))
        print("   四元数: x=%.6f, y=%.6f, z=%.6f, w=%.6f" % (r.x, r.y, r.z, r.w))
        
        # 计算误差
        pos_error = np.sqrt((t.x - expected['x'])**2 + 
                           (t.y - expected['y'])**2 + 
                           (t.z - expected['z'])**2)
        
        quat_error = np.sqrt((r.x - expected['qx'])**2 + 
                            (r.y - expected['qy'])**2 + 
                            (r.z - expected['qz'])**2 + 
                            (r.w - expected['qw'])**2)
        
        print("\n3. 误差分析:")
        print("   位置误差: %.6f 米" % pos_error)
        print("   四元数误差: %.6f" % quat_error)
        
        if pos_error > 0.001:
            print("   [警告] 位置误差过大!")
            print("   差异: dx=%.6f, dy=%.6f, dz=%.6f" % 
                  (t.x - expected['x'], t.y - expected['y'], t.z - expected['z']))
        
        if quat_error > 0.01:
            print("   [警告] 四元数误差过大!")
            print("   差异: dqx=%.6f, dqy=%.6f, dqz=%.6f, dqw=%.6f" % 
                  (r.x - expected['qx'], r.y - expected['qy'], 
                   r.z - expected['qz'], r.w - expected['qw']))
            
            # 检查是否是符号反转
            if abs(r.w + expected['qw']) < 0.01:
                print("   [错误] 四元数w的符号反了!")
                print("   这会导致旋转方向完全相反!")
        
        # 检查相机在基座坐标系中的位置
        print("\n4. 相机在基座坐标系中的位置:")
        transform_base = tf_buffer.lookup_transform(
            'camera_depth_optical_frame',
            'm1n6s200_link_base',
            rospy.Time(0),
            rospy.Duration(3.0)
        )
        
        tb = transform_base.transform.translation
        print("   位置: x=%.4f, y=%.4f, z=%.4f" % (tb.x, tb.y, tb.z))
        
        if tb.z < 0:
            print("   [错误] 相机Z坐标为负，相机在基座下方!")
            print("   这明显不合理，说明TF有严重错误!")
        elif tb.z < 0.1:
            print("   [警告] 相机Z坐标过低 (%.3f < 0.1)" % tb.z)
        else:
            print("   [OK] 相机Z坐标合理")
        
        # 测试坐标变换
        print("\n5. 坐标变换测试:")
        print("   测试点: 相机前方0.5米 -> 基座坐标系")
        
        # 这个测试应该给出合理的结果
        # 如果相机朝下看，相机前方0.5米应该在基座下方约0.5米处
        
    except Exception as e:
        print("\n[错误] 无法获取TF: %s" % str(e))
        return False
    
    print("\n" + "=" * 70)
    print("诊断建议:")
    print("=" * 70)
    
    if pos_error > 0.001 or quat_error > 0.01:
        print("TF配置有问题，可能的原因:")
        print("1. 有多个TF发布器在发布相同的变换")
        print("2. 手眼标定参数没有正确应用")
        print("3. 使用了错误的坐标系名称")
        print("\n检查方法:")
        print("  rosnode list | grep static")
        print("  rostopic echo /tf_static")
        print("  rosrun tf view_frames")
    else:
        print("TF配置正确!")
    
    print("=" * 70)


if __name__ == '__main__':
    try:
        diagnose_tf()
    except rospy.ROSInterruptException:
        pass
