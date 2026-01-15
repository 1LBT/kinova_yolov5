#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TF变换链调试工具

用于验证和调试抓取程序中的坐标变换。
可以独立运行来检查TF树的完整性和变换正确性。

使用方法:
    rosrun ggcnn_kinova_grasping debug_tf_chain.py

功能:
    1. 检查TF树完整性
    2. 打印所有相关坐标系之间的变换
    3. 测试坐标变换计算
    4. 可视化变换结果
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft


class TFDebugger:
    def __init__(self):
        rospy.init_node('tf_debugger', anonymous=True)
        
        # 坐标系名称
        self.base_frame = rospy.get_param('~base_frame', 'm1n6s200_link_base')
        self.ee_frame = rospy.get_param('~ee_frame', 'm1n6s200_end_effector')
        self.camera_link = rospy.get_param('~camera_link', 'camera_link')
        self.camera_optical = rospy.get_param('~camera_frame', 'camera_depth_optical_frame')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 等待TF缓冲区填充
        rospy.loginfo("等待TF缓冲区初始化...")
        rospy.sleep(2.0)
        
    def print_separator(self, title=""):
        print("\n" + "=" * 60)
        if title:
            print(title)
            print("=" * 60)
    
    def get_transform(self, from_frame, to_frame, timeout=3.0):
        """获取两个坐标系之间的变换"""
        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame, from_frame, 
                rospy.Time(0), 
                rospy.Duration(timeout)
            )
            return transform
        except Exception as e:
            rospy.logwarn("无法获取变换 %s -> %s: %s", from_frame, to_frame, str(e))
            return None
    
    def print_transform(self, transform, desc=""):
        """打印变换信息"""
        if transform is None:
            print("  [FAIL] 变换不可用")
            return
        
        t = transform.transform.translation
        r = transform.transform.rotation
        
        # 转换为欧拉角
        euler = tft.euler_from_quaternion([r.x, r.y, r.z, r.w])
        
        print("  位置: x=%.4f, y=%.4f, z=%.4f (米)" % (t.x, t.y, t.z))
        print("  四元数: x=%.4f, y=%.4f, z=%.4f, w=%.4f" % (r.x, r.y, r.z, r.w))
        print("  欧拉角: roll=%.2f, pitch=%.2f, yaw=%.2f (度)" % 
              (np.degrees(euler[0]), np.degrees(euler[1]), np.degrees(euler[2])))
    
    def check_tf_chain(self):
        """检查完整的TF变换链"""
        self.print_separator("TF变换链检查")
        
        # 定义需要检查的变换对
        transform_pairs = [
            (self.base_frame, self.ee_frame, "基座 -> 末端执行器"),
            (self.ee_frame, self.camera_link, "末端执行器 -> 相机链接"),
            (self.camera_link, self.camera_optical, "相机链接 -> 光学帧"),
            (self.base_frame, self.camera_optical, "基座 -> 光学帧 (完整链)"),
        ]
        
        all_ok = True
        for from_frame, to_frame, desc in transform_pairs:
            print("\n[%s]" % desc)
            print("  %s -> %s" % (from_frame, to_frame))
            
            transform = self.get_transform(from_frame, to_frame)
            if transform:
                self.print_transform(transform)
            else:
                all_ok = False
        
        return all_ok
    
    def test_point_transform(self):
        """测试点坐标变换"""
        self.print_separator("坐标变换测试")
        
        # 创建测试点（相机光学坐标系下）
        # 光学坐标系: x右, y下, z前
        test_points = [
            (0.0, 0.0, 0.5, "相机正前方0.5米 (应该在基座下方)"),
            (0.1, 0.0, 0.5, "相机右侧0.1米，前方0.5米"),
            (0.0, 0.1, 0.5, "相机下方0.1米，前方0.5米"),
            (0.0789, -0.1196, 0.42, "GGCNN实际输出点"),
        ]
        
        for x, y, z, desc in test_points:
            print("\n测试点: %s" % desc)
            print("  相机光学坐标系 (x右,y下,z前): (%.4f, %.4f, %.4f)" % (x, y, z))
            
            # 创建位姿
            pose = geometry_msgs.msg.Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            
            # 变换到基座坐标系
            pose_stamped = geometry_msgs.msg.PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = self.camera_optical
            pose_stamped.header.stamp = rospy.Time(0)
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, 
                    self.camera_optical, 
                    rospy.Time(0), 
                    rospy.Duration(3.0)
                )
                
                transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
                p = transformed.pose.position
                print("  基座坐标系: (%.4f, %.4f, %.4f)" % (p.x, p.y, p.z))
                
                # 验证Z坐标是否合理
                if p.z < 0.05:
                    print("  [警告] Z坐标过低 (%.3f < 0.05)，可能低于桌面!" % p.z)
                elif p.z > 0.5:
                    print("  [警告] Z坐标过高 (%.3f > 0.5)，可能超出工作空间!" % p.z)
                else:
                    print("  [OK] Z坐标合理 (%.3f)" % p.z)
                
            except Exception as e:
                print("  [FAIL] 变换失败: %s" % str(e))
             
    
    def print_available_frames(self):
        """打印所有可用的坐标系"""
        self.print_separator("可用坐标系")
        
        try:
            frames = self.tf_buffer.all_frames_as_yaml()
            print(frames)
        except Exception as e:
            print("无法获取坐标系列表: %s" % str(e))
    
    def run(self):
        """运行所有调试检查"""
        self.print_separator("TF调试工具")
        print("基座坐标系: %s" % self.base_frame)
        print("末端执行器坐标系: %s" % self.ee_frame)
        print("相机链接坐标系: %s" % self.camera_link)
        print("相机光学坐标系: %s" % self.camera_optical)
        
        # 检查TF链
        tf_ok = self.check_tf_chain()
        
        if tf_ok:
            # 测试坐标变换
            self.test_point_transform()
        else:
            print("\n[警告] TF链不完整，跳过坐标变换测试")
            self.print_available_frames()
        
        self.print_separator("调试完成")
        
        if tf_ok:
            print("TF变换链正常，可以进行抓取操作")
        else:
            print("TF变换链存在问题，请检查:")
            print("  1. URDF是否正确加载")
            print("  2. robot_state_publisher是否运行")
            print("  3. 相机TF是否发布")


if __name__ == '__main__':
    try:
        debugger = TFDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass
