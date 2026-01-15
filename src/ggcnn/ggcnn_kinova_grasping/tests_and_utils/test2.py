#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import time

class GGCNNTestPublisher:
    def __init__(self):
        rospy.init_node('ggcnn_test_publisher')
        
        # 创建发布器
        self.depth_pub = rospy.Publisher('/camera/depth/image_meters', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=10, latch=True)
        self.tool_pose_pub = rospy.Publisher('/m1n6s200_driver/out/tool_pose', PoseStamped, queue_size=10)
        self.bridge = CvBridge()
        
        # 发布相机信息（一次性）
        self.publish_camera_info()
        
    def publish_camera_info(self):
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_depth_optical_frame"
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = []  # 无畸变
        
        # 相机内参矩阵 (3x3)
        camera_info.K = [381.0, 0.0, 320.0, 
                         0.0, 381.0, 240.0, 
                         0.0, 0.0, 1.0]
        
        # 投影矩阵 (3x4)
        camera_info.P = [381.0, 0.0, 320.0, 0.0,
                         0.0, 381.0, 240.0, 0.0,
                         0.0, 0.0, 1.0, 0.0]
        
        camera_info.header.stamp = rospy.Time.now()
        self.camera_info_pub.publish(camera_info)
        rospy.loginfo("已发布相机信息")
    
    def publish_tool_pose(self, z_position):
        """发布机械臂末端位姿"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = z_position
        pose.pose.orientation.w = 1.0
        self.tool_pose_pub.publish(pose)
    
    def create_test_depth_image(self, frame_count):
        """创建包含移动物体的测试深度图像"""
        depth_image = np.ones((480, 640), dtype=np.float32) * 0.5  # 0.5米深度
        
        # 创建移动的物体
        center_x = 320 + int(100 * np.sin(frame_count * 0.1))
        center_y = 240 + int(100 * np.cos(frame_count * 0.1))
        radius = 50
        
        # 创建圆形物体
        y, x = np.ogrid[:480, :640]
        mask = (x - center_x)**2 + (y - center_y)**2 <= radius**2
        depth_image[mask] = 0.3 + 0.1 * np.sin(frame_count * 0.2)  # 轻微深度变化
        
        return depth_image
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        frame_count = 0
        z_position = 0.35  # 初始高度
        
        while not rospy.is_shutdown():
            try:
                # 创建测试深度图像
                depth_image = self.create_test_depth_image(frame_count)
                
                # 发布深度图像
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                self.depth_pub.publish(depth_msg)
                
                # 发布机械臂位姿
                self.publish_tool_pose(z_position)
                
                # 模拟机械臂高度变化
                z_position = 0.35 + 0.1 * np.sin(frame_count * 0.05)
                
                frame_count += 1
            except Exception as e:
                rospy.logerr(f"发布数据失败: {e}")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.loginfo("启动 GGCNN 测试数据发布器...")
        publisher = GGCNNTestPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭")