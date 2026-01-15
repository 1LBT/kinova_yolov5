#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class CharucoDetector:
    def __init__(self):
        rospy.init_node('charuco_detector', anonymous=True)

        # === 获取参数 ===
        # 字典类型: 默认为 DICT_4X4_50
        dict_name = rospy.get_param('~dictionary_name', 'DICT_4X4_50')
        # 棋盘格横向方块数
        self.squares_x = rospy.get_param('~squares_x', 5)
        # 棋盘格纵向方块数
        self.squares_y = rospy.get_param('~squares_y', 7)
        # 方块边长 (米)
        self.square_length = rospy.get_param('~square_length', 0.04)
        # ArUco 码边长 (米)
        self.marker_length = rospy.get_param('~marker_length', 0.03)
        
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.board_frame = rospy.get_param('~board_frame', 'charuco_board_frame')
        
        # === 初始化 ArUco 字典 ===
        try:
            # 兼容旧版 OpenCV (3.x/4.x early) 和新版
            if hasattr(aruco, dict_name):
                self.aruco_dict = aruco.Dictionary_get(getattr(aruco, dict_name))
            else:
                # 尝试从预定义列表获取
                self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
        except AttributeError:
            rospy.logerr(f"Unknown dictionary name: {dict_name}")
            return

        self.board = aruco.CharucoBoard_create(
            self.squares_x, self.squares_y, 
            self.square_length, self.marker_length, 
            self.aruco_dict
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.camera_matrix = None
        self.dist_coeffs = None

        # 订阅话题
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/charuco_detector/result', Image, queue_size=1)
        
        rospy.loginfo(f"Started Charuco Detector. Board: {self.squares_x}x{self.squares_y}, Sq: {self.square_length}m")

    def cam_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        self.camera_frame = msg.header.frame_id # 自动更新 frame_id

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 1. 检测 ArUco 码
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)
        
        if ids is not None and len(ids) > 0:
            # 2. 插值检测 ChArUco 角点
            retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board
            )
            
            if charuco_corners is not None and len(charuco_corners) > 4:
                # 3. 估计姿态
                valid, rvec, tvec = aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board, 
                    self.camera_matrix, self.dist_coeffs, None, None
                )

                if valid:
                    # 4. 绘制并显示
                    if hasattr(cv2, 'drawFrameAxes'):
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    else:
                        aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    
                    # 5. 发布 TF
                    # rvec 转四元数
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    # 构建 4x4 矩阵以转换四元数
                    matrix = np.eye(4)
                    matrix[:3, :3] = rot_matrix
                    quat = tf.transformations.quaternion_from_matrix(matrix)
                    
                    self.tf_broadcaster.sendTransform(
                        tvec.flatten(),
                        quat,
                        rospy.Time.now(),
                        self.board_frame,
                        self.camera_frame
                    )
                    rospy.logdebug("Published board transform")
                    
                    # 6. 发布结果图像
                    try:
                        self.result_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                    except CvBridgeError as e:
                        rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = CharucoDetector()
    detector.run()
