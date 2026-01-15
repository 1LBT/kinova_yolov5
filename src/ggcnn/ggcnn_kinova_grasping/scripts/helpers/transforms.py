# -*- coding: utf-8 -*-
"""
坐标变换辅助函数

提供ROS TF2坐标变换的封装函数，用于在不同坐标系之间转换位姿。

修复内容：
1. 增加TF查找超时时间（1秒->3秒）
2. 修复时间戳设置错误（rospy.Time().now -> rospy.Time.now()）
3. 增加重试机制
4. 改进错误日志
"""

import rospy
import geometry_msgs.msg as gmsg
import tf2_ros
import tf2_geometry_msgs

# Lazy create on use (convert_pose) to avoid errors.
tfBuffer = None
listener = None


def _init_tf():
    """
    初始化TF缓冲区和监听器
    必须在rospy.init_node()之后调用
    """
    global tfBuffer, listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # 等待缓冲区填充
    rospy.sleep(0.5)


def quaternion_to_list(quaternion):
    """将Quaternion消息转换为列表 [x, y, z, w]"""
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def list_to_quaternion(l):
    """将列表 [x, y, z, w] 转换为Quaternion消息"""
    q = gmsg.Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    q.w = l[3]
    return q


def convert_pose(pose, from_frame, to_frame, timeout=3.0, max_retries=2):
    """
    使用TF在不同坐标系之间转换位姿
    
    参数:
        pose: geometry_msgs/Pose 要转换的位姿
        from_frame: 源坐标系名称
        to_frame: 目标坐标系名称
        timeout: TF查找超时时间（秒）
        max_retries: 最大重试次数
        
    返回:
        转换后的Pose，失败返回None
        
    注意:
        TF变换方向: from_frame -> to_frame
        即查找的是 to_frame 相对于 from_frame 的变换
    """
    global tfBuffer, listener

    if tfBuffer is None or listener is None:
        _init_tf()

    for attempt in range(max_retries):
        try:
            # 查找变换: from_frame -> to_frame
            trans = tfBuffer.lookup_transform(
                to_frame,      # 目标坐标系
                from_frame,    # 源坐标系
                rospy.Time(0), # 使用最新可用的变换
                rospy.Duration(timeout)
            )
            
            # 创建带时间戳的位姿
            spose = gmsg.PoseStamped()
            spose.pose = pose
            spose.header.stamp = rospy.Time.now()  # 修复: 正确的调用方式
            spose.header.frame_id = from_frame

            # 应用变换
            p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)
            
            rospy.logdebug("坐标变换成功: %s -> %s", from_frame, to_frame)
            return p2.pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('TF变换失败 (尝试 %d/%d): %s -> %s, 错误: %s', 
                         attempt + 1, max_retries, from_frame, to_frame, str(e))
            if attempt < max_retries - 1:
                rospy.sleep(0.5)
    
    rospy.logerr('坐标变换最终失败: %s -> %s', from_frame, to_frame)
    rospy.logerr('请检查:')
    rospy.logerr('  1. TF树是否完整 (使用 rosrun tf view_frames 查看)')
    rospy.logerr('  2. robot_state_publisher 是否运行')
    rospy.logerr('  3. 坐标系名称是否正确')
    return None


def current_robot_pose(reference_frame, base_frame):
    """
    Get the current pose of the robot in the given reference frame
        reference_frame         -> A string that defines the reference_frame that the robots current pose will be defined in
    """
    # Create Pose
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Transforms robots current pose to the base reference frame
    return convert_pose(p, base_frame, reference_frame)


def publish_stamped_transform(stamped_transform, seconds=1):
    """
    Publish a stamped transform for debugging purposes.
        stamped_transform       -> A geometry_msgs/TransformStamped to be published
        seconds                 -> An int32 that defines the duration the transform will be broadcast for
    """
    # Create broadcast node
    br = tf2_ros.TransformBroadcaster()

    # Publish once first.
    stamped_transform.header.stamp = rospy.Time.now()
    br.sendTransform(stamped_transform)

    # Publish transform for set time.
    i = 0
    iterations = seconds/0.05
    while not rospy.is_shutdown() and i < iterations:
        stamped_transform.header.stamp = rospy.Time.now()
        br.sendTransform(stamped_transform)
        rospy.sleep(0.05)
        i += 1


def publish_transform(transform, reference_frame, name, seconds=1):
    """
    Publish a Transform for debugging purposes.
        transform           -> A geometry_msgs/Transform to be published
        reference_frame     -> A string defining the reference frame of the transform
        seconds             -> An int32 that defines the duration the transform will be broadcast for
    """
    # Create a stamped_transform and store the transform in it
    st = gmsg.TransformStamped()
    st.transform = transform
    st.header.frame_id = reference_frame
    st.child_frame_id = name

    # Call the publish_stamped_transform function
    publish_stamped_transform(st, seconds)


def publish_pose_as_transform(pose, reference_frame, name, seconds=1):
    """
    Publish a pose as a transform so that it is visualised in rviz.
    pose                -> A geometry_msgs.msg/Pose to be converted into a transform and published
    reference_frame     -> A string defining the reference_frame of the pose
    name                -> A string defining the child frame of the transform
    seconds             -> An int32 that defines the duration the transform will be broadcast for
    """

    # Create a broadcast node and a stamped transform to broadcast
    t = gmsg.TransformStamped()

    # Prepare broadcast message
    t.header.frame_id = reference_frame
    t.child_frame_id = name

    # Copy in pose values to transform
    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation

    # Call the publish_stamped_transform function
    publish_stamped_transform(t, seconds)


def publish_tf_quaterion_as_transform(translation, quaternion, reference_frame, name, seconds=1):
    qm = gmsg.Transform()
    qm.translation.x = translation[0]
    qm.translation.y = translation[1]
    qm.translation.z = translation[2]
    qm.rotation.x = quaternion[0]
    qm.rotation.y = quaternion[1]
    qm.rotation.z = quaternion[2]
    qm.rotation.w = quaternion[3]
    publish_transform(qm, reference_frame, name, seconds)


def align_pose_orientation_to_frame(from_pose, from_reference_frame, to_reference_frame):
    """
    Update the orientation of from_pose so that it matches the orientation of to_reference_frame
    Useful for aligning a desired position pose with a gripper, for example.
        from_pose                   -> A geometry_msgs.msg/Pose to allign
        from_reference_frame        -> A string defining the reference_frame of the pose
        to_reference_frame          -> A string defining the reference_frame to allign to
    """
    # Create transform
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Convert reference frame orientation from -> to
    pose_orientation = convert_pose(p, to_reference_frame, from_reference_frame)

    # Copy orientation to pose.
    from_pose.orientation = pose_orientation.orientation

    return from_pose