# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg


def move_to_position(position, orientation):
    """
    Send a cartesian goal.
    - If use_moveit: plan & execute via MoveIt.
    - Else: send to kinova pose action server.
    """
    global position_client
    global base_frame
    global use_moveit
    global move_group
    global position_scaling
    global velocity_scaling

    if use_moveit:
        if move_group is None:
            if not init_moveit():
                return None
        pose = geometry_msgs.msg.Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        move_group.set_pose_reference_frame(base_frame)
        move_group.set_pose_target(pose)
        move_group.set_max_acceleration_scaling_factor(position_scaling)
        move_group.set_max_velocity_scaling_factor(velocity_scaling)
        plan = move_group.plan()
        # move_group.plan() may return a tuple (success, traj, planning_time, error_code)
        # Normalize to a RobotTrajectory for execute.
        robot_traj = None
        if isinstance(plan, tuple) and len(plan) >= 2:
            robot_traj = plan[1]
            success_plan = plan[0]
        else:
            robot_traj = plan
            success_plan = True if plan else False
        if not success_plan or robot_traj is None:
            rospy.logwarn("MoveIt planning failed.")
            return None
        success = move_group.execute(robot_traj, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        if success:
            return True
        rospy.logwarn("MoveIt execution failed.")
        return None

    if position_client is None:
        init_action()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=base_frame)
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    position_client.send_goal(goal)

    if position_client.wait_for_result(rospy.Duration(10.0)):
        return position_client.get_result()
    else:
        position_client.cancel_all_goals()
        rospy.logwarn('the cartesian action timed-out')
        return None

# Parameters
# 真机默认关闭 MoveIt，直接使用 kinova_driver 提供的 pose action，避免等待仿真控制器
use_moveit = rospy.get_param('~use_moveit', False)
# 默认组名改为 MoveIt 常用的 'arm'，可通过参数覆盖；仅在 use_moveit=true 时使用
move_group_name = rospy.get_param('~move_group', 'arm')
position_scaling = rospy.get_param('~moveit_acc_scaling', 0.5)
velocity_scaling = rospy.get_param('~moveit_vel_scaling', 0.3)
action_address = rospy.get_param('~pose_action', '/m1n6s200_driver/pose_action/tool_pose')
base_frame = rospy.get_param('~base_frame', 'm1n6s200_link_base')

# State
position_client = None
move_group = None

def init_action():
    """Init kinova cartesian action client."""
    global position_client
    position_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    connected = position_client.wait_for_server(rospy.Duration(3.0))
    if not connected:
        rospy.logerr("Cartesian pose action server not available at %s. Ensure kinova driver (or simulated driver exposing the same action) is running.", action_address)
    return connected


def init_moveit():
    """Init MoveIt commander for the specified group."""
    global move_group
    try:
        import moveit_commander
        moveit_commander.roscpp_initialize([])
        move_group = moveit_commander.MoveGroupCommander(move_group_name)
        move_group.set_planning_time(2.0)
        move_group.allow_replanning(True)
        return True
    except Exception as e:
        rospy.logerr("Failed to init MoveIt group %s: %s", move_group_name, e)
        move_group = None
        return False
