# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
import kinova_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg
import std_msgs.msg

finger_maxTurn = 7400

def set_finger_positions(finger_positions):
    """
    Send gripper command according to mode:
    - kinova_action: use kinova SetFingersPositionAction (counts).
    - finger_traj: send FollowJointTrajectoryGoal to finger trajectory controller (positions).
    - finger_position_topics: publish target positions to finger_tip position controllers.
    """
    global gripper_mode
    if not _config_loaded:
        _load_config()
    if gripper_mode == 'kinova_action':
        return _send_kinova_action(finger_positions)
    elif gripper_mode == 'finger_traj':
        return _send_finger_traj(finger_positions)
    elif gripper_mode == 'finger_position_topics':
        return _send_finger_position_topics(finger_positions)
    else:
        rospy.logerr("Unknown gripper_mode: %s", gripper_mode)
        return None


def _send_kinova_action(finger_positions):
    global gripper_client
    global finger_maxTurn
    if gripper_client is None:
        _init_gripper_action()
    if gripper_client is None:
        return None

    finger_positions[0] = min(finger_maxTurn, finger_positions[0])
    finger_positions[1] = min(finger_maxTurn, finger_positions[1])

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    gripper_client.send_goal(goal)
    if gripper_client.wait_for_result(rospy.Duration(5.0)):
        return gripper_client.get_result()
    else:
        gripper_client.cancel_all_goals()
        rospy.logwarn('the gripper action timed-out')
        return None


def _send_finger_traj(finger_positions):
    """Send FollowJointTrajectoryGoal to finger trajectory controller."""
    global finger_traj_client
    if finger_traj_client is None:
        _init_finger_traj()
    if finger_traj_client is None:
        return None

    # Map counts to joint positions (rad or normalized). Simple linear scale.
    joints = []
    for idx, pos in enumerate(finger_positions[:2]):
        ratio = max(0.0, min(1.0, pos / finger_counts_max))
        target = finger_open_positions[idx] + ratio * (finger_closed_positions[idx] - finger_open_positions[idx])
        joints.append(target)

    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = finger_joint_names
    pt = trajectory_msgs.msg.JointTrajectoryPoint()
    pt.positions = joints
    pt.time_from_start = rospy.Duration(finger_traj_time)
    goal.trajectory.points.append(pt)
    finger_traj_client.send_goal(goal)
    if finger_traj_client.wait_for_result(rospy.Duration(finger_traj_timeout)):
        return finger_traj_client.get_result()
    finger_traj_client.cancel_all_goals()
    rospy.logwarn("finger traj action timed-out")
    return None


def _send_finger_position_topics(finger_positions):
    """Publish directly to finger_tip position controllers."""
    global finger_tip_pubs
    if not finger_tip_pubs:
        _init_finger_tip_pubs()
    if not finger_tip_pubs:
        return None
    joints = []
    for idx, pos in enumerate(finger_positions[:2]):
        ratio = max(0.0, min(1.0, pos / finger_counts_max))
        target = finger_open_positions[idx] + ratio * (finger_closed_positions[idx] - finger_open_positions[idx])
        joints.append(target)
        finger_tip_pubs[idx].publish(target)
    return joints


def _load_config():
    """Load ROS params lazily (after rospy.init_node)."""
    global gripper_mode, action_address, finger_traj_action, finger_joint_names, finger_tip_topics
    global finger_counts_max, finger_open_positions, finger_closed_positions
    global finger_traj_time, finger_traj_timeout, _config_loaded
    gripper_mode = rospy.get_param('~gripper_mode', 'kinova_action')  # kinova_action | finger_traj | finger_position_topics
    action_address = rospy.get_param('~fingers_action', '/m1n6s200_driver/fingers_action/finger_positions')
    finger_traj_action = rospy.get_param('~finger_traj_action', '/m1n6s200/effort_finger_trajectory_controller/follow_joint_trajectory')
    finger_joint_names = rospy.get_param('~finger_joint_names', ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2'])
    finger_tip_topics = rospy.get_param('~finger_tip_topics', [
        '/m1n6s200/finger_tip_1_position_controller/command',
        '/m1n6s200/finger_tip_2_position_controller/command'
    ])
    finger_counts_max = rospy.get_param('~finger_counts_max', 8000.0)
    finger_open_positions = rospy.get_param('~finger_open_positions', [0.0, 0.0])
    finger_closed_positions = rospy.get_param('~finger_closed_positions', [0.7, 0.7])
    finger_traj_time = rospy.get_param('~finger_traj_time', 1.0)
    finger_traj_timeout = rospy.get_param('~finger_traj_timeout', 3.0)
    _config_loaded = True

def _init_finger_traj():
    global finger_traj_client
    finger_traj_client = actionlib.SimpleActionClient(finger_traj_action, control_msgs.msg.FollowJointTrajectoryAction)
    ok = finger_traj_client.wait_for_server(rospy.Duration(3.0))
    if not ok:
        rospy.logerr("Finger trajectory action server not available at %s.", finger_traj_action)
        finger_traj_client = None
    return ok


def _init_finger_tip_pubs():
    global finger_tip_pubs
    finger_tip_pubs = [rospy.Publisher(t, std_msgs.msg.Float64, queue_size=1) for t in finger_tip_topics]
    if not finger_tip_pubs:
        rospy.logerr("Finger tip position topics not available.")

def _init_gripper_action():
    global gripper_client
    gripper_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
    if not gripper_client.wait_for_server(rospy.Duration(3.0)):
        rospy.logerr("Gripper action server not available at %s. For simulation, run a compatible driver/controller exposing this action.", action_address)
        gripper_client = None
    return gripper_client

# Config / state placeholders (lazy loaded)
_config_loaded = False
gripper_mode = None
action_address = None
finger_traj_action = None
finger_joint_names = None
finger_tip_topics = None
finger_counts_max = None
finger_open_positions = None
finger_closed_positions = None
finger_traj_time = None
finger_traj_timeout = None

# Clients / pubs
gripper_client = None
finger_traj_client = None
finger_tip_pubs = []