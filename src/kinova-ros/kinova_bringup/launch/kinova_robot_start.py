#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
import tf

class SimpleArmController:
    def __init__(self):
        # 初始化MoveIt（假设已经启动）
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建机械臂运动组
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        
        # 设置运动参数
        self.arm_group.set_planning_time(5)
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        
        rospy.loginfo("机械臂控制器就绪")
    
    def move_to_pose(self, position, orientation):
        """移动到指定位置和姿态"""
        try:
            pose_goal = Pose()
            pose_goal.position = Point(*position)
            pose_goal.orientation = Quaternion(*orientation)
            
            self.arm_group.set_pose_target(pose_goal)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            return success
        except Exception as e:
            rospy.logerr("移动失败: %s", str(e))
            return False
    
    def get_current_pose(self):
        """获取当前位置"""
        return self.arm_group.get_current_pose().pose
    
    def show_menu(self):
        """显示控制菜单"""
        print("\n" + "="*50)
        print("Kinova Mico2 机械臂位置控制")
        print("="*50)
        print("1. 显示当前位置")
        print("2. 移动到标定起点1 (前方视角)")
        print("3. 移动到标定起点2 (侧方视角)")
        print("4. 移动到标定起点3 (上方视角)")
        print("5. 自定义位置")
        print("6. 回到Home位置")
        print("7. 退出")
        print("="*50)

def main():
    rospy.init_node('simple_arm_control', anonymous=True)
    controller = SimpleArmController()
    
    # 预定义的标定位置（需要根据实际情况调整）
    calibration_poses = {
        '1': {  # 前方视角
            'position': [0.3, -0.1, 0.3],
            'orientation': [0.7, 0.0, 0.0, 0.7]  # 相机朝前
        },
        '2': {  # 侧方视角
            'position': [0.2, -0.3, 0.3],
            'orientation': [0.5, 0.5, 0.5, 0.5]  # 相机朝侧面
        },
        '3': {  # 上方视角
            'position': [0.1, -0.1, 0.5],
            'orientation': [0.0, 0.7, 0.0, 0.7]  # 相机朝下
        }
    }
    
    try:
        while not rospy.is_shutdown():
            controller.show_menu()
            choice = input("请选择功能 (1-7): ").strip()
            
            if choice == '1':
                # 显示当前位置
                current_pose = controller.get_current_pose()
                print(f"\n当前位置:")
                print(f"  X: {current_pose.position.x:.3f}m")
                print(f"  Y: {current_pose.position.y:.3f}m") 
                print(f"  Z: {current_pose.position.z:.3f}m")
                print(f"姿态: qx={current_pose.orientation.x:.3f}, "
                      f"qy={current_pose.orientation.y:.3f}, "
                      f"qz={current_pose.orientation.z:.3f}, "
                      f"qw={current_pose.orientation.w:.3f}")
            
            elif choice in ['2', '3', '4']:
                # 移动到预定义标定位置
                pose = calibration_poses[choice]
                print(f"\n移动到标定起点{choice}...")
                if controller.move_to_pose(pose['position'], pose['orientation']):
                    print("移动成功!")
                else:
                    print("移动失败!")
            
            elif choice == '5':
                # 自定义位置
                try:
                    print("\n请输入目标位置 (单位: 米):")
                    x = float(input("X: "))
                    y = float(input("Y: "))
                    z = float(input("Z: "))
                    
                    print("请输入姿态四元数:")
                    qx = float(input("qx: "))
                    qy = float(input("qy: "))
                    qz = float(input("qz: "))
                    qw = float(input("qw: "))
                    
                    if controller.move_to_pose([x, y, z], [qx, qy, qz, qw]):
                        print("移动成功!")
                    else:
                        print("移动失败!")
                        
                except ValueError:
                    print("输入格式错误，请重新输入!")
            
            elif choice == '6':
                # Home位置（需要根据您的机械臂设置调整）
                home_position = [0.0, -0.3, 0.4]  # 示例Home位置
                home_orientation = [0.0, 0.0, 0.0, 1.0]  # 默认姿态
                print("\n回到Home位置...")
                if controller.move_to_pose(home_position, home_orientation):
                    print("回到Home位置成功!")
                else:
                    print("移动失败!")
            
            elif choice == '7':
                print("退出程序")
                break
            
            else:
                print("无效选择，请重新输入!")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        rospy.logerr("程序运行出错: %s", str(e))
    finally:
        moveit_commander.roscpp_shutdown()
        print("程序结束")

if __name__ == '__main__':
    main()