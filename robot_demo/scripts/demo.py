#!/usr/bin/env python

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from icecream import ic
import csv

class robot_move:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('elfin_arm')

        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)

        # 当运动规划失败后，允许重新规划
        # self.arm.allow_replanning(True)

        # 设置允许的最大速度和加速度比例因子
        self.arm.set_max_acceleration_scaling_factor(0.15)
        self.arm.set_max_velocity_scaling_factor(0.15)

    def robot_move_init(self,init_joints_position:list):
        # 初始化机械臂位置
        joint_positions = init_joints_position
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go()
        rospy.sleep(2)
    
    def robot_fk_move(self,target_position:list):
        # 设置目标关节角度
        joint_positions = target_position
        self.arm.set_joint_value_target(joint_positions)
        # 规划运动路径
        tplan_success, plan, planning_time, error_code = self.arm.plan()
        print('planning_time: %f' %(planning_time))
        print(f'error_code: {error_code}')


        ic(tplan_success)

        if tplan_success == True:

            positions = []
            velocities = []

            #ic(plan)
            trajectory = plan.joint_trajectory  # 获取 joint_trajectory 对象
            for point in trajectory.points:
                positions.append([round(x, 5) for x in point.positions])
                velocities.append([round(x, 5) for x in point.velocities])

            ic(positions)
            ic(velocities)
            with open('data.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
    
                # 逐行写入交叉数据
                for i in range(len(positions)):
                    row = positions[i] 
                    writer.writerow(row)
                    row = velocities[i]
                    writer.writerow(row)

            # 按照规划的运动路径控制机械臂运动
            self.arm.execute(plan)
            rospy.sleep(2)
        else:
            rospy.loginfo('Planning failed')
    
    def robot_ik_move(self,target_position:list):
        # 获取终端link的名称
        end_effector_link = self.arm.get_end_effector_link()
        
         # 设置目标位置所使用的参考坐标系
        reference_frame = 'elfin_base'
        self.arm.set_pose_reference_frame(reference_frame)

        # 设置机械臂工作空间中的目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = target_position[0]
        target_pose.pose.position.y = target_position[1]
        target_pose.pose.position.z = target_position[2]
        target_pose.pose.orientation.x = target_position[3]
        target_pose.pose.orientation.y = target_position[4]
        target_pose.pose.orientation.z = target_position[5]
        target_pose.pose.orientation.w = target_position[6]

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, end_effector_link)
        # 规划运动路径
        tplan_success, plan, planning_time, error_code = self.arm.plan()

        print('planning_time: %f' %(planning_time))
        print(f'error_code: {error_code}')


        ic(tplan_success)

        if tplan_success == True:
            # 按照规划的运动路径控制机械臂运动
            self.arm.execute(plan)
            ic(plan)
            rospy.sleep(2)
        else:
            rospy.loginfo('Planning failed')

    def robot_exit(self):
         # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    rospy.init_node('robot')
    robot = robot_move()
    robot.robot_move_init([0,0,0,0,0,0])
    # robot.robot_fk_move([0.5,0.5,0.5,0.5,0.5,0.5])
    # robot.robot_ik_move([-0.206002,0.017636,0.25,0.5,-0.5,0.5,0.5])
    robot.robot_exit()

