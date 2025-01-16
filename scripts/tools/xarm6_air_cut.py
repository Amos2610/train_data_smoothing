#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math


class XArm6AirCut:
    """XArm6の高速エアカットをシミュレーションするクラス
    Attributes:
        current_joint_angles (list): 現在のジョイント角度
        pub (rospy.Publisher): Trajectoryトピックのパブリッシャ

    Returns:
        success (bool): ゴールジョイント角度が有効な場合はTrue
    """
    def __init__(self):
        self.current_joint_angles = [0, 0, 0, 0, 0, 0]
        self.goal_joint_angles = None
        self.tolerance = 0.005  # joint角度の到達許容誤差

        # ジョイントの状態を取得するサブスクライバを作成
        rospy.Subscriber('/xarm/joint_states', JointState, self.joint_state_callback)

        # Trajectoryトピックのパブリッシャを作成
        self.pub = rospy.Publisher('/xarm/xarm6_traj_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(1)  # 少し待って現在のジョイント状態を取得

    def joint_state_callback(self, msg):
        # 現在のジョイント角度を取得
        joint_names = msg.name
        if all(joint in joint_names for joint in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
            indices = [joint_names.index(joint) for joint in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']]
            self.current_joint_angles = [msg.position[i] for i in indices]

    def move_to_position(self, positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1.0)
        
        traj_msg.points.append(point)
        self.pub.publish(traj_msg)
        self.goal_joint_angles = positions

    def is_valid_joint_angles(self, angles):
        # ジョイントの角度制限（ラジアン単位）xArm6の仕様に合わせて設定
        joint_limits = {
            'joint1': (-2*math.pi, 2*math.pi),
            'joint2': (-2.059, 2.094),
            'joint3': (-3.927, 0.192),
            'joint4': (-2*math.pi, 2*math.pi),
            'joint5': (-1.692, 3.142),
            'joint6': (-2*math.pi, 2*math.pi)
        }
        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for joint_name, (min_limit, max_limit) in joint_limits.items():
            if joint_name in joint_names:
                index = joint_names.index(joint_name)
                angle = angles[index]
                if angle < min_limit or angle > max_limit:
                    return False
        return True
    
    def has_reached_goal(self):
        if self.goal_joint_angles is None:
            return False
        return all(abs(self.current_joint_angles[i] - self.goal_joint_angles[i]) < self.tolerance for i in range(len(self.goal_joint_angles)))

    def execute(self, goal_joint_angles):
        if self.is_valid_joint_angles(goal_joint_angles):
            # 高速エアカットを実行
            self.move_to_position(goal_joint_angles)

            # ゴール位置に到達するまでループ
            start_time = rospy.Time.now()
            rate = rospy.Rate(10)  # 10 Hzでチェック
            while not rospy.is_shutdown():
                if rospy.Time.now() - start_time > rospy.Duration(10):
                    break
                if self.has_reached_goal():
                    rospy.loginfo("ゴール位置に到達しました。")
                    return True
                rate.sleep()
            rospy.logwarn("ゴール位置に到達できませんでした。")
            return False
        else:
            rospy.logwarn("ゴールジョイントの角度が無効です。")
            return False

if __name__ == "__main__":
    rospy.init_node('xarm_air_cut_simulation')
    controller = XArm6AirCut()
    start_joint_angles = controller.current_joint_angles
    goal_joint_angles = [3.0863481162356745, 1.4142943957723446, -0.16558442922751349, 2.981032188783809, -0.5522573283655814, 2.852211836755565]
    controller.execute(goal_joint_angles)
    rospy.sleep(1)
