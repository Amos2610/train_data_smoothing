#! /usr/bin/env python3

import os
import rospy
import csv
import rospkg
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from tools.xarm6_air_cut import XArm6AirCut

# 相対パス
rospack = rospkg.RosPack()
package_path = rospack.get_path('train_data_smoothing')  # パッケージ名を指定


class PlanAndSaveCSV:
    def __init__(self):
        rospy.init_node("xArm6")
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")

        self.air_cut = XArm6AirCut()

        # メッセージの初期化
        self.msg = None

        # CSVファイルの読み込み csv_files/after_train.csv
        self.input_csv = os.path.join(package_path, "csv_files", "after_train.csv")
        self.output_csv = 'output_after_smoothing3.csv'

        # Subscriber
        self.subscriber = rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, self.execute_trajectory_callback)

    def execute_trajectory_callback(self, msg):
        # Callback to handle the subscribed data
        self.msg = msg

    def set_state_and_goal_joint_values(self, row_num=0):
        """
        スタートとゴールの関節角度を取得
        """
        joint_data = []
        with open(self.input_csv, "r") as f:
            reader = csv.reader(f)
            # row_num行目のみ取得
            for i, row in enumerate(reader):
                # 1行目はヘッダーなのでスキップ
                # if i == 1:
                #     continue
                if i == row_num:
                    joint_data = row
                    break

        # 取得したデータのうち，最初の6つをスタート位置，最後の6つをゴール位置とする
        start_joint_values = [float(joint) for joint in joint_data[:6]]
        goal_joint_values = [float(joint) for joint in joint_data[-6:]]
        
        return start_joint_values, goal_joint_values

    def planning(self, goal_joint_values):
        """
        プランニング
        """
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_joint_value_target(goal_joint_values)

        # プランニング
        success, plan, _, _ = self.xarm.plan()

        if success:
            success_exe = self.xarm.execute(plan)
            if success_exe:
                print("Planning and execution succeeded.")
            else:
                print("Execution failed.")
        else:
            print("Planning failed.")

    def get_angle_data(self, trajectory) -> list:
        """
        Trajectoryの結果(self.msg)をangle_dataに変換
        Args:
            trajectory: trajectory.joint_trajectory.points
        Returns:
            angle_data: [[s1, s2, s3, s4, s5, s6], [w1, w2, ..., w6], ..., [g1, g2, ..., g6]]
        """
        angle_data = []
        for point in trajectory:
            # 各ポイントのpositionsを追加
            angle_data.append(list(point.positions))
        return angle_data

    def save_csv(self, start_joint_values, angle_data, goal_joint_values) -> bool:
        """
        CSVファイルに書き込み
        Args:
            start_joint_values: スタートの関節角度
            angle_data: 各ウェイポイントの関節角度
            goal_joint_values: ゴールの関節角度
        Returns:
            bool: 保存成功->True, 保存失敗->False
        """
        try:
            # すべてのデータを1行にまとめる
            all_data = []
            all_data.extend(start_joint_values)  # スタートの関節角度
            for waypoint in angle_data:  # 各ウェイポイントの関節角度
                all_data.extend(waypoint)
            all_data.extend(goal_joint_values)  # ゴールの関節角度

            # CSVファイルに書き込み
            output_path = os.path.join(package_path, "csv_files", self.output_csv)
            
            # ファイルが既存かどうかチェック
            file_exists = os.path.exists(output_path)

            with open(output_path, "a", newline="") as f:
                writer = csv.writer(f)
                
                # ヘッダーが必要な場合のみ書き込む
                if not file_exists:
                    header = ["sa", "sb", "sc", "sd", "se", "sf", "m8a", "m8b", "m8c", "m8d", "m8e", "m8f", 
                            "m4a", "m4b", "m4c", "m4d", "m4e", "m4f", "m9a", "m9b", "m9c", "m9d", "m9e", "m9f", 
                            "m2a", "m2b", "m2c", "m2d", "m2e", "m2f", "m10a", "m10b", "m10c", "m10d", "m10e", "m10f", 
                            "m5a", "m5b", "m5c", "m5d", "m5e", "m5f", "m11a", "m11b", "m11c", "m11d", "m11e", "m11f", 
                            "m1a", "m1b", "m1c", "m1d", "m1e", "m1f", "m12a", "m12b", "m12c", "m12d", "m12e", "m12f", 
                            "m6a", "m6b", "m6c", "m6d", "m6e", "m6f", "m13a", "m13b", "m13c", "m13d", "m13e", "m13f", 
                            "m3a", "m3b", "m3c", "m3d", "m3e", "m3f", "m14a", "m14b", "m14c", "m14d", "m14e", "m14f", 
                            "m7a", "m7b", "m7c", "m7d", "m7e", "m7f", "m15a", "m15b", "m15c", "m15d", "m15e", "m15f", 
                            "ga", "gb", "gc", "gd", "ge", "gf"]
                    writer.writerow(header)
                
                # データを追加
                writer.writerow(all_data)

            return True
        except Exception as e:
            print(e)
            return False

    def execute(self):
        """
        メイン関数
        """
        # 学習数
        train_num = 101

        start_row = input("Start row number: ")
        rospy.set_param('send_row', int(start_row))

        for i in range(train_num - int(start_row) + 1):
            if rospy.is_shutdown():
                break
            rospy.set_param('use_pathseed', True)
            # 保存する行数を取得
            send_row = rospy.get_param('send_row', 0)
            print(f"========================= Send row: {send_row} =========================")

            # スタートとゴールの関節角度を取得
            start_joint_values, goal_joint_values = self.set_state_and_goal_joint_values(send_row)
            print(f"Start joint values: {start_joint_values}, \nGoal joint values: {goal_joint_values}")
            # rospy.set_param("use_pathseed", False)
            # # スタート位置に移動
            # self.planning(start_joint_values)
            # rospy.set_param('use_pathseed', True)
            self.air_cut.execute(start_joint_values)
            rospy.loginfo("Start position reached.")
            print("============== Planning to goal position ==============")
            # プランニング
            rospy.loginfo("Planning to goal position...")
            self.planning(goal_joint_values)
            rospy.loginfo("Goal position reached.")
            
            print("=================== Saving CSV file ===================")
            # ウェイポイントの関節角度を取得
            angle_data = self.get_angle_data(self.msg.goal.trajectory.joint_trajectory.points)

            succeess = self.save_csv(start_joint_values, angle_data, goal_joint_values)
            if succeess:
                rospy.loginfo("Successfully saved csv file.")
                # 保存した行数をインクリメント
                rospy.set_param('send_row', send_row+1)
            else:
                rospy.loginfo("Failed to save csv file.")
        
        rospy.loginfo("Finished saving csv files.")
            

if __name__ == "__main__":
    plan_and_save_csv = PlanAndSaveCSV()
    plan_and_save_csv.execute()