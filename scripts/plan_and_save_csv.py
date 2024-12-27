#! /usr/bin/env python3

import os
import rospy
import csv
import rospkg
from moveit_commander import RobotCommander, MoveGroupCommander

# 相対パス
rospack = rospkg.RosPack()
package_path = rospack.get_path('train_data_smoothing')  # パッケージ名を指定


class PlanAndSaveCSV:
    def __init__(self):
        rospy.init_node("xArm6")
        self.robot = RobotCommander()
        self.xarm = MoveGroupCommander("xarm6")

        # CSVファイルの読み込み csv_files/after_train.csv
        self.before_csv = os.path.join(package_path, "csv_files", "after_train.csv")
        self.output_csv = 'output_after_smoothing.csv'

        # Subscriber
        #TODO: 関節角度?を取得するSubscriberを設定
        self.joint_values_sub = rospy.Subscriber(???)

    def planning(self):
        """
        ゴールの関節角度を指定してプランニングを行う
        """
        # TODO: スタートはCSVファイルのn行目の1列目から取得
        
        # ゴールの設定(関節角度で指定)
        fixed_joint_values = [0.0, -0.874, -0.932, 0.0, 1.806, 0.0]
        self.xarm.set_start_state_to_current_state()
        self.xarm.set_joint_value_target(fixed_joint_values)

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

    def plan_and_save_csv(self):
        # プランニング
        self.planning()

        # 結果をCSVファイルに保存
        joint_values = self.xarm.get_current_joint_values()
        with open("joint_values.csv", "w") as f:
            f.write("joint1,joint2,joint3,joint4,joint5,joint6\n")
            f.write(",".join(map(str, joint_values)))


if __name__ == "__main__":
    plan_and_save_csv = PlanAndSaveCSV()
    plan_and_save_csv.plan_and_save_csv()