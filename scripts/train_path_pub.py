#!/usr/bin/env python3

import rospy
from train_data_smoothing.msg import PathSeed
import numpy as np
import os
import rospkg
import csv

# rospkgを使ってパッケージのパスを取得
rospack = rospkg.RosPack()
package_path = rospack.get_path('train_data_smoothing')


class TrainPathPublisher:
    def __init__(self):
        rospy.init_node('train_path_publisher', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher('move_group/path_seed', PathSeed, queue_size=500)
        self.rate = rospy.Rate(50)  # 50 Hz

        # Variables
        self.send_row = 1000000 # 0でなければなんでもいい
        self.count = 0 # 送信回数(debug用)
        self.train_num = 100
        self.filename = os.path.join(package_path, 'csv_files', 'after_train.csv')

        # Parameters
        rospy.set_param('send_row', 0)

    def read_data_from_file(self, csv_file_path: str) -> np.ndarray:
        """
        Read data from a file and convert it to a NumPy array.
        Args:
            csv_file_path: The path to the file.
        Returns:
            A NumPy array containing the data.
        """
        all_data = []
        
        # csvファイルのパスが存在しない場合
        if not os.path.isfile(csv_file_path):
            rospy.logerr("File not found: %s", csv_file_path)
            return np.array([])

        try:
            with open(csv_file_path, 'r') as f:
                reader = csv.reader(f)
                # 1行ずつ読み込み
                for row in reader:
                    # 1行目はヘッダーなのでスキップ
                    if reader.line_num == 1:
                        continue
                    # rowはlist型
                    # すべての要素str型なので，float型に変換
                    row = [float(x) for x in row]
                    all_data.append(row)
        except Exception as e:
            rospy.logerr("Failed to read file: %s. Error: %s", csv_file_path, str(e))
            return np.array([])

        # NumPy配列でしか扱えないので，ListからNumPy配列に変換
        # all_data = np.array(all_data)
        # rospy.loginfo("Data: %s", str(all_data))
        return all_data
        
    def initial_path_publisher(self):
        # Constants
        NUM_COLS = 6 # xArm6の関節数
        
        while not rospy.is_shutdown():
            send_row = rospy.get_param('send_row', 0)
            if send_row != self.send_row:
                self.send_row = send_row
                rospy.loginfo(f"Changed send_row to {self.send_row}")
                data = self.read_data_from_file(self.filename)
                data = data[self.send_row]
                num_rows = len(data) // NUM_COLS
            # flattened_data = data.flatten()
            matrix_msg = PathSeed()
            matrix_msg.rows = num_rows
            matrix_msg.cols = NUM_COLS
            matrix_msg.data = data
            self.pub.publish(matrix_msg)
            self.count += 1
            rospy.loginfo(f"Finished publishing {self.send_row}th row. Count: {self.count}")
            self.rate.sleep()


if __name__ == '__main__':
    try:
        path_publisher = TrainPathPublisher()
        # path_publisher.read_data_from_file(path_publisher.filename)
        path_publisher.initial_path_publisher()
    except rospy.ROSInterruptException:
        pass

"""
rostopic echo /move_group/path_seedで確認
    ---
    data: [2.219969321625942, 0.7143666530728392, -1.7124492026228209, 0.006287773563697859, 0.8836985779542399, 0.4375513264828384, 2.2105467438402706, 0.6978428505387005, -1.7131325914429216, 0.005107791198183657, 0.8031697585819695, 0.33696076374079226, 2.1989900386024352, 0.6816578516693407, -1.7157238418253724, -0.0005581827604894301, 0.7394255945876138, 0.24766131095957425 ...]
    rows: 17
    cols: 6
    ---
    このような形式でデータが送信されていればOK

送信される行数 (trainデータ) はデフォルトは0行目のみ
    rosparam set send_row <送信したい行数> で送信する行数を変更可能

stomp_moveitはuse_pathseedというパラメータがTrueのときのみPathSeedを受け付ける仕様
    rosparam set use_pathseed True でPathSeedを受け付けるようになります
"""
