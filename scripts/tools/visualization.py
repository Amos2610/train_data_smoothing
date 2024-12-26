#!/usr/bin/env python3
# coding: UTF-8

import ast
import os
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
# 視覚化プログラム
class VisualOnePath:
    def set_pose(self, pose_tuple):
        pose = Pose()
        pose.position.x = pose_tuple[0]
        pose.position.y = pose_tuple[1]
        pose.position.z = pose_tuple[2]
        pose.orientation.x = pose_tuple[3]
        pose.orientation.y = pose_tuple[4]
        pose.orientation.z = pose_tuple[5]
        pose.orientation.w = pose_tuple[6]
        return pose

    def append_marker_array(self, positions_list, marker_id_offset=0):
        marker_array = MarkerArray()

        # Add start sphere for the first coordinate
        first_pose = self.set_pose(positions_list[0])
        first_marker = Marker()
        first_marker.header.frame_id = "world"
        first_marker.header.stamp = rospy.Time.now()
        first_marker.ns = "end_effector_markers"
        first_marker.id = marker_id_offset
        first_marker.type = Marker.SPHERE
        first_marker.action = Marker.ADD
        first_marker.pose = first_pose
        first_marker.scale.x = 0.05
        first_marker.scale.y = 0.05
        first_marker.scale.z = 0.05
        first_marker.color.a = 1.0
        first_marker.color.r = 1.0
        first_marker.color.g = 0.0
        first_marker.color.b = 0.0
        marker_array.markers.append(first_marker)

        # Add middle spheres for intermediate coordinates
        intermediate_scale = 0.018
        for i, position in enumerate(positions_list[1:-1]):
            intermediate_pose = self.set_pose(position)
            intermediate_marker = Marker()
            intermediate_marker.header.frame_id = "world"
            intermediate_marker.header.stamp = rospy.Time.now()
            intermediate_marker.ns = "intermediate_markers"
            intermediate_marker.id = marker_id_offset + i + 1
            intermediate_marker.type = Marker.SPHERE
            intermediate_marker.action = Marker.ADD
            intermediate_marker.pose = intermediate_pose
            intermediate_marker.scale.x = intermediate_scale
            intermediate_marker.scale.y = intermediate_scale
            intermediate_marker.scale.z = intermediate_scale
            intermediate_marker.color.a = 1.0
            intermediate_marker.color.r = 0.9
            intermediate_marker.color.g = 0.9
            intermediate_marker.color.b = 0.0
            marker_array.markers.append(intermediate_marker)

        # Add goal sphere for the last coordinate
        last_pose = self.set_pose(positions_list[-1])
        last_marker = Marker()
        last_marker.header.frame_id = "world"
        last_marker.header.stamp = rospy.Time.now()
        last_marker.ns = "end_effector_markers"
        last_marker.id = marker_id_offset + len(positions_list)
        last_marker.type = Marker.SPHERE
        last_marker.action = Marker.ADD
        last_marker.pose = last_pose
        last_marker.scale.x = 0.05
        last_marker.scale.y = 0.05
        last_marker.scale.z = 0.05
        last_marker.color.a = 1.0
        last_marker.color.r = 0.0
        last_marker.color.g = 0.0
        last_marker.color.b = 1.0
        marker_array.markers.append(last_marker)

        # Add green line for all coordinates
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "end_effector_trajectory"
        marker.id = marker_id_offset + len(positions_list) + 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for i, position in enumerate(positions_list):
            pose = self.set_pose(position)
            marker.points.append(pose.position)

        marker_array.markers.append(marker)
        return marker_array

    # １つのパスだけを表示する
    def visualize_one_path(self, file_path):
        rospy.init_node("visualize_path")

        # ファイルからデータを読み込み，リストに入れる
        with open(file_path) as file:
            positions_list = [ast.literal_eval(line) for line in file]

        # マーカをpublish
        marker_pub = rospy.Publisher("visualization_path", MarkerArray, queue_size=10)
        rospy.sleep(1)

        marker_array = self.append_marker_array(positions_list)
        marker_pub.publish(marker_array)

        rospy.spin()

    # すべてのパスを1秒ごときに表示
    def visualize_all_path_1sec(self, directory_path):
        rospy.init_node("visualize_path")

        # すべてのtxtを読み込み
        file_paths = [os.path.join(directory_path, file) for file in os.listdir(directory_path) if file.endswith(".txt")]

        # マーカをpublish
        marker_pub = rospy.Publisher("visualization_path", MarkerArray, queue_size=10)
        rospy.sleep(1)  

        for file_path in file_paths:
            # ファイルからデータを読み込み，リストに入れる
            with open(file_path) as file:
                positions_list = [ast.literal_eval(line) for line in file]

            marker_array = self.append_marker_array(positions_list)
            marker_pub.publish(marker_array)
            rospy.sleep(1.0)

        rospy.spin()

    # すべてのパスを1回に表示
    def visualize_all_path(self, directory_path):
        rospy.init_node("visualize_path")

        # 回数指定できるようにする
        visual_count = 30

        # 回数分のtxtを読み込み
        # file_paths = [os.path.join(directory_path, file) for file in os.listdir(directory_path) if file.endswith("visual.txt")]
        file_paths = [os.path.join(directory_path, file) for file in os.listdir(directory_path) if file.endswith("_visual.txt")][:visual_count]

        # マーカをpublish
        marker_pub = rospy.Publisher("visualization_path", MarkerArray, queue_size=10)
        rospy.sleep(1)  

        marker_id_offset = 0  # マーカID初期化

        # 各ファイル別にデータをマーカpublish
        for i, file_path in enumerate(file_paths):
            rospy.loginfo(f"file_path: {file_path}")
            with open(file_path) as file:
                positions_list = [ast.literal_eval(line) for line in file]

            marker_array = self.append_marker_array(positions_list, marker_id_offset)
            marker_pub.publish(marker_array)
            marker_id_offset += len(positions_list) + 2  # マーカIDオフセットアップデート

            rospy.loginfo(f"Published markers for file {i + 1}/{len(file_paths)}")
            rospy.sleep(0.5) 

        rospy.spin()


if __name__ == "__main__":
    visualization1 = VisualOnePath()
    directory_path = '/home/nishidalab/train_ws/src/pr_pick_and_place_pkg/io/angle_data/compare_ex1_v5_original_stomp_pathseed/ompl/angle_place'
    print("Visualizing all paths in directory...")
    visualization1.visualize_all_path(directory_path)