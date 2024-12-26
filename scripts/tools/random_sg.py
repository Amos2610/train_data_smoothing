import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import random


class RandomSG: 
    def generate_random_coordinates(self, range):
        """x, y, z 範囲内でランダム座標を生成"""
        x = random.uniform(range[0], range[1]) #x_min, x_max
        y = random.uniform(range[2], range[3]) #y_min, y_max
        z = random.uniform(range[4], range[5]) #z_min, z_max
        return x, y, z

    def inverse_kinematics(self, x, y, z):
        """
        x, y, z 座標を受け取り、xArm6の逆運動学を計算して関節角度を返す
        Args:
            x: x 座標
            y: y 座標
            z: z 座標
        Returns:
            関節角度のリスト
        """
        rospy.loginfo("Computing IK for coordinates: x=%f, y=%f, z=%f", x, y, z)
        rospy.wait_for_service("/compute_ik")
        ik_service = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        # MoveIt! 初期化
        robot = RobotCommander()
        move_group = MoveGroupCommander("xarm6")

        # IK 要請生成
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "xarm6"
        ik_request.ik_request.robot_state = robot.get_current_state()

        # 姿勢設定
        ik_request.ik_request.pose_stamped.header.frame_id = move_group.get_planning_frame()
        ik_request.ik_request.pose_stamped.pose.position.x = x
        ik_request.ik_request.pose_stamped.pose.position.y = y
        ik_request.ik_request.pose_stamped.pose.position.z = z
        ik_request.ik_request.pose_stamped.pose.orientation.x = 0.0
        ik_request.ik_request.pose_stamped.pose.orientation.y = -1.0
        ik_request.ik_request.pose_stamped.pose.orientation.z = 0.1
        ik_request.ik_request.pose_stamped.pose.orientation.w = 0.0
        ik_request.ik_request.timeout = rospy.Duration(1.0)

        # IK サービス呼び出し
        response = ik_service(ik_request)

        # IK 結果を確認
        if response.error_code.val != 1:
            print(f"IK computation failed for coordinates: x={x}, y={y}, z={z}")
            print(f"Error code: {response.error_code.val}")
            raise ValueError(f"IK computation failed with error code: {response.error_code.val}")

        # 実際の xArm6 関節の名前を定義
        joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]

        # 6関節の値をフィルタリング
        joint_angles = [
            response.solution.joint_state.position[response.solution.joint_state.name.index(joint)]
            for joint in joint_names
        ]
        return joint_angles


if __name__ == "__main__":
    # 1. x, y, zの最大・最小領域を定義
    start_range = [0.218, 0.415, -0.0767, 0.109, 0.0559, 0.192]
    
    rospy.init_node("inverse_kinematics_example")
    randomsg = RandomSG()

    # 2. x, y, z 座標生成
    random_x, random_y, random_z = randomsg.generate_random_coordinates(start_range)
    print(f"Generated coordinates: x={random_x}, y={random_y}, z={random_z}")

    # 3. 生成された座標を xArm 6関節座標に変換
    try:
        joint_angles = randomsg.inverse_kinematics(random_x, random_y, random_z)
        print(f"Joint angles: {joint_angles}")
    except ValueError as e:
        print(f"Error: {e}")