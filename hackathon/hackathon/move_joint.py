import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')

        # 액션 클라이언트 초기화
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory'
        )

        # 액션 서버 대기
        self.get_logger().info('Waiting for trajectory controller...')
        self._action_client.wait_for_server()
        self.get_logger().info('Controller available!')

        # 명령 보내기
        self.send_trajectory_goal()

    def send_trajectory_goal(self):
        # 조인트 이름
        joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # 목표 위치 (예시: 약간 구부린 자세)
        target_positions = [0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.5]

        # JointTrajectoryPoint 생성
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = 2  # 2초 안에 도달

        # JointTrajectory 메시지 구성
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points.append(point)

        # 액션 목표 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        # 액션 전송
        self.get_logger().info('Sending trajectory goal...')
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by controller.')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ Goal accepted! Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🎉 Trajectory execution complete!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommander()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
