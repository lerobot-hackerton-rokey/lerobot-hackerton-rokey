import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_5', 'joint_6']
        # self.current_joint_positions = [0.0] * 7  # 현재 조인트 위치 초기화
        
        self.joint_states_info = {} # 조인트 상태 정보를 저장할 딕셔너리
        
        # ================================================
        # 이동할 joint 값 수신
        self.joint_command_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_command',
            self.joint_callback,
            10
        )

        # ================================================
        # 조인트 상태 구독
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # ================================================
        # 액션 클라이언트
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/dsr_moveit_controller/follow_joint_trajectory'
        )

        # 액션 서버 대기
        self.get_logger().info('Waiting for trajectory controller...')
        self._action_client.wait_for_server()
        self.get_logger().info('Controller available!')

        # # 명령 보내기
        # self.send_trajectory_goal()
    # ======================== 함수 목록 ========================
    # 상대적 이동할 조인트 값을 받아서 동작
        # 1. 상대적 이동할 조인트 값 수신
        # 2. 현재 조인트 값 수신
        # 3. 상대적 이동할 조인트 값 + 현재 조인트 값으로 이동
    def joint_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(f'❌ Expected {len(self.joint_names)} joint values, got {len(msg.data)}')
            return

        # 현재 조인트 상태가 준비되었는지 확인
        if not self.joint_states_info:
            self.get_logger().warn('⚠️ Joint states not available yet.')
            return

        # 현재값 + 델타 → 딕셔너리로 저장
        joint_goal = {}
        for i, name in enumerate(self.joint_names):
            current = self.joint_states_info.get(name, 0.0) # ??
            delta = msg.data[i]
            joint_goal[name] = current + delta

        self.get_logger().info(f'🎯 Joint goal: {joint_goal}')
        self.send_trajectory_goal(joint_goal)

    # 조인트 값 읽기
    def joint_state_callback(self, msg):
        # 조인트 상태 정보를 딕셔너리에 저장
        self.joint_states_info = {name: position for name, position in zip(msg.name, msg.position)}
        
        # 확인
        # self.get_logger().info('Updated joint states:')
        # for name, pos in self.joint_states_info.items():
        #     self.get_logger().info(f'  {name}: {pos:.3f}')

    
    def send_trajectory_goal(self, joint_goal: dict):
        # 딕셔너리에서 이름과 위치 분리
        joint_names = self.joint_names
        target_positions = [joint_goal[name] for name in joint_names]

        # 메시지 구성
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = 2

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'🚀 Sending goal: {joint_goal}')
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


# ============================아카이브============================
# origin
    # def send_trajectory_goal(self, joint_goal):
    #     # 조인트 이름
    #     joint_names = [
    #         'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
    #         'panda_joint5', 'panda_joint6', 'panda_joint7'
    #     ]

    #     # 목표 위치 (예시: 약간 구부린 자세)
    #     target_positions = [0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.5]

    #     # JointTrajectoryPoint 생성
    #     point = JointTrajectoryPoint()
    #     point.positions = target_positions
    #     point.time_from_start.sec = 2  # 2초 안에 도달

    #     # JointTrajectory 메시지 구성
    #     trajectory = JointTrajectory()
    #     trajectory.joint_names = joint_names
    #     trajectory.points.append(point)

    #     # 액션 목표 생성
    #     goal_msg = FollowJointTrajectory.Goal()
    #     goal_msg.trajectory = trajectory

    #     # 액션 전송
    #     self.get_logger().info('Sending trajectory goal...')
    #     self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)