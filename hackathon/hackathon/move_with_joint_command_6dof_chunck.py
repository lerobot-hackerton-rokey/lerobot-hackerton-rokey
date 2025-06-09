import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 로봇 제어 관련 메시지
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# # joint값 6개 입력 메시지 타입
# from std_msgs.msg import Float64MultiArray

# 만들어진 메시지
from inference_interfaces.msg import ActionChunk

class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_5', 'joint_6']
        # self.current_joint_positions = [0.0] * 7  # 현재 조인트 위치 초기화
        
        self.joint_states_info = {} # 조인트 상태 정보를 저장할 딕셔너리
        
        # ================================================
        # 이동할 joint chunck 값 수신
        self.joint_command_sub = self.create_subscription(
            ActionChunk,
            '/joint_command_chunk',
            self.joint_callback_chunck,
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
    # 상대적 이동할 조인트 청크 값을 받아서 동작
        # 1. 상대적 이동할 조인트 청크 값 수신
        # 2. 현재 조인트 값 청크 수신
        # 3. 상대적 이동할 조인트 값 청크 + 현재 조인트 값으로 이동
    def joint_callback_chunck(self, msg: ActionChunk):
        # 현재 조인트 상태가 준비되었는지 확인
        if not self.joint_states_info:
            self.get_logger().warn('⚠️ Joint states not available yet.')
            return

        # self.get_logger().info(f'🎯 Joint goal: {joint_goal}')
        # self.send_trajectory_goal(joint_goal)
        traj_cmd = self.make_trajectory_(msg)
        if traj_cmd is None:
            self.get_logger().error('❌ Failed to create trajectory from ActionChunk message.')
            return
        else:    
            self.send_trajectory_goal(traj_cmd)
        
    def make_trajectory_(self, msg: ActionChunk):
        joint_names = self.joint_names
        
        if msg.rows != 10 or msg.cols != len(self.joint_names):
            self.get_logger().error(f'❌ Expected 10 rows and {len(self.joint_names)} columns, got {msg.rows} rows and {msg.cols} columns')
            return
        
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        for row in range(msg.rows):
            six_joint_goal = [] # 조인트 목표 값 저장
            
            for col in range(msg.cols):
                current = self.joint_states_info.get(self.joint_names[col], 0.0) # ??
                delta = msg.data[row * msg.cols + col] # 조인트 이동 값
                
                six_joint_goal.append(current + delta) # 조인트 목표 값 리스트

            print("---- six_joint_goal ----")
            print(six_joint_goal)
            
            point = JointTrajectoryPoint()
            point.positions = six_joint_goal
            point.time_from_start.sec = 2 * (row + 1)  # 2초 단위로 늘어나는 시간
            
            trajectory.points.append(point)
        return trajectory
    
    # trajectory 목표 전송
    def send_trajectory_goal(self, trajectory: JointTrajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info(f'🚀 Sending goal: {trajectory}')
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
    
    # 조인트 값 읽기
    def joint_state_callback(self, msg):
        # 조인트 상태 정보를 딕셔너리에 저장
        self.joint_states_info = {name: position for name, position in zip(msg.name, msg.position)}
        
        # 확인
        # self.get_logger().info('Updated joint states:')
        # for name, pos in self.joint_states_info.items():
        #     self.get_logger().info(f'  {name}: {pos:.3f}')
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommander()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
