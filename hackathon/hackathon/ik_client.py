import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK

class OneTimeIKRequester(Node):
    def __init__(self):
        super().__init__('one_time_ik_requester')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')

        self.request_sent = False

    def joint_state_callback(self, msg):
        if self.request_sent:
            return  # 단 한 번만 실행

        self.request_sent = True
        self.get_logger().info('Received joint states. Sending IK request...')

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'panda_arm'
        req.ik_request.ik_link_name = 'panda_link7'
        req.ik_request.robot_state.joint_state = msg

        # 원하는 목표 Pose (수정 가능)
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.4
        pose.pose.position.z = 0.4
        pose.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = pose

        req.ik_request.timeout.sec = 2

        future = self.cli.call_async(req)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        try:
            res = future.result()
            if res.error_code.val == res.error_code.SUCCESS:
                self.get_logger().info('IK solution found:')
                for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position):
                    self.get_logger().info(f'{name}: {pos:.3f}')
            else:
                self.get_logger().warn(f'IK failed with code: {res.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            # 노드 종료
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OneTimeIKRequester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
