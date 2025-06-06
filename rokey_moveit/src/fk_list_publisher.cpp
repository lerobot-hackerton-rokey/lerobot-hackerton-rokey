#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fstream>
#include <sstream>
#include <Eigen/Geometry>
#include <unordered_map>

class FKPublisherNode : public rclcpp::Node
{
public:
  FKPublisherNode()
  : Node("fk_list_publisher")
  {
    // 1. URDF 파라미터 등록
    std::string urdf_path =
      ament_index_cpp::get_package_share_directory("moveit_resources_panda_description")
      + "/urdf/panda.urdf";
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "URDF 파일 열기 실패: %s", urdf_path.c_str());
      return;
    }
    std::stringstream urdf_buffer;
    urdf_buffer << urdf_file.rdbuf();
    this->declare_parameter("robot_description", urdf_buffer.str());
    urdf_file.close();

    // 2. SRDF 파라미터 등록
    std::string srdf_path =
      ament_index_cpp::get_package_share_directory("moveit_resources_panda_moveit_config")
      + "/config/panda.srdf";
    std::ifstream srdf_file(srdf_path);
    if (!srdf_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "SRDF 파일 열기 실패: %s", srdf_path.c_str());
      return;
    }
    std::stringstream srdf_buffer;
    srdf_buffer << srdf_file.rdbuf();
    this->declare_parameter("robot_description_semantic", srdf_buffer.str());
    srdf_file.close();

    RCLCPP_INFO(this->get_logger(), "URDF & SRDF 파라미터 등록 완료");
    // 이 시점에는 shared_from_this()를 사용하지 않습니다.
  }

  // main() 에서 생성자 다음에 반드시 호출해야 하는 초기화 메서드
  void init()
  {
    // 3. RobotModelLoader 생성 (shared_from_this() 호출은 여기서 안전합니다)
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    if (!kinematic_model_) {
      RCLCPP_ERROR(this->get_logger(), "Robot model 로딩 실패");
      return;
    }

    // 4. RobotState 생성 및 초기화
    robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    robot_state_->setToDefaultValues();

    // 5. JointModelGroup 가져오기 ("panda_arm")
    joint_model_group_ = kinematic_model_->getJointModelGroup("panda_arm");
    if (!joint_model_group_) {
      RCLCPP_ERROR(this->get_logger(), "Joint group 'panda_arm' 없음");
      return;
    }

    // 6. Publisher / Subscriber 설정
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ee_pose", 10);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&FKPublisherNode::jointCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "FKPublisherNode 초기화 완료 (init() 호출됨)");
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!robot_state_ || !joint_model_group_) {
      RCLCPP_WARN(this->get_logger(), "로봇 상태나 그룹이 준비되지 않음");
      return;
    }

    // 1. JointState → name→position 맵으로 변환
    std::unordered_map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_map[msg->name[i]] = msg->position[i];
    }

    // 2. 원하는 순서(joint_model_group_->getVariableNames())에 따라 벡터로 추출
    std::vector<std::string> joint_names = joint_model_group_->getVariableNames();
    std::vector<double> joint_positions;
    joint_positions.reserve(joint_names.size());
    for (const auto& name : joint_names) {
      auto it = joint_map.find(name);
      if (it != joint_map.end()) {
        joint_positions.push_back(it->second);
      } else {
        RCLCPP_WARN(this->get_logger(), "Joint [%s] 값 누락", name.c_str());
        return;  // 누락된 joint가 있으면 계산을 중단
      }
    }

    // 3. FK 계산
    robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
    const Eigen::Isometry3d& ee = robot_state_->getGlobalLinkTransform("panda_hand");

    // 4. PoseStamped 메시지로 변환 및 퍼블리시
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "panda_link0";  // 프레임을 URDF 기준 베이스 링크로 설정

    pose_msg.pose.position.x = ee.translation().x();
    pose_msg.pose.position.y = ee.translation().y();
    pose_msg.pose.position.z = ee.translation().z();

    Eigen::Quaterniond q(ee.rotation());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);
  }

  // 내부 멤버 변수
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 1) shared_ptr로 노드 생성
  auto node = std::make_shared<FKPublisherNode>();

  // 2) init() 호출 — 이 시점에 shared_from_this()가 안전하게 동작함
  node->init();

  // 3) spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
