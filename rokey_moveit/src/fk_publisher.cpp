// #include <rclcpp/rclcpp.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>

// #include <fstream>
// #include <sstream>
// #include <Eigen/Geometry>

// class FKPublisherNode : public rclcpp::Node
// {
// public:
//   FKPublisherNode()
//   : rclcpp::Node("fk_publisher")
//   {
//     // 1. URDF 파일 경로 (panda.urdf)
//     std::string urdf_path = ament_index_cpp::get_package_share_directory("moveit_resources_panda_description") + "/urdf/panda.urdf";
//     std::ifstream urdf_file(urdf_path);
//     std::stringstream urdf_buffer;
//     if (!urdf_file.is_open()) {
//       RCLCPP_ERROR(this->get_logger(), "URDF 파일 열기 실패: %s", urdf_path.c_str());
//       return;
//     }
//     urdf_buffer << urdf_file.rdbuf();
//     this->declare_parameter("robot_description", urdf_buffer.str());

//     // 2. SRDF 파일 경로 (panda.srdf)
//     std::string srdf_path = ament_index_cpp::get_package_share_directory("moveit_resources_panda_moveit_config") + "/config/panda.srdf";
//     std::ifstream srdf_file(srdf_path);
//     std::stringstream srdf_buffer;
//     if (!srdf_file.is_open()) {
//       RCLCPP_ERROR(this->get_logger(), "SRDF 파일 열기 실패: %s", srdf_path.c_str());
//       return;
//     }
//     srdf_buffer << srdf_file.rdbuf();
//     this->declare_parameter("robot_description_semantic", srdf_buffer.str());

//     RCLCPP_INFO(this->get_logger(), "URDF & SRDF 파라미터 등록 완료");
//   }

//   void init()
//   {
//     // 3. RobotModelLoader 초기화 (Node::shared_from_this 명시)
//     auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
//       rclcpp::Node::shared_from_this(), "robot_description");

//     const auto& kinematic_model = robot_model_loader->getModel();
//     if (!kinematic_model) {
//       RCLCPP_ERROR(this->get_logger(), "Robot model 로딩 실패");
//       return;
//     }

//     // 4. RobotState 생성 및 초기화
//     moveit::core::RobotState robot_state(kinematic_model);
//     robot_state.setToDefaultValues();

//     // 5. planning group 설정
//     const std::string group_name = "panda_arm";
//     const auto* joint_model_group = kinematic_model->getJointModelGroup(group_name);
//     if (!joint_model_group) {
//       RCLCPP_ERROR(this->get_logger(), "Joint group '%s' 없음", group_name.c_str());
//       return;
//     }

//     // 6. FK 계산용 조인트 값 설정
//     std::vector<double> joint_values = {0.0, -0.3, 0.0, -2.0, 0.0, 2.0, 0.79};
//     robot_state.setJointGroupPositions(joint_model_group, joint_values);

//     // 7. FK 결과 (엔드이펙터 pose)
//     const Eigen::Isometry3d& ee = robot_state.getGlobalLinkTransform("panda_hand");

//     RCLCPP_INFO(this->get_logger(), "End Effector Position: [%.3f, %.3f, %.3f]",
//                 ee.translation().x(), ee.translation().y(), ee.translation().z());

//     Eigen::Quaterniond q(ee.rotation());
//     RCLCPP_INFO(this->get_logger(), "End Effector Orientation (quat): [%.3f, %.3f, %.3f, %.3f]",
//                 q.x(), q.y(), q.z(), q.w());
//   }
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<FKPublisherNode>();
//   node->init();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
