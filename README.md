# lerobot-hackerton-rokey



### hackton 패키지 사용방법 - 위석환 작성
1. moveit으로시뮬레이션이나 실제 로봇을 켠다 
2. "ros2 run hackathon move_with_joint_command_6dof" or 7dof 를 실행한다
3. 이동할 joint 값을 전송한다 ex) ros2 topic pub /joint_command std_msgs/msg/Float64MultiArray "data: [1.0, 0.5, -0.3, 0,0,0]"   

결과 = 현재 값에서 이동할 조인트값만큼 이동된다


  