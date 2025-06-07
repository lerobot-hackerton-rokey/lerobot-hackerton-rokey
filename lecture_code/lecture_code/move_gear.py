# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    
    # publisher -> 조인트 값 계속 쏘는 퍼블리셔
    # subscription -> 시작 입력을 받으면 동작하는 퍼블리셔

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # grip
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            get_tool,
            get_tcp,
            
            # move
            wait,
            set_tool,
            # set_tcp,
            movej,
            movel,
            
            # force control
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
            release_force,
            
            move_periodic,
            amove_periodic,
            
            amovel,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # JReady = [0, 0, 90, 0, 90, 0] # home 위치
    
    # position
    # Global_origin_1 = posx([675.86, -10.75, 93.0, 180.0, -180.0, 180.0]) # 가운데 기어
    Global_origin_2 = posx([704.36, 45.9, 93.0, 180.0, 180.0, -180.0])
    Global_origin_3 = posx([613.54, -13.92, 93.0, 180.0, 180.0, 180.0])
    Global_origin_4 = posx([708.73, -57.87, 93.0, 127.97, -180.0, 38.44])

    # Global_dst_1 = posx([425.28, -5.89, 93.0, 180.0, 180.0, 180.0]) # 가운데 기어
    Global_dst_2 = posx([397.6, -59.76, 93.0, 180.0, 180.0, 180.0])
    Global_dst_3 = posx([487.28, -5.03, 93.0, -180.0, -180.0, 180.0])
    Global_dst_4 = posx([393.06, 46.78, 93.0, 0.0, 180.0, 0.0])
    
    up_pose = posx([0.0, 0.0, -50.0, 0.0, 0.0, 0.0])
    down_pose = posx([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
    # down_pose_little = posx([0.0, 0.0, 25.0, 0.0, 0.0, 0.0]) # 조금만 내려가는거, 기어 1에서 사용
    
    # ====================
    # grip
    ON, OFF = 1, 0
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    def grip():
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
    # ====================    

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()
        print(tool_name)
        print(tcp_name)
        
        # # ============move 2============
        # release()
        # movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # movel(Global_origin_2, vel=VELOCITY, acc=ACC)
        # wait(1.0)
        # movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # grip()
        # wait(1.0)
        # movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # movel(Global_dst_2, vel=VELOCITY, acc=ACC)
        # movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # release()
        # wait(1.0)
        
        # ============move 3============
        release()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(Global_origin_3, vel=VELOCITY, acc=ACC)
        wait(1.0)
        movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grip()
        wait(1.0)
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(Global_dst_3, vel=VELOCITY, acc=ACC)
        movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        release()
        wait(1.0)
        
        # ============move 4============
        release()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(Global_origin_4, vel=VELOCITY, acc=ACC)
        wait(1.0)
        movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grip()
        wait(1.0)
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(Global_dst_4, vel=VELOCITY, acc=ACC)
        movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        release()
        wait(1.0)
        
        # # ============move 1============
        # release()
        # movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # movel(Global_origin_1, vel=VELOCITY, acc=ACC)
        # wait(1.0)
        # movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # grip()
        # wait(1.0)
        # movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        # movel(Global_dst_1, vel=VELOCITY, acc=ACC)
        # movel(down_pose_little, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        
        # # 힘 제어
        # task_compliance_ctrl()
        # time.sleep(0.1)
        # set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # time.sleep(0.1)
        # while not check_force_condition(DR_AXIS_Z, max=10):
        #     pass
        # amove_periodic(amp=[0, 0, 0, 0, 0, 15], period=1.0, atime=0.02, repeat=3, ref=DR_TOOL)
        # wait(3.0)
        # release_force()
        # time.sleep(0.1)
        # release_compliance_ctrl()
        
        # release()
        # wait(1.0)
        # break
    rclpy.shutdown()

if __name__ == "__main__":
    main()
