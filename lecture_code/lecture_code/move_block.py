# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

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
        # wait_digital_input(2)

    def grip():
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1)
    # ====================    

    JReady = [0, 0, 90, 0, 90, 0]
    # 블럭 꽂을 때 높이는 20mm 부터 힘 제어
    org_1 = posx([352.690,   5.000,  60.000,  90.0, -180.0,  90.0])
    org_2 = posx([432.690,   5.000,  60.000,  90.0,  180.0,  90.0])

    dst_1 = posx([352.690, -91.000,  60.000,  90.0,  180.0,  90.0])
    dst_2 = posx([432.690, -91.000,  60.000,  90.0, -180.0,  90.0])
    
    up_pose = posx([0.0, 0.0, -50.0, 0.0, 0.0, 0.0])
    down_pose = posx([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
    down_pose_little = posx([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return
        
        release()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(org_1, vel=VELOCITY, acc=ACC)
        movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grip()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(dst_1, vel=VELOCITY, acc=ACC)
        movel(down_pose_little, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        
        # 힘 제어 1
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        wait(3.0)
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        
        # 블록 2
        release()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(org_2, vel=VELOCITY, acc=ACC)
        movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grip()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(dst_2, vel=VELOCITY, acc=ACC)
        movel(down_pose_little, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        
        # 힘제어
        # 힘 제어 1
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        wait(3.0)
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        
        release()
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        break

        

    rclpy.shutdown()
if __name__ == "__main__":
    main()
