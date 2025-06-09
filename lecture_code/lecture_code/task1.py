import rclpy
import DR_init
import time

from lecture_code.onrobot import RG

# 기본 설정
GRIPPER_NAME = "rg2"
TOOLCHARER_IP = "192.168.1.1"
TOOLCHARER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHARER_IP, TOOLCHARER_PORT)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("task1", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            set_digital_output,
            get_tool,
            get_tcp,
            wait, 
        )
        from DR_common2 import posj
    except ImportError as e:
        print(f"Import Error: {e}")
        return

    # 각 지점에 대한 Joint 값 (예시)
    j_ready  = posj([0, 0, 90, 0, 90, 0])
    j_box = posj([-9.97, 49.79, 16.22, 0.94, 113.08, -97.98])
    j_org_1  = posj([20.26, -4.47, 122.89, 0.0, 61.58, 18.43])
    j_org_2  = posj([-10.58, 5.32, 114.91, 0, 59.77, 78.64])
    j_org_3  = posj([15.99, 15.92, 101.08, 0, 63.01, 15.9])
    j_org_4 = posj([12.97, 36.44, 70.56, 0, 72.56, 12.64])

    while rclpy.ok():
        if get_tool() == "" or get_tcp() == "":
            print("Tool or TCP not set. Abort.")
            break

        movej(j_ready, vel=VELOCITY, acc=ACC)

        # 블럭 1 pick & place
        gripper.open_gripper()
        movej(j_org_1, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        wait(1.0)
        movej(j_box, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()

        gripper.open_gripper()
        movej(j_org_2, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        wait(1.0)
        movej(j_box, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        
        gripper.open_gripper()
        movej(j_org_3, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        wait(1.0)
        movej(j_box, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        
        gripper.open_gripper()
        movej(j_org_4, vel=VELOCITY, acc=ACC)
        gripper.close_gripper()
        wait(1.0)
        movej(j_box, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()

        movej(j_ready, vel=VELOCITY, acc=ACC)
        break

    rclpy.shutdown()


if __name__ == "__main__":
    main()
