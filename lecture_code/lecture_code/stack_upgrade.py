import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("rokey_stacking", namespace=ROBOT_ID)
DR_init.__dsr__node = node

ON, OFF = 1, 0
HOME_READY = [0, 0, 90, 0, 90, 0]

position_A = "[500.0, 0.0, 200.0, 0.0, 180.0, 0.0]"
position_B = "[500.0, 50.0, 200.0, 0.0, 180.0, 0.0]"
position_C = "[500.0, -50.0, 200.0, 0.0, 180.0, 0.0]"

import time

try:
    from DSR_ROBOT2 import (
        get_digital_input,
        set_digital_output,
        get_current_posx,
        trans,
        set_tool,
        set_tcp,
        movej,
        movel,
        wait,
        mwait,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,
        check_force_condition,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

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
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait_digital_input(1)


class Box:
    def __init__(self, id, pos_id, position):
        self.id = id
        self.pos_id = pos_id
        self.position = position
        self.target_offset = 100
        self.stacked = False

    def set_pos_id(self, pos_id):
        self.pos_id = pos_id

    def set_box_id(self, id):
        self.id = id
    
    def set_position(self, pos_list):
        self.position = pos_list
    
    def info(self):
        return f"id : {self.id}\nposition : {self.pos_id} -> {self.position}\n=====\n"

    def __move_to_pos(self, target_pos, action = None):
        # 목표 위치로 이동
        movel(target_pos, vel=VELOCITY, acc=ACC, mod=0)
        print(target_pos)
        
        # 1. 그립 후 힘 제어로 바닥 or 물체 확인
        grip()
        mwait()
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(1)
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        print("end force control")
        release_force()
        release_compliance_ctrl()
        time.sleep(1)
                
        mwait()
        
        # A. 바닥 확인 후 물체 놓고 종료 
        if action == 'stack':
            # A.1. 살짝 올라가서 물체 놓기
            movel([0,0,10,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
            release()
            mwait()
            movej(HOME_READY, vel=VELOCITY, acc=ACC)
            
        # B. 물체 확인 후 가져와서 홈 위치에 내려놓기
        elif action == 'unstack':
            # B.1 살짝 올라가서 물체 잡기
            movel([0,0,20,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
            release()
            movel([0,0,-35,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
            grip()
            
            # B.2 살짝 올라가서 홈 위치로 이동
            movel([0,0,50,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
            mwait()
            movej(HOME_READY, vel=VELOCITY, acc=ACC)
            
            # B.3 힘 제어로 바닥 확인 후 내려놓기
            
            movel([0, 0, -150, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)
            mwait()
            
            ## B.3.1 힘제어
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            time.sleep(1)
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            print("end force control")
            release_force()
            release_compliance_ctrl()
            time.sleep(1)
            
            ## B.3.1 살짝 올라가서 물건 내려놓기
            movel([0,0,10,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
            release()
            mwait()
            movej(HOME_READY, vel=VELOCITY, acc=ACC)
        
        return target_pos

    def stack(self):
        if self.stacked:
            print(f"Box {self.id} is already stacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return
        self.__move_to_pos(self.position, action='stack')
        self.stacked=True

    def unstack(self):
        if not self.stacked:
            print(f"Box {self.id} is already unstacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return
        self.__move_to_pos(self.position, action='unstack')
        self.stacked = False

def to_grip():
    movej(HOME_READY, vel=VELOCITY, acc=ACC)
    grip()
    mwait()
    movel([0,0,-100,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(1)
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    while not check_force_condition(DR_AXIS_Z, max=5):
        pass
    print("end force control")
    release_force()
    release_compliance_ctrl()
    time.sleep(1)
    movel([0,0,20,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    release()
    movel([0,0,-35,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    grip()
    movej(HOME_READY, vel=VELOCITY, acc=ACC)
    # movej([0.0, 15.85, 45.0, 0.0, 120.0, 0.0], vel=VELOCITY, acc=ACC)


def main():
    box_dict = {}
    while rclpy.ok():
        print("choose target")
        print("==========")
        print("new : add new box")
        print("delete : delete box")
        print("grip : go to grip positoin and grip something")
        print("=====")
        for box in box_dict.values():
            print(box.info())
        user_input = input(">> ")
        print()

        if user_input == "new":
            print("input box id")
            box_id = input(">> ")
            print()
            print("input position id")
            position_id = int(input(">> "))
            print()
            # print("input target position(tool position)")
            # print("ex) [637.29, -30.07, 488.51, 175.65, -163.45, 178.82]")
            # target_position = input(">> ")
            if position_id == 1:
                target_position = position_A
            elif position_id == 2:
                target_position = position_B
            elif position_id == 3:
                target_position = position_C
            print()
            box_dict[box_id] = Box(box_id, position_id, eval(target_position))
            print("box is saved!")
        elif user_input == "delete":
            print("input box id")
            box_id = input(">> ")
            print()
            if box_id not in box_dict.keys():
                print("id is not matched : ", box_id)
                continue
            print(f"do you want to delete {box_id}?")
            print("1 : continue\n2 : cancel")
            answer = input(">> ")
            print()
            if answer == "2":
                print("delete is canceled")
                continue
            box_dict.pop(box_id)
            print(f"{box_id} is deleted")
        elif user_input == "grip":
            to_grip()
        else:
            if user_input not in box_dict.keys():
                print("id is not matched : ", user_input)
                continue
            target = box_dict[user_input]
            print("1 : stack")
            print("2 : unstack")
            user_input = input(">> ")
            print()
            if user_input == "1":
                target.stack()
            elif user_input == "2":
                target.unstack()
            else:
                print("----- invalid option -----")

    rclpy.shutdown()

if __name__ == "__main__":
    main()