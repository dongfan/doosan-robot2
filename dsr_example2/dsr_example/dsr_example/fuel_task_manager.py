import cv2
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time
import math

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import DR_init
from dsr_example.gripper_drl_controller import GripperController

from enum import Enum, auto

class RobotState(Enum):
    IDLE = auto()
    MOVING = auto()
    ERROR = auto()
    FUEL_READY = auto()
    PAUSED = auto()

VELOCITY, ACC = 70, 70

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

g_vel_move = 80
g_vel_rotate = 120

g_force_lift = 20.0

# 주유건 위치
g_diesel_posj = [-9, 68, 22, 91, 88, -88]
g_gasoline_posj = [-14, 65, 48, 87, 86, -123]

# 주유구 위치
g_fuel_posj = [-13, 33, 82, -52, 58, 40]
g_fuel2_posj = [500, 0, 300, 0, 0, 0]

grip_shot = 440
grip_gun = 200

g_Cap_Grip_Off = 420
g_Cap_Grip_On = 580

class FuelTaskManager(Node):
    def __init__(self):
        super().__init__("fuel_controller_node")

        self.bridge = CvBridge()

        self.get_logger().info("ROS 2 구독자 설정을 시작합니다...")

        self.gripper = None
        try:
            from DSR_ROBOT2 import wait
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)

            if not self.gripper.initialize():
                self.get_logger().error("Gripper initialization failed. Exiting.")
                raise Exception("Gripper initialization failed")
            
            self.get_logger().info("그리퍼를 활성화합니다...")
            self.gripper_is_open = True
            self.gripper.move(0)
            wait(2)
            
        except Exception as e:
            self.get_logger().error(f"An error occurred during gripper setup: {e}")
            rclpy.shutdown()

        self.get_logger().info("RealSense ROS 2 구독자와 로봇 컨트롤러가 초기화되었습니다.")

    def robot_init(self):
        self.pos_init()
        self.grip_init()

        self.get_logger().info("Robot 초기화 완료.")

    def pos_init(self):
        from DSR_ROBOT2 import movej, posj, wait
        p_start = posj(0, 0, 90, 0, 90, 0)
        movej(p_start, VELOCITY, ACC)
        wait(3)

    def grip_init(self):
        from DSR_ROBOT2 import wait
        self.gripper.move(0)
        wait(3)

    def terminate_gripper(self):
        if self.gripper:
            self.gripper.terminate()

    def set_pos_callback(self, event, u, v, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.latest_cv_depth_mm is None or self.intrinsics is None:
                self.get_logger().warn("아직 뎁스 프레임 또는 카메라 정보가 수신되지 않았습니다.")
                return

            try:
                depth_mm = self.latest_cv_depth_mm[v, u]
            except IndexError:
                self.get_logger().warn(f"클릭 좌표(u={u}, v={v})가 이미지 범위를 벗어났습니다.")
                return
            
            if depth_mm == 0:
                print(f"({u}, {v}) 지점의 깊이를 측정할 수 없습니다 (값: 0).")
                return

            # 픽셀 좌표와 깊이 값을 사용하여 3D 좌표 계산
            depth_m = float(depth_mm) / 1000.0

            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)

            x_mm = point_3d[1] * 1000
            y_mm = point_3d[0] * 1000
            z_mm = point_3d[2] * 1000

            final_x = 635 + x_mm - 20
            final_y = y_mm
            final_z = 970 - z_mm + 140
            if(final_z <= 150):
                final_z = 150

            if(final_x <= 200):
                final_x = 200

            print("--- 변환된 최종 3D 좌표 ---")
            print(f"픽셀 좌표: (u={u}, v={v}), Depth: {depth_m*1000:.1f} mm")
            print(f"로봇 목표 좌표: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}\n")

            self.move_robot_and_control_gripper(final_x, final_y, final_z, g_Cap_Grip_Off)
            print("=" * 50)

    # dongfan
    def set_robot_state(self, state: RobotState):
        self.robot_state = state
        self.get_logger().info(f"Robot state updated to: {state.name}")

    # 주유건이 충돌했는지 확인하고 대응하는 함수
    def check_crash(self):
        from DSR_ROBOT2 import (task_compliance_ctrl, set_desired_force, get_tool_force,
            release_force, release_compliance_ctrl, amovel, wait, DR_MV_MOD_REL)
        from DR_common2 import posx
        
        k_d = [500.0, 500.0, 500.0, 200.0, 200.0, 200.0]
        task_compliance_ctrl(k_d)
        # 강성 제어
        f_d = [0.0, 0.0, -20, 0.0, 0.0, 0.0]
        f_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(f_d, f_dir)
        wait(2.0)

        # 외력감지
        while True:
            force_ext = get_tool_force()
            # c_pos = get_current_posx()
            # x, y, z = c_pos[0]
            if force_ext[2] > 4:
                release_force()
                release_compliance_ctrl()

                self.gripper.move(g_Cap_Grip_Off)
                wait(1.0)
                amovel(posx(0, 0, 79, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
                wait(1.0)
                break
    
    # 주유구를 오픈하기 위해 그리퍼를 회전시키는 함수
    def rotate_grip(self, cnt):
        from DSR_ROBOT2 import (amovel, DR_MV_MOD_REL,
            movel, movej, wait)
        from DR_common2 import posx, posj
        count = 0

        while count < cnt :
            self.gripper.move(g_Cap_Grip_On)
            wait(2.5)
            
            movej(posj(0, 0, 0, 0, 0, -120), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
            wait(2.0)
            count = count + 1

            if count < cnt:
                self.gripper.move(g_Cap_Grip_Off)
                wait(1.5)
                movej(posj(0, 0, 0, 0, 0, 120), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
                wait(2.0)

        movel(posx(0, 0, 79, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.gripper.move(0)
        wait(1.5)


    # 반복적으로 그리퍼를 열고 닫는 작업을 수행 : 주유 시작       
    def run_fuel_task(self, force_on, force_off, cnt):
        try:
            for i in range(cnt):
                self.get_logger().info(f"[Cycle {i+1}/{cnt}] 🔹 Gripper close → open")

                # 1) force_on 동작 (예: 닫기)
                self.get_logger().info(f"   → move({force_on})")
                result_on = self.gripper.move(force_on)
                if not result_on:
                    self.get_logger().error(f"❌ Gripper move({force_on}) failed at cycle {i+1}")
                    break

                import time
                start = time.monotonic()
                while time.monotonic() - start < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.1)

                # 2) force_off 동작 (예: 열기)
                self.get_logger().info(f"   → move({force_off})")
                result_off = self.gripper.move(force_off)
                if not result_off:
                    self.get_logger().error(f"❌ Gripper move({force_off}) failed at cycle {i+1}")
                    break

                start = time.monotonic()
                while time.monotonic() - start < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info(f"✅ Gripper 반복 동작 완료 ({cnt}회 실행)")

        except Exception as e:
            self.get_logger().error(f"Gripper 반복 동작 중 오류 발생: {e}")        

def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej, DR_MV_MOD_REL
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
        rclpy.shutdown()
        exit(1)

    fuel_controller = FuelTaskManager()
    fuel_controller.robot_init()
    
    # P0 = posj(0,0,90,0,90,0)
    # print("홈 위치로 이동합니다")
    # movej(P0, 100, 80)
    # wait(10.0)
    
    # 주유구 위치로 이동 
    movej(g_fuel_posj, 80, 80)
    wait(3.0)

    # 주유구 뚜껑 잡으러 이동 -> 오픈을 위한 그리퍼 회전
    movel(posx(-5, -38, -25, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    fuel_controller.rotate_grip(3)

    fuel_controller.grip_init()
    # 주유건 위치로 이동 후 그리퍼 닫기
    movej(g_diesel_posj, 80, 80)
    wait(2.0)
    movel(posx(0, 40, 0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    wait(2.0)
    fuel_controller.gripper.move(grip_gun)
    wait(2.5)
    fuel_controller.set_robot_state(RobotState.FUEL_READY)

    # 주유건 뽑기
    movel(posx(-35, 0, 120, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    wait(2.0)
    movel(posx(0, -70, 0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    wait(2.0)

    #--------------------- 주유 작업 시작 ---------------------#
    # 주유구 위치로 이동 
    movej(g_fuel_posj, 80, 80)
    wait(3.0)

    # 주유건 넣기
    # movel(posx(0, -60, -50, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    fuel_controller.run_fuel_task(grip_shot, grip_gun, 5)

    # movel(posx(0, 0, -100, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
    # wait(1.0)
    # fuel_controller.check_crash()

    # fuel_controller.rotate_grip(3)

    while rclpy.ok():
        rclpy.spin_once(fuel_controller, timeout_sec=0.001)
        rclpy.spin_once(dsr_node, timeout_sec=0.001)
    
    # finally:
    print("프로그램을 종료합니다...")
    fuel_controller.terminate_gripper()
    # cv2.destroyAllWindows()
    fuel_controller.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()
    print("종료 완료.")

if __name__ == '__main__':
    main()