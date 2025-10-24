import cv2
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time
import math

from std_msgs.msg import String
import json

import threading
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
import message_filters

from enum import Enum, auto

import DR_init
from dsr_example.gripper_drl_controller import GripperController
from dsr_example.yolo_manager import YoloDetector

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
        self.robot_state = RobotState.IDLE
        self.get_logger().info("🦾 로봇 제어 노드 초기화 중...")

        # --- Gripper 초기화 ---
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

        # --- YOLO 객체 인식기 생성 ---
        self.get_logger().info("YOLO 객체 인식기 생성")
        self.create_subscription(String, '/yolo/detections', self.yolo_callback, 10)

    def terminate_gripper(self):
        if self.gripper:
            try:
                print("🧹 Gripper 연결 종료 중...")   # ✅ ROS logger 대신 print 사용
                self.gripper.terminate()
            except Exception as e:
                print(f"⚠️ 그리퍼 종료 중 오류: {e}")

    def yolo_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"YOLO 데이터 파싱 오류: {e}")
            return

        # fuel_cap 감지 시 로봇 동작 실행
        for d in detections:
            if d["cls"] == "fuel_cap" and d["conf"] > 0.7:
                if self.robot_state == RobotState.IDLE:
                    self.get_logger().info("🚀 'fuel_cap' 감지됨 → 로봇 시퀀스 실행")
                    self.robot_state = RobotState.MOVING
                    threading.Thread(target=self.run_robot_sequence, daemon=True).start()
                    break

    #--------------------- 초기화 부분 ---------------------#
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

                time.sleep(2.5)

                # 2) force_off 동작 (예: 열기)
                self.get_logger().info(f"   → move({force_off})")
                result_off = self.gripper.move(force_off)
                if not result_off:
                    self.get_logger().error(f"❌ Gripper move({force_off}) failed at cycle {i+1}")
                    break

                time.sleep(2.5)

            self.get_logger().info(f"✅ Gripper 반복 동작 완료 ({cnt}회 실행)")

        except Exception as e:
            self.get_logger().error(f"Gripper 반복 동작 중 오류 발생: {e}")        

    def run_robot_sequence(self):
        try:
            from DSR_ROBOT2 import get_current_posx, movel, wait, movej, DR_MV_MOD_REL
            from DR_common2 import posx, posj
        except ImportError as e:
            print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
            rclpy.shutdown()
            exit(1)

        # 로봇 위치, 그리퍼 초기화
        self.robot_init()

        #--------------------- 차량 진입 후 작업 시작 ---------------------#
        # 주유구 위치로 이동 
        movej(g_fuel_posj, 80, 80)
        wait(3.0)

        # 주유구 뚜껑 잡으러 이동 -> 오픈을 위한 그리퍼 회전
        movel(posx(-5, -38, -25, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.rotate_grip(3)

        self.grip_init()
        # 주유건 위치로 이동 후 그리퍼 닫기
        movej(g_diesel_posj, 80, 80)
        wait(2.0)
        movel(posx(0, 40, 0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        self.gripper.move(grip_gun)
        wait(2.5)
        self.set_robot_state(RobotState.FUEL_READY)

        # 주유건 뽑기
        movel(posx(-35, 0, 120, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        movel(posx(0, -70, 0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        #--------------------- 직접 주유 작업 시작 ---------------------#
        # 주유구 위치로 이동 
        movej(g_fuel_posj, 80, 80)
        wait(3.0)

        # 주유건 넣기
        # movel(posx(0, -60, -50, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.run_fuel_task(grip_shot, grip_gun, 5)

        # movel(posx(0, 0, -100, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        # wait(1.0)
        # fuel_controller.check_crash()

        # fuel_controller.rotate_grip(3)
        self.robot_init()
    
def main(args=None):
    # ✅ 1️⃣ ROS 초기화 먼저
    rclpy.init(args=args)

    # ✅ 2️⃣ 노드 생성 순서 정리
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    # ✅ 3️⃣ FuelTaskManager 생성 (이제 Node 생성 가능)
    fuel_controller = FuelTaskManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(fuel_controller, timeout_sec=0.05)
            
            # 특정 조건 (예: 입력, 토픽, 상태 변화)에 따라 실행
            if fuel_controller.robot_state == RobotState.IDLE:
                fuel_controller.run_robot_sequence()

    except KeyboardInterrupt:
        print("🛑 Keyboard Interrupt 감지됨, 로봇 정지 중...")
        fuel_controller.terminate_gripper() 
        pass

    finally:
        # 1️⃣ 먼저 로봇 동작이 끝났는지 기다림
        while fuel_controller.robot_state == RobotState.MOVING:
            time.sleep(0.1)

        try:
            fuel_controller.destroy_node()
            dsr_node.destroy_node()
        except Exception:
            print("⚠️ Node 종료 중 오류 무시")

        # 3️⃣ ROS context 마지막에 shutdown
        if rclpy.ok():
            rclpy.shutdown()

        print("✅ 종료 완료.")

if __name__ == '__main__':
    main()