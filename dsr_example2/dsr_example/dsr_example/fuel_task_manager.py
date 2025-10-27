import cv2
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math

from std_msgs.msg import String
import json

import threading
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
import message_filters

from enum import Enum

import DR_init
from dsr_example.gripper_drl_controller import GripperController

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dsr_example.realsense_manager import RealSenseManager
from dsr_example.yolo_manager import YoloDetector

ROBOT_STATE = Enum('ROBOT_STATE',
                   ['IDLE',
                    'MOVE_TO_FUEL_POS',
                    'APPROACH_FUEL_NOZZLE',
                    'GRIP_NOZZLE',
                    'LIFT_NOZZLE',
                    'MOVE_TO_CAR_FUEL_PORT',
                    'FUELING',
                    'RETURN_NOZZLE',
                    'RELEASE_NOZZLE',
                    'MOVE_TO_HOME_POS'])

CAR_TYPE = Enum('CAR_TYPE',
                ['orange_car',
                 'green_car',
                 'yellow_car'])

detected_car_list = []

VELOCITY, ACC = 70, 70

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

g_vel_move = 80
g_vel_rotate = 120

g_force_lift = 20.0

# 첫 주유건 위치 : 더미 좌표
g_oil1_ready_posj = [-10, 63, 32, 91, 85, -104]
g_oil2_ready_posj = [-14, 65, 48, 87, 86, -123]

# 주유구 위치 : 더미 좌표
g_car1_posj = [-13, 33, 82, -52, 58, 40]
g_car2_posj = [500, 0, 300, 0, 0, 0]
# 주유시작 위치 : 더미 좌표
g_fuel_car1_posj = [-52, 34, 81, 61, 83, -135]
g_fuel_car2_posj = [500, 0, 300, 0, 0, 0]

# 주유 완료후 주유건 위치 : 더미 좌표
g_oil1_go_posj = [-9, 68, 22, 91, 88, -88]
g_oil2_go_posj = [-14, 65, 48, 87, 86, -123]
g_oil1_end_posj = [-9, 68, 22, 91, 88, -88]
g_oil2_end_posj = [-14, 65, 48, 87, 86, -123]

grip_shot = 440
grip_gun = 230

g_Cap_Grip_Off = 440
g_Cap_Grip_On = 580

class FuelTaskManager(Node):
    def __init__(self):
        super().__init__("fuel_task_manager")
        self.get_logger().info("🦾 로봇 제어 노드 초기화 중...")

        # ✅ /fuel_task/start 구독 주유결제 명령 수신
        self.subscription = self.create_subscription(
            String,
            '/fuel_task/start',
            self.on_task_start,
            10)
        self.status_pub = self.create_publisher(String, '/fuel_status', 10)
        self.get_logger().info("🦾 FuelTaskManager started — waiting for /fuel_task/start")

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
        self.bridge = CvBridge()
        self.realsense = RealSenseManager(self)
        self.yolo = YoloDetector()

        # 🔹 YOLO 결과 영상 퍼블리시 (rqt에서 구독)
        self.pub_result = self.create_publisher(Image, '/fuel/image_result', 10)
        # self.pub_info = self.create_publisher(String, '/fuel/yolo_info', 10)

        # 20Hz 루프
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("📸 FuelTaskManager with YOLO initialized")

        self.current_state = ROBOT_STATE.IDLE

    def timer_callback(self):
        color_frame, depth_frame = self.realsense.get_latest_frames()
        if color_frame is None:
            # self.get_logger().warn("⚠️ RealSense frame not received")
            return

        detections = self.yolo.detect(color_frame)
        annotated = self.yolo.draw_detections(color_frame, detections)

        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.pub_result.publish(img_msg)

        if not detections:
            self.tracked_objects = {}  # 감지 안될 때 리셋
            return
        
        current_time = time.time()
        
        # self.get_logger().info(f"🎯 Detected: {[(d['cls'], round(d['conf'],2)) for d in detections]}")

        # msg = String()
        # msg.data = "Detected: " + ", ".join([d['cls'] for d in detections])
        # self.pub_info.publish(msg)
        # self.get_logger().info(msg.data)

        # ✅ YOLO 감지된 객체 순회
        for det in detections:
            cls = det["cls"]
            x1, y1, x2, y2 = det["bbox"]
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            # 이전 정보 가져오기
            if not hasattr(self, "tracked_objects"):
                self.tracked_objects = {}

            if cls not in self.tracked_objects:
                # 첫 감지
                self.tracked_objects[cls] = {
                    "cx": cx,
                    "cy": cy,
                    "start_time": current_time,
                }
            else:
                prev = self.tracked_objects[cls]
                dist = math.sqrt((cx - prev["cx"])**2 + (cy - prev["cy"])**2)
                elapsed = current_time - prev["start_time"]

                # ✅ 움직임이 거의 없고 3초 이상 지속되면 정지로 판단
                if dist < 10 and elapsed >= 3.0:  # 픽셀 단위로 약간의 오차 허용
                    # self.get_logger().info(f"🟩 {cls} 정지 상태로 판단됨 (3초 이상 고정)")

                    # 👉 여기서 로봇 시퀀스 실행 가능
                    detected_car_list.append(det)

                    # 중복 실행 방지: start_time 갱신
                    self.tracked_objects[cls]["start_time"] = current_time

                # 업데이트
                self.tracked_objects[cls]["cx"] = cx
                self.tracked_objects[cls]["cy"] = cy


    def terminate_gripper(self):
        if self.gripper:
            try:
                print("🧹 Gripper 연결 종료 중...")
                if rclpy.ok():
                    self.gripper.terminate()
                    print("✅ Gripper 종료 완료")
                else:
                    print("⚠️ ROS context 종료됨 — terminate() 생략")
            except Exception as e:
                print(f"⚠️ 그리퍼 종료 중 오류: {e}")

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

    #--------------------- Set Position Callback ---------------------#
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
            wait(1.5)
            count = count + 1

            if count < cnt:
                self.gripper.move(g_Cap_Grip_Off)
                wait(1.5)
                movej(posj(0, 0, 0, 0, 0, 120), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
                wait(1.5)

        movel(posx(0, 0, 79, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.gripper.move(0)
        wait(1.5)

    # 반복적으로 그리퍼를 열고 닫는 작업을 수행 : 주유 시작       
    def run_fuel_task(self, cnt):
        try:
            for i in range(cnt):
                self.get_logger().info(f"[Cycle {i+1}/{cnt}] 🔹 Gripper close → open")

                # 1) force_on 동작 (예: 닫기)
                self.get_logger().info(f"   → move({grip_shot})")
                result_on = self.gripper.move(grip_shot)
                if not result_on:
                    self.get_logger().error(f"❌ Gripper move({grip_shot}) failed at cycle {i+1}")
                    break

                time.sleep(2.5)

                # 2) force_off 동작 (예: 열기)
                self.get_logger().info(f"   → move({grip_gun})")
                result_off = self.gripper.move(grip_gun)
                if not result_off:
                    self.get_logger().error(f"❌ Gripper move({grip_gun}) failed at cycle {i+1}")
                    break

                time.sleep(2.5)

            self.get_logger().info(f"✅ Gripper 반복 동작 완료 ({cnt}회 실행)")

        except Exception as e:
            self.get_logger().error(f"Gripper 반복 동작 중 오류 발생: {e}")  
    
    # --------------------- Subscribe Server ---------------------#
    def on_task_start(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid message: {e}")
            return

        fuel_type = data.get("fuelType")
        amount = data.get("amount")
        order_id = data.get("orderId")

        self.get_logger().info(f"🚀 Starting fueling task for {fuel_type}, {amount}원 (Order {order_id})")

        # 실제 주유 로직 수행 ...
        self.current_state = ROBOT_STATE.MOVE_TO_FUEL_POS
        self.status_pub.publish(String(data="in_progress"))

        # 실제 로봇 주유 시퀀스 로직 연결
        self.execute_fuel_task(fuel_type, amount)

    def execute_fuel_task(self, fuel_type, amount):
        self.get_logger().info(f"🛠️ Executing robot motion for {fuel_type} / {amount}원 ...")
        # TODO: 여기에 로봇 제어 코드 삽입 (movel, 그리퍼, force control 등)

        # 유종별 주유량 로직 예시
        if fuel_type == "휘발유":
            self.start_gasoline_fuel(amount)
        elif fuel_type == "경유":
            self.start_diesel_fuel(amount)
        else:
            self.get_logger().warn(f"Unknown fuel type: {fuel_type}")

    def start_gasoline_fuel(self, amount):
        try:
            from DSR_ROBOT2 import get_current_posj, movel, wait, movej, DR_MV_MOD_REL
            from DR_common2 import posx, posj
        except ImportError as e:
            print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
            rclpy.shutdown()
            exit(1)

        # 🔧 실제 로봇 주유 동작 시퀀스 작성
        self.get_logger().info(f"⛽ Gasoline fueling sequence for {amount}원 started...")
        
        if amount < 30000:
            amount = 30000 # 최소 1회 주유
        
        m_count = amount // 30000  # 30000원 단위로 주유 횟수 결정

        gun_posj = get_current_posj()

        # 로봇 위치, 그리퍼 초기화
        self.robot_init()

        #--------------------- 차량 진입 후 작업 시작 ---------------------#
        # 주유구 위치로 이동 
        movej(g_car1_posj, 80, 80)
        wait(2.0)

        # # 주유구 뚜껑 잡으러 이동 -> 오픈을 위한 그리퍼 회전
        movel(posx(-5, -38, -25, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        self.rotate_grip(3)
        
        movel(posx(0, 180, -120, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(3.0)
        self.grip_init()

        # 주유건 위치로 이동 후 그리퍼 닫기
        movej(g_oil1_ready_posj, 80, 80)
        wait(2.0)
        movel(posx(12, 60, -0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        g_oil1_end_posj = get_current_posj()
        self.gripper.move(grip_gun)
        wait(2.5)

        # 주유건 그립 이후 주유건 뽑아 가기
        movel(posx(-35, 0, 120, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)
        g_oil1_go_posj = get_current_posj()
        movel(posx(0, -70, 0, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        #--------------------- 직접 주유 작업 시작 ---------------------#
        # 주유구 위치로 이동 
        movej(g_fuel_car1_posj, 80, 80)
        wait(3.0)
        movel(posx(-10, -90, -80, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        # 주유 작업 반복 수행
        self.run_fuel_task(m_count)
        
        movel(posx(0, 100, 90, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        wait(2.0)

        movej(g_oil1_go_posj, 80, 80)
        wait(2.0)
        movel(posx(10, 0, -120, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        # movej(g_oil1_end_posj, 80, 80)
        wait(2.0)

        self.grip_init()
        wait(2.0)

        movej(g_oil1_ready_posj, 80, 80)
        wait(2.0)
        # movel(posx(0, 0, -100, 0, 0, 0), v=g_vel_move, a=g_vel_move, mod=DR_MV_MOD_REL)
        # wait(1.0)
        # fuel_controller.check_crash()

        # fuel_controller.rotate_grip(3)
        self.robot_init()

        # 주유 완료 시:
        self.current_state = ROBOT_STATE.IDLE
        self.status_pub.publish(String(data="completed"))
        self.get_logger().info("✅ Fueling completed.")
    
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
            for d in detected_car_list:
                car_type = d['cls']
                fuel_controller.get_logger().info(f"🟩 {car_type} 주유 시작")
                if car_type == 'orange_car' and fuel_controller.current_state == ROBOT_STATE.IDLE:
                    fuel_controller.get_logger().info(f"🟩 {car_type} 주유 시작")
                    # fuel_controller.run_robot_sequence()
                    # fuel_controller.current_state = ROBOT_STATE.MOVE_TO_FUEL_POS
                # elif car_type == 'yellow_car' and self.current_state == ROBOT_STATE.IDLE:
                #     fuel_controller.run_robot_sequence()
                #     self.current_state = ROBOT_STATE.MOVE_TO_FUEL_POS
            
    except KeyboardInterrupt:
        print("🛑 Keyboard Interrupt 감지됨, 로봇 정지 중...")
        fuel_controller.terminate_gripper() 
        pass

    finally:
        try:
            fuel_controller.terminate_gripper()
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