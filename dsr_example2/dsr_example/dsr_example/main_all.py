import rclpy
from rclpy.executors import MultiThreadedExecutor
from dsr_example.car_detector import CarDetector
from dsr_example.fuel_task_manager import FuelTaskManager

import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

def main(args=None):
    rclpy.init(args=args)

    # ✅ 1. ROS2 노드 한 번 생성
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)

    # ✅ 2. DR_init에 등록 (DSR_ROBOT2가 쓸 노드 핸들)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = dsr_node

    # ✅ 3. 이제 DSR_ROBOT2 import (여기서 g_node를 올바르게 초기화함)
    import DSR_ROBOT2

    # ✅ 4. 두 노드 생성 (이제 DR_init 설정이 살아있음)
    car_detector_node = CarDetector()
    fuel_task_node = FuelTaskManager()

    executor = MultiThreadedExecutor()
    executor.add_node(car_detector_node)
    executor.add_node(fuel_task_node)

    car_detector_node.get_logger().info("🚀 CarDetector + FuelTaskManager 통합 실행 시작")
    fuel_task_node.get_logger().info("⛽ 두산 로봇 제어 시퀀스 활성화")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        car_detector_node.destroy_node()
        fuel_task_node.destroy_node()
        dsr_node.destroy_node()        # ✅ 추가: 우리가 만든 노드도 같이 해제
        rclpy.shutdown()

if __name__ == '__main__':
    main()
