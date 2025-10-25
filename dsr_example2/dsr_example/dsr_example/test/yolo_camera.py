from ultralytics import YOLO
import cv2

# YOLOv8 모델 로드 (가볍고 빠른 버전)
model = YOLO("yolov8n.pt")

# 내장 카메라 열기 (기본 장치: 0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ 카메라를 열 수 없습니다. 장치 번호를 확인하세요 (0, 1 등).")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 가져올 수 없습니다.")
        break

    # YOLO 추론
    results = model(frame, verbose=False)

    # 결과 시각화 (bounding box 등)
    annotated_frame = results[0].plot()

    # 영상 출력
    cv2.imshow("YOLO Real-Time Detection", annotated_frame)

    # ESC(27) 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
