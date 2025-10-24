import cv2
import torch
import numpy as np
from ultralytics import YOLO  # pip install ultralytics

class YoloDetector:
    def __init__(self, model_path="yolov8n.pt", conf=0.6):
        self.model = YOLO(model_path)
        self.conf = conf

    def detect(self, frame):
        results = self.model.predict(frame, conf=self.conf, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                conf = float(box.conf)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    "cls": self.model.names[cls_id],
                    "conf": conf,
                    "bbox": (x1, y1, x2, y2)
                })
        return detections
