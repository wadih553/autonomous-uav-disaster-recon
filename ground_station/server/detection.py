#!/usr/bin/env python3
"""
Optional ground-station video perception pipeline.

The final-year report describes human and fire/smoke detection on the
ground-station computer. This module expects model weights at runtime:
  - models/yolov8n.pt for YOLO person detection (Ultralytics may also fetch
    standard pretrained weights depending on the environment).
  - models/fire_smoke_cnn.pt for the project's fire/smoke detector.

The project-specific fire/smoke weights are not included in this repository.
Consequently, this code alone does not demonstrate a working detector or
reproduce any accuracy figure. The report's detection claims are not treated
as independently benchmarked by this repository.
"""

import base64

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    YOLO = None

HUMAN_MODEL_WEIGHTS = 'models/yolov8n.pt'
FIRE_MODEL_WEIGHTS = 'models/fire_smoke_cnn.pt'
PERSON_CLASS_ID = 0
CONFIDENCE_THRESHOLD = 0.45


class DetectionPipeline:
    def __init__(self):
        self.display_mode = 'live'  # 'live' | 'human' | 'fire'
        self.human_model = None
        self.fire_model = None
        self._load_models()

    def _load_models(self):
        if YOLO is None:
            print('[detection] ultralytics not installed - detection disabled')
            return
        try:
            self.human_model = YOLO(HUMAN_MODEL_WEIGHTS)
        except Exception as e:
            print(f'[detection] failed to load human detection model: {e}')

        try:
            self.fire_model = YOLO(FIRE_MODEL_WEIGHTS)
        except Exception as e:
            print(f'[detection] failed to load fire/smoke model: {e}')

    def set_display_mode(self, mode: str):
        if mode in ('live', 'human', 'fire'):
            self.display_mode = mode

    def process_frame_b64(self, frame_b64: str) -> dict:
        """Decode a base64 JPEG frame and return normalized detection boxes."""
        if cv2 is None:
            return {'human_boxes': [], 'fire_boxes': []}

        try:
            jpg_bytes = base64.b64decode(frame_b64)
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return {'human_boxes': [], 'fire_boxes': []}

        if frame is None:
            return {'human_boxes': [], 'fire_boxes': []}

        h, w = frame.shape[:2]
        human_boxes = self._run_human_detection(frame, w, h)
        fire_boxes = self._run_fire_detection(frame, w, h)

        return {'human_boxes': human_boxes, 'fire_boxes': fire_boxes}

    def _run_human_detection(self, frame, frame_w, frame_h):
        if self.human_model is None:
            return []
        results = self.human_model.predict(
            frame, classes=[PERSON_CLASS_ID], conf=CONFIDENCE_THRESHOLD, verbose=False
        )
        return self._extract_boxes(results, frame_w, frame_h, label='person')

    def _run_fire_detection(self, frame, frame_w, frame_h):
        if self.fire_model is None:
            return []
        results = self.fire_model.predict(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)
        return self._extract_boxes(results, frame_w, frame_h, label='fire')

    def _extract_boxes(self, results, frame_w, frame_h, label):
        boxes = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                boxes.append({
                    'label': label,
                    'x': x1 / frame_w,
                    'y': y1 / frame_h,
                    'w': (x2 - x1) / frame_w,
                    'h': (y2 - y1) / frame_h,
                    'confidence': round(conf, 3),
                })
        return boxes
