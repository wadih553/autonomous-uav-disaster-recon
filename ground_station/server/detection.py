#!/usr/bin/env python3
"""
Optional ground-station video perception pipeline.

Human detection uses Ultralytics YOLO weights when a local weights file is
available. Fire/smoke detection is enabled only when a separate, compatible
Ultralytics detection model is explicitly configured with
UAV_FIRE_YOLO_WEIGHTS. A generic CNN classifier is not interchangeable with
a YOLO object detector and is not loaded by this module.

Model weights are intentionally not bundled. Without configured weights,
the relevant detector remains disabled; no accuracy claim is implied.
"""

import base64
import os
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    YOLO = None

_DEFAULT_HUMAN_WEIGHTS = Path(__file__).resolve().parent / "models" / "yolov8n.pt"
HUMAN_MODEL_WEIGHTS = os.environ.get(
    "UAV_HUMAN_MODEL_WEIGHTS", str(_DEFAULT_HUMAN_WEIGHTS)
)
# Configure this only with weights trained for an Ultralytics-compatible
# object-detection model. The project's fire_smoke_cnn.pt is not assumed to
# be compatible and is therefore not loaded automatically.
FIRE_MODEL_WEIGHTS = os.environ.get("UAV_FIRE_YOLO_WEIGHTS", "").strip()
PERSON_CLASS_ID = 0
CONFIDENCE_THRESHOLD = 0.45


class DetectionPipeline:
    def __init__(self):
        self.display_mode = "live"  # "live" | "human" | "fire"
        self.human_model = None
        self.fire_model = None
        self._load_models()

    def _load_model_if_available(self, weights, label):
        if not weights:
            print(f"[detection] {label} detection disabled: no weights configured")
            return None
        if not Path(weights).is_file():
            print(f"[detection] {label} detection disabled: weights file not found: {weights}")
            return None
        if YOLO is None:
            print(f"[detection] {label} detection disabled: ultralytics is not installed")
            return None
        try:
            return YOLO(weights)
        except Exception as exc:
            print(f"[detection] failed to load {label} detection model: {exc}")
            return None

    def _load_models(self):
        self.human_model = self._load_model_if_available(
            HUMAN_MODEL_WEIGHTS, "human"
        )
        self.fire_model = self._load_model_if_available(
            FIRE_MODEL_WEIGHTS, "fire/smoke"
        )

    def set_display_mode(self, mode: str):
        if mode in ("live", "human", "fire"):
            self.display_mode = mode

    def process_frame_b64(self, frame_b64: str) -> dict:
        """Decode a base64 JPEG frame and return normalized detection boxes."""
        if cv2 is None or np is None:
            return {"human_boxes": [], "fire_boxes": []}

        try:
            jpg_bytes = base64.b64decode(frame_b64, validate=True)
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except (ValueError, TypeError, base64.binascii.Error):
            return {"human_boxes": [], "fire_boxes": []}

        if frame is None:
            return {"human_boxes": [], "fire_boxes": []}

        h, w = frame.shape[:2]
        if w <= 0 or h <= 0:
            return {"human_boxes": [], "fire_boxes": []}

        return {
            "human_boxes": self._run_human_detection(frame, w, h),
            "fire_boxes": self._run_fire_detection(frame, w, h),
        }

    def _run_human_detection(self, frame, frame_w, frame_h):
        if self.human_model is None:
            return []
        try:
            results = self.human_model.predict(
                frame, classes=[PERSON_CLASS_ID],
                conf=CONFIDENCE_THRESHOLD, verbose=False
            )
            return self._extract_boxes(results, frame_w, frame_h, label="person")
        except Exception as exc:
            print(f"[detection] human inference failed: {exc}")
            return []

    def _run_fire_detection(self, frame, frame_w, frame_h):
        if self.fire_model is None:
            return []
        try:
            results = self.fire_model.predict(
                frame, conf=CONFIDENCE_THRESHOLD, verbose=False
            )
            return self._extract_boxes(results, frame_w, frame_h, label="fire")
        except Exception as exc:
            print(f"[detection] fire/smoke inference failed: {exc}")
            return []

    @staticmethod
    def _extract_boxes(results, frame_w, frame_h, label):
        boxes = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                confidence = float(box.conf[0]) if box.conf is not None else 0.0
                boxes.append({
                    "label": label,
                    "x": max(0.0, min(1.0, x1 / frame_w)),
                    "y": max(0.0, min(1.0, y1 / frame_h)),
                    "w": max(0.0, min(1.0, (x2 - x1) / frame_w)),
                    "h": max(0.0, min(1.0, (y2 - y1) / frame_h)),
                    "confidence": round(confidence, 3),
                })
        return boxes
