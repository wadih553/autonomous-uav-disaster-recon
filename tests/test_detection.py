import os
import sys
import tempfile
import unittest
from unittest.mock import patch

SERVER_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "ground_station", "server")
)
sys.path.insert(0, SERVER_DIR)

import detection


class DetectionPipelineTests(unittest.TestCase):
    def test_missing_weights_leave_models_disabled(self):
        with patch.object(detection, "HUMAN_MODEL_WEIGHTS", "/no/such/human.pt"), \
             patch.object(detection, "FIRE_MODEL_WEIGHTS", ""):
            pipeline = detection.DetectionPipeline()
        self.assertIsNone(pipeline.human_model)
        self.assertIsNone(pipeline.fire_model)

    def test_fire_model_is_disabled_unless_explicitly_configured(self):
        with patch.object(detection, "HUMAN_MODEL_WEIGHTS", ""), \
             patch.object(detection, "FIRE_MODEL_WEIGHTS", ""):
            pipeline = detection.DetectionPipeline()
        self.assertIsNone(pipeline.fire_model)

    def test_configured_existing_weights_are_passed_to_yolo(self):
        class FakeYOLO:
            def __init__(self, path):
                self.path = path

        with tempfile.NamedTemporaryFile(suffix=".pt") as weights, \
             patch.object(detection, "YOLO", FakeYOLO), \
             patch.object(detection, "HUMAN_MODEL_WEIGHTS", weights.name), \
             patch.object(detection, "FIRE_MODEL_WEIGHTS", ""):
            pipeline = detection.DetectionPipeline()
        self.assertEqual(pipeline.human_model.path, weights.name)
        self.assertIsNone(pipeline.fire_model)

    def test_invalid_display_mode_is_ignored(self):
        with patch.object(detection, "HUMAN_MODEL_WEIGHTS", ""), \
             patch.object(detection, "FIRE_MODEL_WEIGHTS", ""):
            pipeline = detection.DetectionPipeline()
        pipeline.set_display_mode("not-a-mode")
        self.assertEqual(pipeline.display_mode, "live")


if __name__ == "__main__":
    unittest.main()
