#!/usr/bin/env python3
from pathlib import Path

from ultralytics import YOLO

_YOLO_DIR = Path(__file__).resolve().parent
model = YOLO("best-v1.pt")
# Export the model to TensorRT with DLA enabled (only works with FP16 or INT8)
model.export(format="engine", device="dla:0", half=True)  # dla:0 or dla:1 corresponds to the DLA cores

# Load the exported TensorRT model
trt_model = YOLO("best-v1.engine")

# Run inference
results = trt_model(str(_YOLO_DIR / "bus.jpg"))