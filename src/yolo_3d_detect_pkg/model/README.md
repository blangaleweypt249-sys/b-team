# Model Directory

This package intentionally contains no trained weights.

To use the default configuration, copy a newly trained Ultralytics YOLO model
to this directory as `detector.pt`, then rebuild the workspace:

```bash
colcon build --packages-select yolo_3d_detect_pkg
```

Alternatively, set the `block_detector.model_path` parameter to the absolute
path of a `.pt`, `.onnx`, or compatible TensorRT engine file. Set
`box_class_id` to the class index for the target that the new model detects.