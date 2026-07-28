# ax-ros2-ref — Model Swapping Guide

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

## Contents

- [How the Model Pipeline Works](#how-the-model-pipeline-works)
- [Available Models](#available-models)
- [Compiling a New Model](#compiling-a-new-model)
- [Swapping the Model in ax-ros2-ref](#swapping-the-model-in-ax-ros2-ref)
- [Normalization Reference](#normalization-reference)
- [Using a Custom Model](#using-a-custom-model)
- [Verifying a New Model Works](#verifying-a-new-model-works)

---

## How the Model Pipeline Works

Every model deployed on Metis consists of two files that must exist side-by-side in the
same directory:

| File | Purpose |
| :--- | :------ |
| `model.json` | The compiled model graph for the Metis AIPU. Contains the quantized weights and the RISC-V kernel schedule. **Architecture-specific** — a YOLOv5 `model.json` cannot run a YOLOv8 graph. |
| `postprocess_graph.onnx` | An ONNX graph that decodes raw INT8 AIPU outputs into bounding boxes. Dequantizes tensors, applies NMS, and maps class indices to labels. **Also architecture-specific.** |

The [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk) compiler produces both files
during the deployment step. Compiled models are stored under:

```
~/voyager-sdk/build/<MODEL_NAME>/<MODEL_NAME>/<N>/
```

where `<N>` is the number of AIPU cores compiled for (typically `4` for the full Metis M.2).

The `metis_ros2_inference` node reads the directory path from the `model_dir` ROS2 parameter.
The label file path comes from `labels_path`. No recompilation of the reference implementation is ever needed when
swapping models — only the launch file parameters change.

---

## Available Models

The full list of supported models is in the
[Voyager SDK model zoo](https://github.com/axelera-ai-hub/voyager-sdk/tree/main/ax_models/zoo).

The following models are known to work end-to-end with this reference implementation:

| Model | Task | Input size | Classes | `normalize` | Notes |
| :---- | :--- | :--------- | :------ | :---------- | :---- |
| `yolov5s-v7-coco` | Object detection | 640×640 | 80 (COCO) | `false` | **Default** — fastest, lowest memory |
| `yolov5m-v7-coco` | Object detection | 640×640 | 80 (COCO) | `false` | Better accuracy than `s` |
| `yolov5l-v7-coco` | Object detection | 640×640 | 80 (COCO) | `false` | Best accuracy, higher latency |
| `yolox-s-coco` | Object detection | 640×640 | 80 (COCO) | `true` | ImageNet mean/stddev required |
| `yolox-m-coco` | Object detection | 640×640 | 80 (COCO) | `true` | ImageNet mean/stddev required |

> 💡 **Tip:** Start with `yolov5s-v7-coco`. It is the fastest model in the zoo and is already
> compiled for 4 AIPU cores by the Voyager SDK installer.

---

## Compiling a New Model

If the model you want is not already in `~/voyager-sdk/build/`, compile it with the Voyager
SDK deployment tool. Compilation is a one-time step per model — the result is cached.

```bash
cd ~/voyager-sdk
source venv/bin/activate

# Example: compile YOLOv5m for 4 AIPU cores
python3 deploy.py ax_models/zoo/yolo/object_detection/yolov5m-v7-coco-onnx.yaml \
  --target metis \
  --num-cores 4
```

<details>
<summary>What happens during compilation</summary>

The Voyager SDK compiler:

1. Loads the ONNX model from the zoo YAML
2. Quantizes weights to INT8 using the calibration dataset specified in the YAML
3. Generates RISC-V kernel code for the on-device schedule
4. Writes `model.json` (AIPU graph) and `postprocess_graph.onnx` (CPU post-processing)
   to `build/<MODEL_NAME>/<MODEL_NAME>/4/`

Compilation can take 5–30 minutes depending on model size. The result is fully reusable —
you only compile once per model per core count.

</details>

After compilation, verify the output directory:

```bash
ls ~/voyager-sdk/build/yolov5m-v7-coco/yolov5m-v7-coco/4/
# model.json  postprocess_graph.onnx
```

---

## Swapping the Model in ax-ros2-ref

Only **one file** needs to change: the launch file. No C++ code is touched.

Open `ros2_ws/src/metis_ros2_inference/launch/metis_inference.launch.py`:

```python
MODEL_NAME = 'yolov5s-v7-coco'   # ← change this line
MODEL_DIR  = os.path.join(VOYAGER_SDK, 'build', MODEL_NAME, MODEL_NAME, '4')
LABELS     = os.path.join(VOYAGER_SDK, 'ax_datasets', 'labels', 'coco.names')
```

Change `MODEL_NAME` and rebuild:

```bash
source ~/ax-ros2-ref/workspace_setup.sh
cd ~/ax-ros2-ref/ros2_ws
colcon build --packages-select metis_ros2_inference
ros2 launch metis_ros2_inference metis_inference.launch.py
```

### YOLOX models (require normalization)

YOLOX uses ImageNet mean/stddev normalization before inference. YOLOv5 does not — it only
divides by 255. The `normalize` parameter controls this:

```python
MODEL_NAME = 'yolox-s-coco'
MODEL_DIR  = os.path.join(VOYAGER_SDK, 'build', MODEL_NAME, MODEL_NAME, '4')

# In the Node parameters dict, add:
'normalize': True,
'mean':      [0.485, 0.456, 0.406],
'stddev':    [0.229, 0.224, 0.225],
```

> ⚠️ **Warning:** Setting `normalize: false` for a YOLOX model (or `normalize: true` for
> YOLOv5) will produce silently wrong detections — the model will run but bounding boxes
> will be meaningless. Always check the model zoo YAML for the `normalize` key.

---

## Normalization Reference

| Model family | `normalize` | `mean` | `stddev` | Where to check |
| :----------- | :---------- | :----- | :------- | :------------- |
| YOLOv5 (all variants) | `false` | (ignored) | (ignored) | `ax_models/zoo/.../yolov5*.yaml` — no `mean` key present |
| YOLOv8 (all variants) | `false` | (ignored) | (ignored) | Same as YOLOv5 |
| YOLOX (all variants) | `true` | `[0.485, 0.456, 0.406]` | `[0.229, 0.224, 0.225]` | `ax_models/zoo/.../yolox*.yaml` — `mean` key present |

To check a specific model YAML:

```bash
grep -E 'normalize|mean|stddev' \
  ~/voyager-sdk/ax_models/zoo/yolo/object_detection/yolox-s-coco-onnx.yaml
```

If the YAML contains a `mean` or `normalize` key with a non-zero value, set `normalize: true`
in the launch file.

---

## Using a Custom Model

You can bring any ONNX detection model to this reference implementation in five steps:

**Step 1 — Export to ONNX**

```bash
# PyTorch example (YOLOv5-style)
python3 export.py --weights my_model.pt --include onnx --opset 17
```

**Step 2 — Create a zoo YAML**

Copy the closest existing zoo YAML and update the `model` path, input shape,
class names, and calibration dataset fields:

```bash
cp ~/voyager-sdk/ax_models/zoo/yolo/object_detection/yolov5s-v7-coco-onnx.yaml \
   ~/voyager-sdk/ax_models/zoo/yolo/object_detection/my_model.yaml
# Edit my_model.yaml: set onnx_model, labels, input_size, calibration_dataset
```

**Step 3 — Compile**

```bash
cd ~/voyager-sdk && source venv/bin/activate
python3 deploy.py ax_models/zoo/yolo/object_detection/my_model.yaml \
  --target metis --num-cores 4
```

**Step 4 — Update the launch file**

```python
MODEL_NAME = 'my_model'
LABELS     = '/path/to/my_labels.names'
```

**Step 5 — Rebuild and run**

```bash
colcon build --packages-select metis_ros2_inference
ros2 launch metis_ros2_inference metis_inference.launch.py
```

For detailed guidance on custom YAML authoring and calibration dataset preparation, see the
[Voyager SDK custom model tutorial](https://github.com/axelera-ai-hub/voyager-sdk/blob/main/docs/tutorials/custom_model.md).

---

## Verifying a New Model Works

Run through this checklist after every model swap:

```bash
# 1. Compiled artifacts exist
ls ~/voyager-sdk/build/<MODEL_NAME>/<MODEL_NAME>/4/
# Expected: model.json  postprocess_graph.onnx

# 2. Node logs correct tensor shapes at startup
ros2 launch metis_ros2_inference metis_inference.launch.py
# Look for: "[metis_inference]: input shape: [1, H, W, 3]"

# 3. Detections topic is publishing
ros2 topic hz /detections
# Expected: rate matches video FPS

# 4. Detections are non-empty (at least occasionally)
ros2 topic echo /detections | grep class_id
# Expected: class_id: "<label>" lines for each detected object

# 5. Annotated frames appear in results/
ls ~/ax-ros2-ref/results/ | tail -5
```

> 💡 **Tip:** If `/detections` publishes at the correct rate but all detection arrays are
> empty, the most common causes are: wrong `normalize` setting, mismatched `conf_threshold`
> (try lowering to `0.1`), or a mismatch between the label file and the model's class count.
