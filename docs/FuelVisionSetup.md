# FuelVision — PhotonVision ML Setup for OrangePi 5

Camera: **Arducam OV9782**, 1MP 1280×800 global shutter, 70°(H) M12 low-distortion lens.
Pipeline name in PhotonVision dashboard: **`fuelCam`** (must match `FuelVisionConstants.kCameraName`).

---

## Recommended Model: Use the Built-In PhotonVision Fuel Model

PhotonVision v2026.2.1+ ships with an official Fuel detection model contributed by **Team 2826 (Wave Robotics)**. It is bundled in the PhotonVision JAR and extracted automatically on first boot — no manual download needed if you flash the latest image.

- Architecture: YOLOv11, 640×640 input
- Training data: ~2,600 augmented images from 60 original photos
- Expected performance on OrangePi 5 NPU: 30+ FPS

**This is the right starting point.** Try it before considering a custom model.

---

## Model Options

### Option 1 — Official PhotonVision Fuel model (bundled, recommended)
Already on the device after flashing. Select it in the PhotonVision dashboard under the Object Detection pipeline.

### Option 2 — Wave Robotics YOLOv11 standalone release
Same model as above but available separately if you need to update without reflashing.
- Chief Delphi thread: https://www.chiefdelphi.com/t/introducing-wave-robotics-yolov11-model-for-rebuilt/512701

### Option 3 — Popcorn Penguins (Team 6238) YOLOv11 model
An independently trained alternative — worth testing if the official model gives too many false positives in your venue.
- Chief Delphi thread: https://www.chiefdelphi.com/t/popcorn-penguins-yolov11-rebuilt-vision-model/514496
ashboard

**OrangePi 5 requirements:**
- Format: `.rknn` only (uses the RK3588 6-TOPS NPU via PhotonVision's RKNN JNI wrapper)
- Must be quantized (int8) — non-quantized models will not run
- Supported architectures: YOLOv5, YOLOv5u, YOLOv8, YOLOv11 (640×640)

**Benchmark (OrangePi 5, COCO 2017):**
| Model    | Inference | mAP    |
|----------|-----------|--------|
| YOLOv5   | ~15 ms    | 0.2243 |
| YOLOv5u  | ~16 ms    | 0.2745 |
| YOLOv8   | ~17 ms    | 0.3051 |
| YOLOv11  | ~23 ms    | 0.3251 |


## Pre-Downloading Models for Competition (Offline Use)

Models live on the OrangePi at:
```
/opt/photonvision/photonvision_config/models/
```

Each model needs two files:
```
fuel-640-640-yolov11s.rknn
fuel-640-640-yolov11s-labels.txt
```

**To pre-load a model without internet at the venue:**
1. Download the `.rknn` and `-labels.txt` files at home
2. Copy them to the OrangePi via SCP/FileZilla over USB or the robot network:
   ```
   scp fuel-640-640-yolov11s.rknn pi@10.82.48.11:/opt/photonvision/photonvision_config/models/
   scp fuel-640-640-yolov11s-labels.txt pi@10.82.48.11:/opt/photonvision/photonvision_config/models/
   ```
3. Restart PhotonVision — the model will appear in the dashboard

**Alternative — export/import full config:**
The PhotonVision dashboard can export the entire `photonvision_config/` directory as a ZIP (Settings → Export). Import it on another device to replicate all pipelines and models at once. Useful for swapping or cloning OrangePi units at competition.

---

## Deploying a New PhotonVision Version

Latest release: https://github.com/PhotonVision/photonvision/releases

Flash the OrangePi image rather than upgrading the JAR when possible — this ensures the RKNN runtime and JNI libraries match the PhotonVision version.

---

## Simulation Note

The real camera ML pipeline cannot be simulated — PhotonVision's `VisionSystemSim` only renders AprilTag scenes and does not run ML models on synthetic frames. `FuelVision.java` uses Maple Sim ground-truth game piece positions in simulation instead, which is the standard approach for ML subsystem testing in FRC.

---

## TODO After Physical Camera Mounting

- [ ] Measure actual camera offset and update `FuelVisionConstants.kRobotToCamera` (X forward, Y left, Z up from robot center, plus roll/pitch/yaw)
- [ ] Calibrate camera in PhotonVision dashboard (required for ML pipeline)
- [ ] Confirm pipeline name matches `"fuelCam"` in PhotonVision
- [ ] Verify `kHorizHalfFOVDegrees` / `kVertHalfFOVDegrees` match actual lens (spec says 70° H)
- [ ] Test detection at various field lighting conditions and tune confidence threshold in PhotonVision if needed
