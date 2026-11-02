# ADAS System

This automatic driver assistance system adopts OV5640 binocular cameras and HDMI input interfaces as data acquisition sources, to simulate multi-channel surveillance video inputs from four directions (front, rear, left and right) of a motor vehicle in real-vehicle scenarios.
The system adopts a hardware-software co-design architecture. The FPGA lower computer is mainly responsible for image acquisition, capturing raw image data, and performing preprocessing including image denoising, color space conversion and binarization on the captured raw frames. It also completes stitching and fusion of multi-channel images to simulate the imaging effect of an on-board panoramic view.
The PCIe protocol is used for data communication between the FPGA lower computer and the RK3568 upper computer. The preprocessed image data and driver operation information simulated by keys on the development board are synchronized and transmitted to the upper computer.
A YOLO object detection model is deployed locally on the upper computer. The model performs object recognition for lane information, traffic light status and surrounding road conditions during driving. Once a safety risk in the driver’s operation is detected, corresponding warning markers are overlaid on the output image to alert the driver, realizing intelligent ADAS early warning.
The system records risky driving events and live camera frames. The recorded data can be accessed from PCs and mobile terminals for driving behavior supervision and traceability.

```

adas system
├─ fpga                         Verilog source for FPGA side
├─ rk3568                       Source code for RK3568
│  ├─ app/main.cpp               Entry point for CLI version
│  ├─ capture/                   Packet parsing, 720p frame assembly, binocular fusion frame assembly, UDP receiving, pcap replay
│  ├─ vision/                    RKNN inference wrapper, preprocessing (letterbox), postprocessing (decode/NMS), mask
│  ├─ perception/                Lane clustering / lane type / tracking, IPM inverse perspective transformation + distortion correction, rule engine
│  │  └─ rules/                  Hazard rules R1–R7
│  ├─ render/                    BEV drawing commands & software rasterization
│  ├─ ui/                        Qt UI (main window / video widget / BEV widget / background pipeline)
│  ├─ event/                     JPEG encoding + event storage (JSONL index, capacity-limited FIFO)
│  ├─ net/                       HTTP multipart upload
│  └─ common/                    Common data types & logging
└─ README.md                    ← This file

```

## Glossary
- BEV: Bird’s Eye View
- IPM: Inverse Perspective Mapping
- NMS: Non-Maximum Suppression
- RKNN: Rockchip Neural Network
- JSONL: JSON Lines
- CLI: Command-Line Interface
