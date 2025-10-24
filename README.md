# RobostellarX - 6th Kibo RPC Project

Welcome to the repository for **RobostellarX**, our entry for the 6th Kibo Robot Programming Challenge (KRPC).

## 🏆 Achievement: 3rd Place (Bangladesh Region) 🏆

We are proud to have achieved **3rd Place in the Bangladesh region** with this project.

This project involves programming JAXA's Astrobee robot to perform autonomous tasks aboard the International Space Station (ISS) Kibo module.

## Project Overview

The mission required our Astrobee to navigate through the Kibo module, identify specific targets, and visually detect "treasure items." The core of the mission was to:

1. Navigate to five different target locations.

2. At each location, scan the area, identify AR (Aruco) markers, and capture images.

3. Process the captured images to detect and classify landmark items and special treasure items.

4. Identify a final treasure item displayed by the astronaut (Target 5).

5. Match the final treasure item with one of the previously scanned targets.

6. Navigate to the correct target's final position with high precision and take a snapshot.

## Key Features & Strategy

Our solution was built around precision navigation, robust image processing, and efficient object detection.

* **Navigation:** We used a series of predefined waypoints (`gov.nasa.arc.astrobee.types.Point` and `Quaternion`) to move the Astrobee. A retry mechanism was implemented in our `moveAstrobee` function to ensure reliability in movement commands.

* **AR Marker Detection:** We used OpenCV's `ArucoDetector` to find `DICT_5X5_250` markers. This was crucial for:

  * **Image Cropping:** Automatically locating the region of interest (ROI) around the marker at each target. The `process_AR` function handles image sharpening, undistortion, and rotation to extract a clear, cropped image (`postImage`) for object detection.

  * **Final Targeting:** Our `finalTarget` function re-scans the AR marker of the matched area, calculates its pixel offset from the camera's center, and converts this offset into a precise coordinate adjustment. This allowed us to move the Astrobee with centimeter-level accuracy for the final snapshot.

* **Object Detection:**

  * We employed a **YOLO (You Only Look Once)** object detection model (`krpc-aug-small-yolo11m-batch4-v3.onnx`).

  * The model was loaded using OpenCV's DNN module (`org.opencv.dnn.Net`).

  * Our `detectObject` function runs the model on the AR-cropped images to identify 11 different classes (like "crystal", "diamond", "coin", "treasure_box", etc.).

  * We applied Non-Maximum Suppression (NMS) and an edge-filtering algorithm to reduce false positives and improve detection accuracy.

## Technology Stack

* **Language:** Java

* **Core APIs:**

  * Kibo RPC API (`jp.jaxa.iss.kibo.rpc.api`)

  * NASA Astrobee API (`gov.nasa.arc.astrobee`)

* **Computer Vision:** OpenCV 4.9.0

* **Machine Learning:** OpenCV DNN Module with an ONNX (YOLO) model

* **Environment:** Android Studio

## How It Works (Mission Flow)

1. **Start:** The mission begins, and the Astrobee moves to Target 1. The YOLO model is loaded.

2. **Scanning Phase:**

   * The robot moves sequentially from Target 1 to Target 4.

   * At each stop, it captures images using the NavCam (`api.getMatNavCam()`) and processes the *previous* target's image to find treasure items. This parallel processing saved valuable mission time.

3. **Final Target:**

   * The robot moves to Target 5 (Astronaut) and uses the DockCam (`api.getMatDockCam()`) to capture the final item.

   * It retries AR processing up to 40 times to ensure the marker is found.

   * The detected item (`treasureItem5`) is compared against the items found at Targets 1-4.

4. **Final Approach:**

   * Once the matching area is identified (e.g., Area 2), the `finalTarget(2)` function is called.

   * Astrobee moves to the general area, re-scans the AR marker, calculates the precise offset, and performs a final corrective movement.

5. **Completion:** `api.takeTargetItemSnapshot()` is called to complete the mission.
