import cv2
import depthai as dai
import numpy as np
import time

# --- CONFIGURATION ---
SAFE_DISTANCE_MM = 600  # 60cm
ENABLE_SERIAL = False   # True only when connected to Robot (Windows = COMx)

if ENABLE_SERIAL:
    import serial
    try:
        # Windows example: 'COM5' (check Device Manager > Ports)
        ser = serial.Serial('COM5', 115200, timeout=0.1)
    except Exception as e:
        ser = None
        print(f"WARNING: Could not connect to Serial Port! ({e})")

# --- PIPELINE SETUP (DepthAI v3 style) ---
pipeline = dai.Pipeline()

# v3 uses Camera node (MonoCamera is deprecated)
# OAK-D Lite mono cameras are on CAM_B and CAM_C
left_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)

# Stereo settings (your intent preserved)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)
stereo.setExtendedDisparity(False)

# Link cameras to stereo
left_out = left_cam.requestFullResolutionOutput()
right_out = right_cam.requestFullResolutionOutput()
left_out.link(stereo.left)
right_out.link(stereo.right)

# Create a host output queue directly from the stereo output (v3)
depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

print("Starting Obstacle Detection Logic (DepthAI v3)...")
print("Press 'q' to quit.")

with pipeline:
    pipeline.start()

    while pipeline.isRunning():
        in_depth = depth_queue.tryGet()
        if in_depth is None:
            # No new frame yet
            time.sleep(0.001)
            continue

        frame = in_depth.getFrame()  # depth map (mm)

        # --- CALCULATE DISTANCE ---
        h, w = frame.shape
        cy, cx = h // 2, w // 2

        roi = frame[cy-5:cy+5, cx-5:cx+5]
        valid = roi[roi > 0]

        if valid.size > 0:
            # Median is often more stable than mean for depth speckle/outliers
            current_distance = int(np.median(valid))
        else:
            current_distance = 9999

        # --- VISUALIZE ---
        # Normalize for display (not for measurement)
        disp = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        disp = cv2.equalizeHist(disp)
        disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

        # --- LOGIC & DECISION ---
        if 0 < current_distance < SAFE_DISTANCE_MM:
            status = "CRITICAL: OBSTACLE TOO CLOSE!"
            command = "<STOP>"
            color = (0, 0, 255)
            print(f"COMMAND SENT >>> {command} (Dist: {current_distance}mm)")
            if ENABLE_SERIAL and ser is not None:
                ser.write(command.encode("ascii", errors="ignore"))
        else:
            status = "PATH CLEAR"
            command = "<MOVE>"
            color = (0, 255, 0)

        cv2.putText(disp, f"Dist: {current_distance}mm", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(disp, status, (30, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.rectangle(disp, (cx-10, cy-10), (cx+10, cy+10), (255, 255, 255), 2)

        cv2.imshow("Robot Vision Logic", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            pipeline.stop()
            break

cv2.destroyAllWindows()
if ENABLE_SERIAL and 'ser' in globals() and ser is not None:
    ser.close()
