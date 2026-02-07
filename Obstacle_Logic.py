import cv2
import depthai as dai
import numpy as np
import time

# --- TUNING PARAMETERS ---
SAFE_DISTANCE_MM = 600
BLACK_THRESHOLD = 60
MIN_CONTOUR_AREA = 500

# --- 1. OBSTACLE ZONE (Red Box - Look Ahead) ---
OBST_ROI_X = 158
OBST_ROI_Y = 100
OBST_ROI_W = 100
OBST_ROI_H = 80

# --- 2. LINE ZONE (Yellow Box - Look Down) ---
# OLD: Full width (0 to 416)
# NEW: Narrower box in the center to ignore side noise
LINE_ROI_W = 200        # Width of the box (smaller = ignores sides more)
LINE_ROI_X = (416 - LINE_ROI_W) // 2  # Automatically center it
LINE_ROI_Y = 220
LINE_ROI_H = 80

# Pipeline setup
pipeline = dai.Pipeline()

# Mono Cameras
left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# RGB Camera
rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgb_out = rgb_cam.requestOutput((416, 312), dai.ImgFrame.Type.BGR888p)

# Stereo Depth
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(False)

left.requestFullResolutionOutput().link(stereo.left)
right.requestFullResolutionOutput().link(stereo.right)

# Outputs
depth_q = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
rgb_q = rgb_out.createOutputQueue(maxSize=4, blocking=False)

print("Running... Press 'q' to quit.")

# Main loop
with pipeline:
    pipeline.start()

    while pipeline.isRunning():
        in_depth = depth_q.tryGet()
        in_rgb = rgb_q.tryGet()

        if in_depth is None or in_rgb is None:
            time.sleep(0.001)
            continue

        depth_frame = in_depth.getFrame()
        rgb_frame = in_rgb.getCvFrame()
        
        # --- 1. OBSTACLE DETECTION (Red Box) ---
        obst_roi = depth_frame[OBST_ROI_Y:OBST_ROI_Y+OBST_ROI_H, 
                               OBST_ROI_X:OBST_ROI_X+OBST_ROI_W]
        valid_depths = obst_roi[obst_roi > 0]
        distance = int(np.median(valid_depths)) if valid_depths.size else 9999

        # Draw Obstacle Box
        color_status = (0, 255, 0)
        if 0 < distance < SAFE_DISTANCE_MM:
            color_status = (0, 0, 255)
        cv2.rectangle(rgb_frame, (OBST_ROI_X, OBST_ROI_Y), 
                      (OBST_ROI_X+OBST_ROI_W, OBST_ROI_Y+OBST_ROI_H), color_status, 2)
        cv2.putText(rgb_frame, f"Dist: {distance}mm", (OBST_ROI_X, OBST_ROI_Y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_status, 2)

        # --- 2. LINE DETECTION (Yellow Box) ---
        # Crop to the NEW narrower ROI
        line_roi_slice = rgb_frame[LINE_ROI_Y:LINE_ROI_Y+LINE_ROI_H, 
                                   LINE_ROI_X:LINE_ROI_X+LINE_ROI_W]
        
        gray_roi = cv2.cvtColor(line_roi_slice, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_roi, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_detected = False
        error = 0
        
        # Center of the SEARCH BOX, not the whole screen
        box_center_x = LINE_ROI_W // 2 

        # Draw the Yellow Search Box
        cv2.rectangle(rgb_frame, (LINE_ROI_X, LINE_ROI_Y), 
                      (LINE_ROI_X+LINE_ROI_W, LINE_ROI_Y+LINE_ROI_H), (0, 255, 255), 2)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx_local = int(M["m10"] / M["m00"])
                    cy_local = int(M["m01"] / M["m00"])
                    
                    # Error is relative to the center of the yellow box
                    error = cx_local - box_center_x
                    line_detected = True
                    
                    # Convert local box coordinates to global screen coordinates for drawing
                    cx_global = cx_local + LINE_ROI_X
                    cy_global = cy_local + LINE_ROI_Y
                    
                    # Draw Line
                    c_shifted = c + [LINE_ROI_X, LINE_ROI_Y]
                    cv2.drawContours(rgb_frame, [c_shifted], -1, (0, 255, 0), 2)
                    cv2.circle(rgb_frame, (cx_global, cy_global), 5, (0, 0, 255), -1)

        # --- 3. LOGIC ---
        if 0 < distance < SAFE_DISTANCE_MM:
            status = "STOP! OBSTACLE"
            color = (0, 0, 255)
            print(f"CMD: <STOP> {distance}mm")
        elif line_detected:
            # Note: Threshold is smaller now because the box is smaller
            if error < -15:
                status = "Turn LEFT"
                color = (0, 255, 0)
            elif error > 15:
                status = "Turn RIGHT"
                color = (0, 255, 0)
            else:
                status = "FORWARD"
                color = (0, 255, 0)
        else:
            status = "Searching..."
            color = (255, 100, 0)

        cv2.putText(rgb_frame, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)
        cv2.imshow("Robot View", rgb_frame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
