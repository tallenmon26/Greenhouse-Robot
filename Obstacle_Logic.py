import cv2
import depthai as dai
import numpy as np
import time

# --- CONFIGURATION ---
SAFE_DISTANCE_MM = 600   # Distance to trigger "STOP"
BLACK_THRESHOLD = 60     # Line detection sensitivity

# Pipeline setup
pipeline = dai.Pipeline()

# Mono Cameras (for Depth)
left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# RGB Camera (for Line Following)
rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgb_out = rgb_cam.requestOutput((416, 312), dai.ImgFrame.Type.BGR888p)

# Stereo Depth Setup
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(False)

left.requestFullResolutionOutput().link(stereo.left)
right.requestFullResolutionOutput().link(stereo.right)

# Create Output Queues
depth_q = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
rgb_q = rgb_out.createOutputQueue(maxSize=4, blocking=False)

print("System Started — Press 'q' to quit")

# Main loop
with pipeline:
    pipeline.start()

    while pipeline.isRunning():
        in_depth = depth_q.tryGet()
        in_rgb = rgb_q.tryGet()

        if in_depth is None or in_rgb is None:
            time.sleep(0.001)
            continue

        # --- 1. DEPTH PROCESSING ---
        depth = in_depth.getFrame()
        h, w = depth.shape
        
        # Calculate distance from center
        roi = depth[h//2-20:h//2+20, w//2-20:w//2+20]
        valid = roi[roi > 0]
        distance = int(np.median(valid)) if valid.size else 9999

        # VISUALIZATION: Depth Map
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        depth_vis = cv2.applyColorMap(cv2.equalizeHist(depth_vis), cv2.COLORMAP_JET)
        cv2.imshow("Depth View", depth_vis)

        # --- 2. RGB LINE FOLLOWING ---
        frame = in_rgb.getCvFrame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_detected = False
        error = 0
        center_x = frame.shape[1] // 2

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    error = cx - center_x
                    line_detected = True
                    cv2.drawContours(frame, [c], -1, (0,255,0), 2)
                    cv2.line(frame, (center_x, 240), (cx, 240), (255,0,0), 2)

        # --- 3. LOGIC DECISION ---
        # This is where the code "Thinks"
        
        if 0 < distance < SAFE_DISTANCE_MM:
            # STOP LOGIC
            status = "STOP! OBSTACLE"
            color = (0, 0, 255) # Red
            print(f"COMMAND: <STOP> | Distance: {distance}mm") 
            
        elif line_detected:
            # LINE FOLLOWING LOGIC
            if error < -20:
                status = "Turning Left"
                color = (0, 255, 0) # Green
            elif error > 20:
                status = "Turning Right"
                color = (0, 255, 0) # Green
            else:
                status = "Forward"
                color = (0, 255, 0) # Green
        else:
            # SEARCH LOGIC
            status = "Searching..."
            color = (255, 0, 0) # Blue

        # Draw the status on the screen
        cv2.putText(frame, f"ACTION: {status}", (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        cv2.imshow("RGB View", frame)
        
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
