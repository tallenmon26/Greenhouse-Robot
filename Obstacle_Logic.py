import cv2
import depthai as dai
import numpy as np
import warnings
import time

warnings.filterwarnings("ignore")

SAFE_DISTANCE_MM = 600
BLACK_THRESHOLD = 60
ENABLE_SERIAL = False

if ENABLE_SERIAL:
    import serial
    ser = serial.Serial("COM5", 115200, timeout=0.1)

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

print("Robot Vision Started — Press Q to quit")

# Main loop
with pipeline:
    pipeline.start()

    while pipeline.isRunning():
        in_depth = depth_q.tryGet()
        in_rgb = rgb_q.tryGet()

        if in_depth is None or in_rgb is None:
            time.sleep(0.001)
            continue

        # Depth processing
        depth = in_depth.getFrame()
        h, w = depth.shape
        roi = depth[h//2-10:h//2+10, w//2-10:w//2+10]
        valid = roi[roi > 0]
        distance = int(np.median(valid)) if valid.size else 9999

        depth_vis = cv2.normalize(depth, None, 0, 255,
                                  cv2.NORM_MINMAX, cv2.CV_8UC1)
        depth_vis = cv2.applyColorMap(
            cv2.equalizeHist(depth_vis), cv2.COLORMAP_JET)
        cv2.imshow("Depth", depth_vis)

        # RGB LINE FOLLOWING
        frame = in_rgb.getCvFrame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, BLACK_THRESHOLD, 255,
                                  cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

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
                    cv2.line(frame, (center_x, 240),
                             (cx, 240), (255,0,0), 2)

        cv2.imshow("RGB Line Follow", frame)

        # Make decision based on distance and line position
        if 0 < distance < SAFE_DISTANCE_MM:
            command = "<STOP>"
        elif line_detected:
            if error < -20:
                command = "<TURN_L>"
            elif error > 20:
                command = "<TURN_R>"
            else:
                command = "<MOVE_FWD>"
        else:
            command = "<SEARCH>"

        if ENABLE_SERIAL:
            ser.write(command.encode())

        if cv2.waitKey(1) == ord('q'):
            pipeline.stop()
            break

cv2.destroyAllWindows()
if ENABLE_SERIAL:
    ser.close()
