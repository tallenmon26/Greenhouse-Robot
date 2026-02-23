import cv2
import depthai as dai
import numpy as np
import time
import serial 
import contextlib
import signal
import sys

# --- Setup Serial Connection to ESP32 ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200
try:
    esp32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) 
except serial.SerialException as e:
    print(f"Warning: Could not connect to ESP32: {e}")
    esp32 = None

FPS_LIMIT = 30  
should_quit = False

# --- Graceful shutdown on Ctrl+C ---
def handle_sigint(sig, frame):
    global should_quit
    print("\nCtrl+C detected — stopping...")
    should_quit = True

signal.signal(signal.SIGINT, handle_sigint)

# --- TUNING PARAMETERS ---
SAFE_DISTANCE_MM = 600
BLACK_THRESHOLD = 60
MIN_CONTOUR_AREA = 500
MISSING_LINE_THRESHOLD = 5

OBST_ROI_X, OBST_ROI_Y, OBST_ROI_W, OBST_ROI_H = 220, 60, 200, 140
LINE_ROI_W, LINE_ROI_H = 200, 200
LINE_ROI_X, LINE_ROI_Y = (640 - LINE_ROI_W) // 2, 280

# --- MULTI-DEVICE INITIALIZATION (DepthAI v3) ---
device_infos = dai.Device.getAllAvailableDevices()
num_cams = len(device_infos)
print(f"Found {num_cams} OAK-D camera(s) on the hub.")

if num_cams == 0:
    raise RuntimeError("No cameras detected! Check your hub power and USB connections.")

active_cam_idx = 1  
missing_line_frames = 0

with contextlib.ExitStack() as stack:
    rgb_qs = []
    depth_qs = []

    for i, info in enumerate(device_infos):
        device = stack.enter_context(dai.Device(info))
        pipeline = stack.enter_context(dai.Pipeline(device))
        
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        rgb_out = rgb_cam.requestOutput((640, 480), dai.ImgFrame.Type.BGR888p)
        
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        
        left.requestOutput((640, 400)).link(stereo.left)
        right.requestOutput((640, 400)).link(stereo.right)
        
        rgb_qs.append(rgb_out.createOutputQueue(maxSize=4, blocking=False))
        depth_qs.append(stereo.depth.createOutputQueue(maxSize=4, blocking=False))
        
        pipeline.start()
        print(f"Started Camera {i}: {info.getDeviceId()}")
        time.sleep(0.2)

    print("\nRunning headless. Press Ctrl+C to stop.")
    last_frame_time = time.time()

    while not should_quit:
        current_time = time.time()
        if current_time - last_frame_time < (1.0 / FPS_LIMIT):
            time.sleep(0.001)
            continue
        last_frame_time = current_time

        for i in range(num_cams):
            in_depth = depth_qs[i].tryGet()
            in_rgb = rgb_qs[i].tryGet()

            if i == active_cam_idx and in_depth is not None and in_rgb is not None:
                depth_frame = in_depth.getFrame()
                rgb_frame = in_rgb.getCvFrame()
                
                # --- OBSTACLE DETECTION ---
                obst_roi = depth_frame[OBST_ROI_Y:OBST_ROI_Y+OBST_ROI_H, OBST_ROI_X:OBST_ROI_X+OBST_ROI_W]
                valid_depths = obst_roi[obst_roi > 0]
                distance = int(np.percentile(valid_depths, 25)) if valid_depths.size else 9999

                # --- LINE DETECTION ---
                line_roi_slice = rgb_frame[LINE_ROI_Y:LINE_ROI_Y+LINE_ROI_H, LINE_ROI_X:LINE_ROI_X+LINE_ROI_W]
                _, thresh = cv2.threshold(line_roi_slice[:, :, 1], BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                line_detected = False
                error = 0
                box_center_x = LINE_ROI_W // 2

                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            error = int(M["m10"] / M["m00"]) - box_center_x
                            line_detected = True

                # --- SWAP LOGIC & MOTOR COMMANDS ---
                command_to_send = b"STOP\n"
                status = ""

                if 0 < distance < SAFE_DISTANCE_MM:
                    status = "STOP! OBSTACLE"
                    command_to_send = b"STOP\n"
                    
                elif line_detected:
                    missing_line_frames = 0
                    
                    if active_cam_idx == 1:
                        if error < -15: status, command_to_send = "Turn LEFT", b"LEFT\n"
                        elif error > 15: status, command_to_send = "Turn RIGHT", b"RIGHT\n"
                        else: status, command_to_send = "FORWARD", b"FORWARD\n"
                    else:
                        if error < -15: status, command_to_send = "Reverse LEFT", b"RIGHT\n" 
                        elif error > 15: status, command_to_send = "Reverse RIGHT", b"LEFT\n"
                        else: status, command_to_send = "BACKWARD", b"BACKWARD\n"
                        
                else:
                    missing_line_frames += 1
                    status = f"Searching... ({missing_line_frames}/{MISSING_LINE_THRESHOLD})"
                    command_to_send = b"STOP\n"
                    
                    if missing_line_frames >= MISSING_LINE_THRESHOLD:
                        if num_cams > 1:
                            active_cam_idx = 1 if active_cam_idx == 0 else 0
                            missing_line_frames = 0
                            print(f"\n--- SWAPPING TO CAMERA {active_cam_idx} ---")
                            time.sleep(0.5) 
                        else:
                            status = "END OF TAPE. STOPPED."

                if esp32:
                    esp32.write(command_to_send)

                cam_label = "FRONT CAM" if active_cam_idx == 0 else "REAR CAM"
                print(f"\r[{cam_label}] {status:<30} | Dist: {distance}mm | Error: {error:+d}   ", end="", flush=True)

# --- SAFE SHUTDOWN ---
if esp32:
    print("\nShutting down... sending final STOP command.")
    esp32.write(b"STOP\n")
    time.sleep(0.1) 
    esp32.close()
    print("Motors stopped.")
