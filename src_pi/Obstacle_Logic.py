import cv2
import depthai as dai
import numpy as np
import time
import serial 
import contextlib

# Hardware Communication Protocols
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

try:
    esp32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) 
except serial.SerialException as e:
    print(f"Warning: Serial initialization failed: {e}")
    esp32 = None    

FPS_LIMIT = 30  
should_quit = False

# System Parameters & Constraints
SAFE_DISTANCE_MM = 600
MIN_CONTOUR_AREA = 500
MISSING_LINE_THRESHOLD = 30 

OBST_ROI_W, OBST_ROI_H = 200, 100
OBST_ROI_X = (640 - OBST_ROI_W) // 2  
OBST_ROI_Y = 0

LINE_ROI_W, LINE_ROI_H = 200, 200
LINE_ROI_X, LINE_ROI_Y = (640 - LINE_ROI_W) // 2, 280

# --- FINITE STATE MACHINE VARIABLES ---
STATE_ROW_OUTWARD = 0
STATE_ROW_RETURN = 1
STATE_AISLE_TRANSIT = 2

current_nav_state = STATE_ROW_OUTWARD

# --- VERTICAL ACTUATION VARIABLES ---
print("\n=== GREENHOUSE MISSION SETUP ===")
while True:
    try:
        lower_height = float(input("Enter LOWER target height (ft): "))
        higher_height = float(input("Enter HIGHER target height (ft): "))
        break # If both inputs succeed, break out of the loop and continue the script
    except ValueError:
        print("[ERROR] Invalid input. Please enter valid numbers and try again.\n")

current_height = 0.0
target_height = lower_height # Start by going to the lower height
is_adjusting_height = True if esp32 else False # Skip if no ESP32 connected
height_tolerance = 0.1
last_auto_cmd = b""
last_auto_cmd_time = 0
# --------------------------------------

# DepthAI Multi-Device Configuration
device_infos = dai.Device.getAllAvailableDevices()
if len(device_infos) > 2:
    device_infos = device_infos[:2]

num_cams = len(device_infos)
print(f"\nInitialized hub. Navigation Devices detected: {num_cams}")

if num_cams == 0:
    raise RuntimeError("Hardware Error: Zero OAK-D devices enumerated.")

# Hardware index mapping
FRONT_CAM_INDEX = 1
REAR_CAM_INDEX = 0

active_cam_idx = FRONT_CAM_INDEX  
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
        print(f"Pipeline active for Device {i} [ID: {info.getDeviceId()}]")
        time.sleep(0.2) 

    print("\nSystem active. Awaiting user interrupt (q).")
    last_frame_time = time.time()

    while not should_quit:
        current_time = time.time()
        if current_time - last_frame_time < (1.0 / FPS_LIMIT):
            time.sleep(0.001)
            continue
        last_frame_time = current_time
        
        # 1. READ LIDAR DATA (Non-blocking)
        if esp32:
            while esp32.in_waiting > 0:
                try:
                    line = esp32.readline().decode('utf-8').strip()
                    if "LIDAR Distance:" in line:
                        try:
                            # 1.5 Foot Ground Offset
                            raw_lidar_feet = float(line.split(":")[1].strip())
                            current_height = raw_lidar_feet + 1.5 
                        except ValueError:
                            pass
                except UnicodeDecodeError:
                    pass 

        for i in range(num_cams):
            in_depth = depth_qs[i].tryGet()
            in_rgb = rgb_qs[i].tryGet()

            if i == active_cam_idx and in_depth is not None and in_rgb is not None:
                depth_frame = in_depth.getFrame()
                rgb_frame = in_rgb.getCvFrame()
                
                # Depth Processing & Obstacle Detection
                obst_roi = depth_frame[OBST_ROI_Y:OBST_ROI_Y+OBST_ROI_H, OBST_ROI_X:OBST_ROI_X+OBST_ROI_W]
                valid_depths = obst_roi[obst_roi > 0]
                distance = int(np.percentile(valid_depths, 25)) if valid_depths.size else 9999

                color_status = (0, 0, 255) if (0 < distance < SAFE_DISTANCE_MM) else (0, 255, 0)
                cv2.rectangle(rgb_frame, (OBST_ROI_X, OBST_ROI_Y), (OBST_ROI_X+OBST_ROI_W, OBST_ROI_Y+OBST_ROI_H), color_status, 2)
                cv2.putText(rgb_frame, f"Dist: {distance}mm", (OBST_ROI_X, OBST_ROI_Y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_status, 2)

                # --- VISION PROCESSING ---
                line_roi_slice = rgb_frame[LINE_ROI_Y:LINE_ROI_Y+LINE_ROI_H, LINE_ROI_X:LINE_ROI_X+LINE_ROI_W]
                blurred_roi = cv2.GaussianBlur(line_roi_slice, (9, 9), 0)
                hsv_roi = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)
                
                if current_nav_state in [STATE_ROW_OUTWARD, STATE_ROW_RETURN]:
                    lower_color_1 = np.array([0, 60, 60]) 
                    upper_color_1 = np.array([10, 255, 255])
                    mask1 = cv2.inRange(hsv_roi, lower_color_1, upper_color_1)
                    
                    lower_color_2 = np.array([160, 60, 60])
                    upper_color_2 = np.array([180, 255, 255])
                    mask2 = cv2.inRange(hsv_roi, lower_color_2, upper_color_2)
                    thresh = cv2.bitwise_or(mask1, mask2)
                    target_color_text = "Target: RED"
                else: 
                    lower_green = np.array([40, 60, 60])
                    upper_green = np.array([90, 255, 255])
                    thresh = cv2.inRange(hsv_roi, lower_green, upper_green)
                    target_color_text = "Target: GREEN"
                
                kernel = np.ones((5, 5), np.uint8)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

                cv2.imshow("Binary Mask", thresh)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                line_detected = False
                error = 0
                box_center_x = LINE_ROI_W // 2 

                cv2.rectangle(rgb_frame, (LINE_ROI_X, LINE_ROI_Y), (LINE_ROI_X+LINE_ROI_W, LINE_ROI_Y+LINE_ROI_H), (0, 255, 255), 2)

                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            error = cx - box_center_x
                            
                            topmost = tuple(c[c[:,:,1].argmin()][0])
                            bottommost = tuple(c[c[:,:,1].argmax()][0])
                            
                            dx = topmost[0] - bottommost[0]
                            dy = bottommost[1] - topmost[1] 
                            if dy == 0: dy = 1 
                            
                            angle = int(np.degrees(np.arctan2(dx, dy)))
                            line_detected = True
                            
                            cv2.circle(rgb_frame, (cx + LINE_ROI_X, cy + LINE_ROI_Y), 5, (0, 0, 255), -1)
                            cv2.line(rgb_frame, 
                                     (bottommost[0] + LINE_ROI_X, bottommost[1] + LINE_ROI_Y), 
                                     (topmost[0] + LINE_ROI_X, topmost[1] + LINE_ROI_Y), 
                                     (0, 255, 0), 3)

                # ==========================================
                # DECISION ENGINE (Driving vs Lifting)
                # ==========================================
                command_to_send = b"" # Default to no command sent to prevent buffer flooding
                status = ""

                # OVERRIDE 1: Are we adjusting the height?
                if is_adjusting_height:
                    status = f"Lifting to {target_height}ft..."
                    
                    if current_height < target_height - height_tolerance:
                        if last_auto_cmd != b"UP\n" or (current_time - last_auto_cmd_time > 0.2):
                            command_to_send = b"UP\n"
                            last_auto_cmd = b"UP\n"
                            last_auto_cmd_time = current_time
                            
                    elif current_height > target_height + height_tolerance:
                        if last_auto_cmd != b"DOWN\n" or (current_time - last_auto_cmd_time > 0.2):
                            command_to_send = b"DOWN\n"
                            last_auto_cmd = b"DOWN\n"
                            last_auto_cmd_time = current_time
                            
                    else:
                        print(f"\n[SYSTEM] Height {target_height}ft reached. Resuming driving.")
                        command_to_send = b"STOP\n"
                        is_adjusting_height = False
                        last_auto_cmd = b""
                        missing_line_frames = 0 # Reset frames to prevent accidental double-triggers

                # OVERRIDE 2: Is there an obstacle?
                elif 0 < distance < SAFE_DISTANCE_MM:
                    status = "STOP! OBSTACLE"
                    command_to_send = b"STOP\n"
                    
                # STANDARD NAVIGATION: We are at the correct height and path is clear!
                elif line_detected:
                    missing_line_frames = 0 
                    throttle_speed = int(np.interp(cy, [0, LINE_ROI_H], [127, 40]))
                    if esp32: esp32.write(f"SPD:{throttle_speed}\n".encode())
                    
                    if active_cam_idx == FRONT_CAM_INDEX:
                        if error < -100: status, command_to_send = "Edge LEFT", b"LEFT\n"
                        elif error > 100: status, command_to_send = "Edge RIGHT", b"RIGHT\n"
                        elif error < -50 or angle < -50: status, command_to_send = "Tight Arc L", b"TIGHT_ARC_LEFT\n"
                        elif error > 50 or angle > 50: status, command_to_send = "Tight Arc R", b"TIGHT_ARC_RIGHT\n"
                        elif error < -15 or angle < -15: status, command_to_send = "Arc LEFT", b"ARC_LEFT\n"
                        elif error > 15 or angle > 15: status, command_to_send = "Arc RIGHT", b"ARC_RIGHT\n"
                        else: status, command_to_send = "FORWARD", b"FORWARD\n"
                        
                    elif active_cam_idx == REAR_CAM_INDEX:
                        if error < -100: status, command_to_send = "Edge REV L", b"LEFT\n" 
                        elif error > 100: status, command_to_send = "Edge REV R", b"RIGHT\n"
                        elif error < -50 or angle < -50: status, command_to_send = "Tight Arc L", b"TIGHT_ARC_REV_RIGHT\n"
                        elif error > 50 or angle > 50: status, command_to_send = "Tight Arc R", b"TIGHT_ARC_REV_LEFT\n"
                        elif error < -15 or angle < -15: status, command_to_send = "Arc REV L", b"ARC_REV_RIGHT\n"
                        elif error > 15 or angle > 15: status, command_to_send = "Arc REV R", b"ARC_REV_LEFT\n"
                        else: status, command_to_send = "BACKWARD", b"BACKWARD\n"
                
                # FSM TRANSITION PROTOCOL
                else:
                    missing_line_frames += 1
                    status = f"Searching... ({missing_line_frames}/{MISSING_LINE_THRESHOLD})"
                    command_to_send = b"STOP\n"
                    
                    if missing_line_frames >= MISSING_LINE_THRESHOLD:
                        missing_line_frames = 0
                        
                        if current_nav_state == STATE_ROW_OUTWARD:
                            if num_cams > 1:
                                current_nav_state = STATE_ROW_RETURN
                                active_cam_idx = REAR_CAM_INDEX
                                
                                # TRIGGER HEIGHT ADJUSTMENT
                                target_height = higher_height
                                is_adjusting_height = True
                                print("\n[SYSTEM] End of row. Adjusting to HIGHER height. Active feed: REAR CAM (RED TAPE)")
                            else:
                                status = "END OF TAPE. STOPPED."
                                
                        elif current_nav_state == STATE_ROW_RETURN:
                            current_nav_state = STATE_AISLE_TRANSIT
                            active_cam_idx = FRONT_CAM_INDEX
                            
                            # TRIGGER HEIGHT ADJUSTMENT
                            target_height = lower_height
                            is_adjusting_height = True
                            print("\n[SYSTEM] Row complete. Adjusting to LOWER height. Entering aisle. Active feed: FRONT CAM (GREEN TAPE)")
                            
                        elif current_nav_state == STATE_AISLE_TRANSIT:
                            current_nav_state = STATE_ROW_OUTWARD
                            active_cam_idx = FRONT_CAM_INDEX
                            print("\n[SYSTEM] Arrived at new row. Active feed: FRONT CAM (RED TAPE)")

                        time.sleep(0.5) 

                if esp32 and command_to_send != b"":
                    esp32.write(command_to_send)

                # Render active telemetry and FSM State
                cam_label = "FRONT CAM" if active_cam_idx == FRONT_CAM_INDEX else "REAR CAM"
                cv2.putText(rgb_frame, cam_label, (450, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
                cv2.putText(rgb_frame, target_color_text, (450, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(rgb_frame, f"Height: {current_height:.1f}ft", (450, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                cv2.putText(rgb_frame, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                
                cv2.imshow("Robot View", rgb_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    should_quit = True

# System Teardown
cv2.destroyAllWindows()
if esp32:
    print("\nInitiating safe shutdown sequence...")
    esp32.write(b"STOP\n")
    time.sleep(0.1) 
    esp32.close()
    print("Actuators disengaged. Offline.")
