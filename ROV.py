import sys
import pygame
import socket
import threading
import serial  # For serial communication
import cv2
import numpy as np
import time
import os
from ultralytics import YOLO
# ===============================
# Global Variables and Settings
# ===============================
# For video feed
latest_frame = None
frame_lock = threading.Lock()




#YOlO Initialization 

YOLO_MODEL_PATH = "/home/rov/Desktop/ROV_Scripts/my_model/my_model.pt"  # Update with your model file
yolo_model = YOLO(YOLO_MODEL_PATH, task="detect")
MIN_CONF_THRESH = 0.5  # Minimum confidence threshold


# ROV sensor state variables (all sensor data stored in global variables)
yaw = pitch = roll = 0.0
accelx = accely = accelz = 0.0
depth = temp = bus_voltage = shunt_voltage = current = power = 0.0
surge = 0
joystick_x = joystick_y = C_roll= 0
mode = "Manual"
drive = "D"

#Notification Settings
temp_message = ""
temp_message_timestamp = 0
temp_message_duration = 0

# For picture-taking via YOLO
button = 0  # This should be updated by your system (e.g., from serial/UDP data)
SAVE_LOCATION = "/home/rov/results"  # Change this to your desired save path
if not os.path.exists(SAVE_LOCATION):
    os.makedirs(SAVE_LOCATION)

# ===============================
# UDP Network Setup
# ===============================
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# ===============================
# Serial Setup
# ===============================
BAUD_RATE = 921600
try:
    ser = serial.Serial('/dev/ttyUSB1', BAUD_RATE, timeout=1)
except serial.SerialException:
    try:
        ser = serial.Serial('/dev/ttyUSB0', BAUD_RATE, timeout=1)
    except serial.SerialException:
        ser = None

if ser:
    ser.flush()

# ===============================
# Video Capture Thread Function
# ===============================
def video_capture_thread():
    global latest_frame
    cap = cv2.VideoCapture(0)
    # Lower resolution for speed (e.g., 640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        return
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame.copy()
        time.sleep(0.02)  # Adjust capture rate as needed
    cap.release()

# ===============================
# Notification Function 
# ===============================

def display_temporary_message(text, duration=0.5):
    global temp_message, temp_message_timestamp, temp_message_duration
    temp_message = text
    temp_message_timestamp = time.time()
    temp_message_duration = duration


# ===============================
# UDP Data Reception Thread Function
# (Simply forwards UDP data to the STM32 via serial)
# ===============================
def udp_data_thread():
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            data_str = data.decode().strip()
            if ser:
                ser.write((data_str + "\n").encode('utf-8'))
        except BlockingIOError:
            pass  # No UDP data available
        except Exception:
            pass
        time.sleep(0.01)

# ===============================
# Serial Listener Thread Function
# ===============================
def serial_listener_thread():
    global yaw, pitch, roll, accelx, accely, accelz, depth, temp, bus_voltage, shunt_voltage, current, power
    global surge, joystick_x, joystick_y, C_roll, mode, drive, button
    if ser is None:
        return

    # Map numeric mode values to strings.
    mode_map = {
         1: "Manual",
         2: "Depth Hold",
         3: "Picture",
         4: "Record"
    }
    drive_mode = {
         0: "D",
         1: "R"
    }
    

    while True:
        try:
            line = ser.readline()
            if line:
                try:
                    decoded_line = line.decode('utf-8').strip()
                    # Expected CSV format:
                    # Yaw,Pitch,Roll,Accelx,Accely,Accelz,depth,temp,bus voltage,shunt voltage,current,power,throttle,x,y,dive,mode
                    parts = decoded_line.split(',')
                    if len(parts) >= 18:
                        yaw, pitch, roll = map(float, parts[0:3])
                        accelx, accely, accelz = map(float, parts[3:6])
                        depth = float(parts[6])
                        temp = float(parts[7]) - 273.0  # Convert Kelvin to Celsius
                        bus_voltage, shunt_voltage = map(float, parts[8:10])
                        current, power = map(float, parts[10:12])
                        surge = int(parts[12])
                        joystick_y, joystick_x = map(int, parts[13:15])
                        C_roll = int(parts[15])
                        mode_num = int(parts[16])
                        button = int(parts[16])
                        if mode_num != 0:
                            mode = mode_map.get(mode_num)
                        drive_num = float(parts[17])
                        drive = drive_mode.get(drive_num)
                        
                    # Else: Incomplete data; ignore.
                except Exception:
                    pass  # Skip decoding errors
        except serial.SerialException:
            break
        time.sleep(0.005)


# ===============================
# YOLO Processing Functions & Thread
# ===============================
def process_with_yolo(image, min_thresh=MIN_CONF_THRESH):
    """
    Run YOLO inference on the given image and annotate the detections.
    
    Args:
        image (np.array): Input image in BGR format.
        min_thresh (float): Minimum confidence threshold for drawing detections.
        
    Returns:
        annotated_image (np.array): Image with bounding boxes and labels drawn.
    """
    # Run inference
    results = yolo_model(image, verbose=False)
    annotated_image = image.copy()

    # Process the first result (assuming one image input)
    if results and len(results) > 0:
        detections = results[0].boxes  # Detections from the model
        for detection in detections:
            # Convert coordinates to numpy array and then to integers
            xyxy = detection.xyxy.cpu().numpy().squeeze().astype(int)
            if xyxy.ndim == 0 or len(xyxy) < 4:
                continue
            xmin, ymin, xmax, ymax = xyxy[:4]
            conf = detection.conf.item()
            class_id = int(detection.cls.item())
            label = yolo_model.names[class_id]

            # Draw detection if confidence is above threshold
            if conf >= min_thresh:
                cv2.rectangle(annotated_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                text = f"{label}: {conf:.2f}"
                cv2.putText(annotated_image, text, (xmin, max(ymin - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return annotated_image

def yolo_process_thread():
    global latest_frame, button
    while True:
        # Check if the button value indicates to take a picture (button == 8)
        if button == 8:
            display_temporary_message("Picture Taken", duration=0.8)
            # Safely copy the latest frame
            with frame_lock:
                if latest_frame is not None:
                    frame_copy = latest_frame.copy()
                else:
                    frame_copy = None
            if frame_copy is not None:
                # Create a timestamp for the filename
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                image_filename = os.path.join(SAVE_LOCATION, f"image_{timestamp}.jpg")
                cv2.imwrite(image_filename, frame_copy)
                
                # Run YOLO model processing
                result_image = process_with_yolo(frame_copy)
                result_filename = os.path.join(SAVE_LOCATION, f"image_{timestamp}_result.jpg")
                cv2.imwrite(result_filename, result_image)
            # Optional: debounce to avoid repeated triggering.
            time.sleep(1)
        time.sleep(0.1)


# ===============================
# Start Threads
# ===============================
video_thread = threading.Thread(target=video_capture_thread, daemon=True)
video_thread.start()

udp_thread = threading.Thread(target=udp_data_thread, daemon=True)
udp_thread.start()

if ser:
    serial_thread = threading.Thread(target=serial_listener_thread, daemon=True)
    serial_thread.start()

# Start YOLO processing thread
yolo_thread = threading.Thread(target=yolo_process_thread, daemon=True)
yolo_thread.start()

# ===============================
# Pygame Setup
# ===============================
pygame.init()
SCREEN_WIDTH, SCREEN_HEIGHT = 1920, 920
SENSOR_PANEL_HEIGHT = 100
CAMERA_AREA_HEIGHT = SCREEN_HEIGHT - SENSOR_PANEL_HEIGHT

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("ROV Control and Metrics")
clock = pygame.time.Clock()

sensor_font = pygame.font.SysFont(None, 16)
large_font = pygame.font.SysFont(None, 36)

# ===============================
# Main Pygame Loop
# ===============================
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            pygame.quit()
            sys.exit()

    # Draw Video Feed
    with frame_lock:
        frame = latest_frame.copy() if latest_frame is not None else None

    if frame is not None:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_surface = pygame.image.frombuffer(frame_rgb.tobytes(),
                                                (frame_rgb.shape[1], frame_rgb.shape[0]),
                                                "RGB")
        frame_surface = pygame.transform.scale(frame_surface, (SCREEN_WIDTH, CAMERA_AREA_HEIGHT))
        screen.blit(frame_surface, (0, 0))
    else:
        screen.fill((128, 128, 128), rect=pygame.Rect(0, 0, SCREEN_WIDTH, CAMERA_AREA_HEIGHT))
        no_feed_text = large_font.render("No Feed", True, (255, 255, 255))
        text_rect = no_feed_text.get_rect(center=(SCREEN_WIDTH // 2, CAMERA_AREA_HEIGHT // 2))
        screen.blit(no_feed_text, text_rect)

    # Draw Sensor Panel (all sensor data)
    sensor_panel_rect = pygame.Rect(0, CAMERA_AREA_HEIGHT, SCREEN_WIDTH, SENSOR_PANEL_HEIGHT)
    pygame.draw.rect(screen, (0, 0, 0), sensor_panel_rect)

    sensor_texts = [
        f"Yaw: {yaw:.2f}",
        f"Pitch: {pitch:.2f}",
        f"Roll: {roll:.2f}",
        f"AccelX: {accelx:.2f}",
        f"AccelY: {accely:.2f}",
        f"AccelZ: {accelz:.2f}",
        f"Depth: {depth:.2f}",
        f"Temp: {temp:.2f}",
        f"BusV: {bus_voltage:.2f}",
        f"ShuntV: {shunt_voltage:.2f}",
        f"Current: {current:.2f}",
        f"Power: {power:.2f}",
        f"Surge: {surge}",
        f"Y: {joystick_y}",
        f"X: {joystick_x}",
        f"C_roll: {C_roll}",
        f"Mode: {mode}",
        f"{drive}"
    ]
    # Display sensor data in two rows
    row1 = sensor_texts[:9]
    row2 = sensor_texts[9:]
    row1_y = CAMERA_AREA_HEIGHT + 5
    row2_y = row1_y + sensor_font.get_height() + 5
    x = 10
    for text in row1:
        rendered = sensor_font.render(text, True, (255, 255, 255))
        screen.blit(rendered, (x, row1_y))
        x += rendered.get_width() + 10
    x = 10
    for text in row2:
        rendered = sensor_font.render(text, True, (255, 255, 255))
        screen.blit(rendered, (x, row2_y))
        x += rendered.get_width() + 10

    # Draw Joystick Dot in Top-Right Corner
    dot_center = (SCREEN_WIDTH - 50, 50)
    offset_factor = 0.3
    dot_offset_x = int(joystick_x * offset_factor)
    dot_offset_y = int(joystick_y * offset_factor)
    dot_position = (dot_center[0] + dot_offset_x, dot_center[1] + dot_offset_y)
    pygame.draw.circle(screen, (255, 0, 0), dot_position, 10)

    current_time = time.time()
    if temp_message and (current_time - temp_message_timestamp < temp_message_duration):
        # Render the text message
        message_surface = large_font.render(temp_message, True, (255, 255, 255))
        
        # Define padding around the text
        padding = 10
        # Calculate the overlay size based on text dimensions and padding
        block_width = message_surface.get_width() + 2 * padding
        block_height = message_surface.get_height() + 2 * padding
        
        # Create a surface with per-pixel alpha (for transparency)
        overlay = pygame.Surface((block_width, block_height), pygame.SRCALPHA)
        # Fill it with black and an alpha value (e.g., 150 out of 255 for slight transparency)
        overlay.fill((0, 0, 0, 150))
        
        # Blit the overlay onto the main screen at the top-left corner
        screen.blit(overlay, (0, 0))
        # Then blit the text message on top of the overlay with some padding
        screen.blit(message_surface, (padding, padding))


    pygame.display.flip()
    clock.tick(120)
