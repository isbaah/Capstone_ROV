import sys
import pygame
import socket
import threading
import serial  # For serial communication
import cv2
import numpy as np
import time

# ===============================
# Global Variables and Settings
# ===============================
# For video feed
latest_frame = None
frame_lock = threading.Lock()

# ROV sensor state variables (all sensor data stored in global variables)
yaw = pitch = roll = 0.0
accelx = accely = accelz = 0.0
depth = temp = bus_voltage = shunt_voltage = current = power = 0.0
throttle = 0
joystick_x = joystick_y = dive = 0
mode = "Manual"

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
BAUD_RATE = 19200
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
    global throttle, joystick_x, joystick_y, dive, mode
    if ser is None:
        return

    # Map numeric mode values to strings.
    mode_map = {
         1: "Manual",
         2: "Depth Hold",
         3: "Picture",
         4: "Record"
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
                    if len(parts) >= 17:
                        yaw = float(parts[0])
                        pitch = float(parts[1])
                        roll = float(parts[2])
                        accelx = float(parts[3])
                        accely = float(parts[4])
                        accelz = float(parts[5])
                        depth = float(parts[6])
                        temp = float(parts[7])-273.0
                        bus_voltage = float(parts[8])
                        shunt_voltage = float(parts[9])
                        current = float(parts[10])
                        power = float(parts[11])
                        throttle = int(parts[12])
                        joystick_x = int(parts[13])
                        joystick_y = int(parts[14])
                        dive = int(parts[15])
                        mode_num = int(parts[16])
                        mode = mode_map.get(mode_num, "Unknown")
                    # Else: Incomplete data; ignore.
                except Exception:
                    pass  # Skip decoding errors
        except serial.SerialException:
            break
        time.sleep(0.005)

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

# ===============================
# Pygame Setup
# ===============================
pygame.init()
SCREEN_WIDTH, SCREEN_HEIGHT = 1920, 1080
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
        f"Throttle: {throttle}",
        f"X: {joystick_x}",
        f"Y: {joystick_y}",
        f"Dive: {dive}",
        f"Mode: {mode}"
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

    pygame.display.flip()
    clock.tick(120)