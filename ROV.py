import sys
import pygame
import socket
import threading
import serial  # For serial communication
from pygame.locals import *
import time 

# Network setup
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # Non-blocking mode

# Serial setup
BAUD_RATE = 19200
try:
    ser = serial.Serial('/dev/ttyUSB1', BAUD_RATE, timeout=1)
    print("Connected to /dev/ttyUSB1")
except serial.SerialException:
    try:
        ser = serial.Serial('/dev/ttyUSB0', BAUD_RATE, timeout=1)
        print("Connected to /dev/ttyUSB0")
    except serial.SerialException:
        print("Error: Could not connect to /dev/ttyUSB1 or /dev/ttyUSB0")
        ser = None
ser.flush()

# Initialize pygame
pygame.init()
pygame.display.set_caption('ROV Control and Metrics')
screen = pygame.display.set_mode((800, 600), 0, 32)
clock = pygame.time.Clock()

# ROV State Variables
throttle = 0
mode = "Manual"  # Modes: Manual, Depth Hold, Record, Capture
pitch = 0
yaw = 0
speed = 0

# Square representation
my_square = pygame.Rect(50, 50, 50, 50)
my_square_color = 0
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

def receive_data():
    global throttle, mode, pitch, yaw, speed, my_square
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            data_str = data.decode().strip()
            values = list(map(int, data_str.split(',')))

            # Convert list to a comma-separated string and send via serial
            formatted_values = ','.join(map(str, values)) + '\n'
            ser.write(formatted_values.encode('utf-8'))  # Encode string to bytes

            
            if len(values) == 5:
                throttle_input, direction_y, direction_x, _, button = values

                speed += (throttle_input / 100.0 - speed) * 0.1
                my_square.x += int(direction_x * 0.1)
                my_square.y += int(direction_y * 0.1)

                throttle = int(speed * 100)

                if button:
                    if button == 1:
                        mode = "Depth Hold"
                    elif button == 2:
                        mode = "Record"
                    elif button == 3:
                        mode = "Capture"
                    else:
                        mode = "Manual"
                
                
        except BlockingIOError:
            pass
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break

def listen_to_serial():
    while True:
        try:
            serial_data = ser.readline()

            try:
                decoded_data = serial_data.decode('utf-8').strip()
                if decoded_data:
                    print("Received from Serial:", decoded_data)
            except UnicodeDecodeError as e:
                print(f"Decode error ignored: {e}")
                continue  # Skip this iteration and continue reading

        except serial.SerialException as e:
            print(f"Serial read error: {e}")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")

threading.Thread(target=receive_data, daemon=True).start()
threading.Thread(target=listen_to_serial, daemon=True).start()

while True:
    screen.fill((0, 0, 0))
    pygame.draw.rect(screen, colors[my_square_color], my_square)

    font = pygame.font.SysFont(None, 36)
    throttle_text = font.render(f'Throttle: {throttle}', True, (255, 255, 255))
    mode_text = font.render(f'Mode: {mode}', True, (255, 255, 255))
    screen.blit(throttle_text, (10, 10))
    screen.blit(mode_text, (10, 50))

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()
            if event.key == K_SPACE:
                my_square_color = (my_square_color + 1) % len(colors)

    pygame.display.update()
    clock.tick(120)
