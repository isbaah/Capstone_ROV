import pygame
import socket

# Setup UDP socket
UDP_IP = "192.168.10.3"  # Replace with Raspberry Pi's IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512)  # Reduce buffer size
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
sock.setsockopt(socket.IPPROTO_UDP, socket.IP_TOS, 0x10)  # Set low-latency flag

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Check if joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect a joystick and restart.")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Limit frame rate
clock = pygame.time.Clock()
UPDATE_RATE = 200  # FPS cap at 200

print("Joystick connected. Press 'q' to quit.")

running = True
while running:
    clock.tick(UPDATE_RATE)  # Regulate loop speed efficiently

    # Process events (only necessary ones)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            running = False

    # Read joystick axes
    surge = int((joystick.get_axis(5)+1)*50)   # Left stick Y (Throttle)
    roll = max(min(int(joystick.get_axis(0) * 100), 100), -100)  # Left stick X (Roll)
    direction_x = max(min(int(joystick.get_axis(2) * 100), 100), -100)  # Right stick X
    direction_y = int((joystick.get_axis(4)+1)*50) # Right stick Y


    # Read buttons efficiently using bitwise OR
    button_state = sum(joystick.get_button(i) << i for i in range(6))  # Encode buttons as a bitmask

    # Pack data in a compact format (binary encoding reduces transmission time)
    data_bytes = f"{surge},{roll},{direction_x},{direction_y},{button_state}".encode()

    # Send data over UDP
    sock.sendto(data_bytes, (UDP_IP, UDP_PORT))

    # Debugging (optional: remove or comment out for speed)
    print("Sent:", data_bytes.decode())

# Cleanup
pygame.quit()
sock.close()
print("Program exited gracefully.")
