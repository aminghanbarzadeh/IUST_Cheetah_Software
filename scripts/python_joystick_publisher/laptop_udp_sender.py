import pygame
import socket
import time
import sys
import struct
import argparse

def main():
    parser = argparse.ArgumentParser(description='Send Xbox Controller inputs over direct UDP.')
    parser.add_argument('--ip', type=str, default='10.42.0.124', help='IP address of the UP board.')
    parser.add_argument('--port', type=int, default=5000, help='UDP port to send to.')
    args = parser.parse_args()

    # Initialize pygame and joystick
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Error: No joystick connected. Please connect your Xbox controller and try again.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Detected joystick: {joystick.get_name()}")
    print(f"Sending direct UDP packets to {args.ip}:{args.port}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            pygame.event.pump()

            # Read buttons
            b_a = joystick.get_button(0)
            b_b = joystick.get_button(1)
            b_x = joystick.get_button(2)
            b_y = joystick.get_button(3)
            b_lb = joystick.get_button(4)
            b_rb = joystick.get_button(5)
            b_back = joystick.get_button(6)
            b_start = joystick.get_button(7)

            try:
                b_lclick = joystick.get_button(8)
                b_rclick = joystick.get_button(9)
            except Exception:
                b_lclick = 0
                b_rclick = 0

            # Read axes
            ls_x = joystick.get_axis(0)
            ls_y = -joystick.get_axis(1)
            rs_x = joystick.get_axis(3)
            rs_y = -joystick.get_axis(4)

            # Triggers
            lt = (joystick.get_axis(2) + 1.0) / 2.0
            rt = (joystick.get_axis(5) + 1.0) / 2.0

            b_lt = 1 if lt > 0.5 else 0
            b_rt = 1 if rt > 0.5 else 0

            # Pack all this data into a binary struct
            # Format: 12 integers (buttons), 6 floats (axes/triggers)
            # 12 * 4 bytes + 6 * 4 bytes = 72 bytes total
            packet = struct.pack('<12i6f',
                b_lb, b_rb, b_lt, b_rt, b_back, b_start, b_a, b_b, b_x, b_y, b_lclick, b_rclick,
                lt, rt, ls_x, ls_y, rs_x, rs_y
            )

            sock.sendto(packet, (args.ip, args.port))

            time.sleep(0.02) # ~50Hz

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()
        sock.close()

if __name__ == '__main__':
    main()
