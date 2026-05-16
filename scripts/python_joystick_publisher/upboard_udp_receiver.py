import socket
import struct
import lcm
from lcm_types.gamepad_lcmt import gamepad_lcmt
import argparse

def main():
    parser = argparse.ArgumentParser(description='Receive direct UDP joystick commands and publish via local LCM.')
    parser.add_argument('--port', type=int, default=5000, help='UDP port to listen on.')
    args = parser.parse_args()

    # Listen on all interfaces
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))

    # Initialize local LCM
    lc = lcm.LCM()

    print(f"Listening for direct UDP packets on port {args.port}...")
    print("Publishing to local LCM channel 'interface'.")

    try:
        while True:
            data, addr = sock.recvfrom(1024)

            # Unpack the 72 byte struct
            if len(data) == 72:
                unpacked = struct.unpack('<12i6f', data)

                msg = gamepad_lcmt()
                msg.leftBumper = unpacked[0]
                msg.rightBumper = unpacked[1]
                msg.leftTriggerButton = unpacked[2]
                msg.rightTriggerButton = unpacked[3]
                msg.back = unpacked[4]
                msg.start = unpacked[5]
                msg.a = unpacked[6]
                msg.b = unpacked[7]
                msg.x = unpacked[8]
                msg.y = unpacked[9]
                msg.leftStickButton = unpacked[10]
                msg.rightStickButton = unpacked[11]

                msg.leftTriggerAnalog = unpacked[12]
                msg.rightTriggerAnalog = unpacked[13]
                msg.leftStickAnalog = [unpacked[14], unpacked[15]]
                msg.rightStickAnalog = [unpacked[16], unpacked[17]]

                lc.publish("interface", msg.encode())
            else:
                print(f"Received malformed packet of size {len(data)} from {addr}")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
