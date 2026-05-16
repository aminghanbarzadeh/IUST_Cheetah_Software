import pygame
import lcm
import time
import sys
from lcm_types.gamepad_lcmt import gamepad_lcmt

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Error: No joystick connected. Please connect your Xbox controller and try again.")
    sys.exit(1)

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Detected joystick: {joystick.get_name()}")

# Initialize LCM
lc = lcm.LCM()

print("Publishing joystick inputs over LCM on channel 'interface'...")

try:
    while True:
        # Process pygame events (required to update joystick state)
        pygame.event.pump()

        # Create the LCM message
        msg = gamepad_lcmt()

        # In pygame for Xbox controllers:
        # Buttons usually map as:
        # A: 0, B: 1, X: 2, Y: 3, Left Bumper: 4, Right Bumper: 5, Back: 6, Start: 7, LStickBtn: 8, RStickBtn: 9
        # These indices may vary slightly by OS/driver. Adjust if needed.
        msg.a = joystick.get_button(0)
        msg.b = joystick.get_button(1)
        msg.x = joystick.get_button(2)
        msg.y = joystick.get_button(3)
        msg.leftBumper = joystick.get_button(4)
        msg.rightBumper = joystick.get_button(5)
        msg.back = joystick.get_button(6)
        msg.start = joystick.get_button(7)

        try:
            msg.leftStickButton = joystick.get_button(8)
            msg.rightStickButton = joystick.get_button(9)
        except Exception:
            msg.leftStickButton = 0
            msg.rightStickButton = 0

        # Axes usually map as:
        # 0: LStick X (left -> right)
        # 1: LStick Y (up -> down)
        # 2: LTrigger
        # 3: RStick X
        # 4: RStick Y
        # 5: RTrigger

        msg.leftStickAnalog = [joystick.get_axis(0), -joystick.get_axis(1)]
        msg.rightStickAnalog = [joystick.get_axis(3), -joystick.get_axis(4)]

        left_trigger = joystick.get_axis(2)
        right_trigger = joystick.get_axis(5)

        # Trigger outputs are usually -1.0 (unpressed) to 1.0 (fully pressed) in pygame
        msg.leftTriggerAnalog = (left_trigger + 1.0) / 2.0
        msg.rightTriggerAnalog = (right_trigger + 1.0) / 2.0

        msg.leftTriggerButton = 1 if msg.leftTriggerAnalog > 0.5 else 0
        msg.rightTriggerButton = 1 if msg.rightTriggerAnalog > 0.5 else 0

        # Publish the message
        lc.publish("interface", msg.encode())

        # Sleep to run at ~50 Hz
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.quit()
