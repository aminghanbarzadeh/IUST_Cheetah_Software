# Remote Python Joystick Publisher

This tool allows you to plug an Xbox controller into your laptop and wirelessly command the UP board running the Cheetah robot software over the network using LCM.

## Prerequisites

You need Python 3 installed on your laptop, along with the `pygame` and `lcm` packages.

### Installation

1. Ensure Python 3 and `pip` are installed on your laptop.
2. Open a terminal or command prompt and install the dependencies:
   ```bash
   pip install pygame lcm
   ```

### Running the Publisher

1. Make sure your laptop and the UP board are connected to the same network (e.g., connected to the same Wi-Fi hotspot) so that they are on the same subnet. LCM uses multicast, which requires the devices to be on the same subnet.
2. Plug your Xbox controller into your laptop.
3. Open a terminal on your laptop, navigate to this directory (`scripts/python_joystick_publisher`), and run:
   ```bash
   python publish_joystick.py
   ```
4. You should see output indicating that the joystick was detected and is publishing over LCM on the `interface` channel.
5. On the UP board, run your robot code as usual. The `HardwareBridge` will listen to the network for LCM messages on the `interface` channel, picking up your joystick commands just as if the joystick were plugged directly into the UP board!

## Troubleshooting

- **Joystick not found:** Ensure your controller is properly connected to your laptop. Check your OS device manager to ensure it is detected.
- **Robot not receiving commands:**
  - Ensure the UP board and laptop are on the same subnet. If you created a Wi-Fi hotspot with your laptop and connected the UP board to it, they are on the same subnet.
  - Make sure your firewall on your laptop isn't blocking UDP multicast traffic (LCM uses IP `239.255.76.67` on port `7667`).
