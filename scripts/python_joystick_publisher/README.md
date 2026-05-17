# Remote Python Joystick Publisher

This tool allows you to plug an Xbox controller into your laptop and wirelessly command the UP board running the Cheetah robot software over the network.

Because Wi-Fi hotspots often drop or block LCM UDP Multicast packets, we provide two methods. **Method B (Direct UDP)** is recommended as it bypasses all multicast issues.

## Prerequisites

You need Python 3 installed on your laptop, along with the `pygame` and `lcm` packages.
The UP board also requires the `lcm` python package.

### Installation

On **both** your laptop and the UP board:
```bash
pip install pygame lcm
```

---

## Method A: Direct UDP (Recommended)

This method guarantees delivery by sending raw UDP packets from your laptop directly to the UP board's IP address, and then having the UP board translate it to local LCM.

1. **On the UP board**, navigate to this directory and start the receiver:
   ```bash
   python upboard_udp_receiver.py
   ```
   *This script listens for the network packets and publishes them to the local `interface` LCM channel so the robot code can read it.*

2. **On your laptop**, navigate to this directory, plug in the Xbox controller, and run:
   ```bash
   # Replace the IP address with the UP board's actual IP
   python laptop_udp_sender.py --ip 10.42.0.124
   ```

3. **On the UP board**, run your robot C++ controller as usual.

---

## Method B: Pure LCM Multicast (Legacy)

If you have a network router that perfectly supports UDP multicast, you can run just one script on your laptop.

1. Make sure your laptop and the UP board are connected to the same network.
2. Plug your Xbox controller into your laptop.
3. Open a terminal on your laptop, navigate to this directory, and run:
   ```bash
   python publish_joystick.py
   ```
4. You should see output indicating that the joystick was detected.
5. On the UP board, run your robot code. The `HardwareBridge` will listen to the network for the multicast LCM messages.

*If the robot does not receive commands, use **Method A** instead.*
