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

1. Make sure your laptop and the UP board are connected to the same network (e.g., connected to the same Wi-Fi hotspot) so that they are on the same subnet.
2. Plug your Xbox controller into your laptop.
3. Open a terminal on your laptop, navigate to this directory (`scripts/python_joystick_publisher`), and run:
   ```bash
   python publish_joystick.py
   ```
4. You should see output indicating that the joystick was detected and is publishing over LCM on the `interface` channel.
5. On the UP board, run your robot code as usual. The `HardwareBridge` will listen to the network for LCM messages on the `interface` channel.

## Troubleshooting Network Issues

LCM uses UDP Multicast by default (`udpm://239.255.76.67:7667`). However, Wi-Fi hotspots or your laptop's network routing might block or misroute multicast packets. If the UP board doesn't receive the messages (e.g., the robot does not move), try one of the following methods to force LCM to route the packets directly to the UP board's subnet:

**Method 1: Use the `--lcm-url` argument (Recommended)**
If the UP board's IP address is `10.42.0.124`, its subnet is `10.42.0.0/24`. You can force LCM to multicast *only* on that specific network interface by providing the laptop's IP address on that network (e.g., if your laptop is `10.42.0.1`, use that).
```bash
python publish_joystick.py --lcm-url "udpm://239.255.76.67:7667?ttl=1"
```
*(Setting `ttl=1` often helps force multicast packets out across the local network).*

If that still fails, you can force it to route out via the specific network interface by adding the `recv_iface` parameter:
```bash
# Replace 10.42.0.X with YOUR LAPTOP'S IP address on the hotspot
python publish_joystick.py --lcm-url "udpm://239.255.76.67:7667?ttl=1&recv_iface=10.42.0.X"
```

**Method 2: Set the `LCM_DEFAULT_URL` environment variable**
You can also set the default URL before running the script:
- Linux/macOS: `export LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"`
- Windows CMD: `set LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1`
- Windows PowerShell: `$env:LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=1"`

**Verifying Pygame Mapping:**
If the network is working but the robot does the wrong actions, your laptop OS might map Xbox buttons differently. You can uncomment the `print(f"A:{msg.a} B:{msg.b} ...")` line inside `publish_joystick.py` to see what values are actually being pressed.
