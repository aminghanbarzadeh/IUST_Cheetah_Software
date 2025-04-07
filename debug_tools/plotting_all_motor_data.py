import pandas as pd
import matplotlib.pyplot as plt

# Log file path
log_file_path = "leg_controller_data.txt"

# Define column names
columns = []
# Per leg columns
for leg in range(4):
    columns.extend([
        f"leg{leg}",
        f"q0_leg{leg}", f"q1_leg{leg}", f"q2_leg{leg}",
        f"qDes0_leg{leg}", f"qDes1_leg{leg}", f"qDes2_leg{leg}",
        f"tauEst0_leg{leg}", f"tauEst1_leg{leg}", f"tauEst2_leg{leg}",
        f"tauFF0_leg{leg}", f"tauFF1_leg{leg}", f"tauFF2_leg{leg}"
    ])
# Shared joint gains
columns.extend(["kpJoint0", "kpJoint1", "kpJoint2", "kdJoint0", "kdJoint1", "kdJoint2"])
# Cartesian data and gains
for leg in range(4):
    columns.extend([
        f"p0_leg{leg}", f"p1_leg{leg}", f"p2_leg{leg}",
        f"pDes0_leg{leg}", f"pDes1_leg{leg}", f"pDes2_leg{leg}"
    ])
columns.extend(["kpCartesian0", "kpCartesian1", "kpCartesian2", "kdCartesian0", "kdCartesian1", "kdCartesian2"])
# Force feedforward
for leg in range(4):
    columns.extend([f"forceFF0_leg{leg}", f"forceFF1_leg{leg}", f"forceFF2_leg{leg}"])

# Load the log file into a DataFrame
data = pd.read_csv(log_file_path, delim_whitespace=True, names=columns)

# Debug: Display the structure of the DataFrame
print("Data loaded successfully.")
print(data.head())

# Function to plot all data for a specific leg and its joints in one figure
def plot_leg_data(leg, data):
    fig, axes = plt.subplots(5, 3, figsize=(15, 12))  # Increased to 5 rows to include force feedforward
    fig.suptitle(f"Leg {leg} Data", fontsize=16)

    # Plot joint positions and desired positions
    for joint in range(3):
        axes[0, joint].plot(data.index, data[f"q{joint}_leg{leg}"], label=f"q{joint}")
        axes[0, joint].plot(data.index, data[f"qDes{joint}_leg{leg}"], label=f"qDes{joint}", linestyle="--")
        axes[0, joint].set_title(f"Joint {joint} Position")
        axes[0, joint].legend()
        axes[0, joint].set_xlabel("Time Step")
        axes[0, joint].set_ylabel("Position")

    # Plot torques
    for joint in range(3):
        axes[1, joint].plot(data.index, data[f"tauEst{joint}_leg{leg}"], label=f"Tau Estimate {joint}")
        axes[1, joint].plot(data.index, data[f"tauFF{joint}_leg{leg}"], label=f"Tau FF {joint}", linestyle="--")
        axes[1, joint].set_title(f"Joint {joint} Torque")
        axes[1, joint].legend()
        axes[1, joint].set_xlabel("Time Step")
        axes[1, joint].set_ylabel("Torque")

    # Plot gains
    for joint in range(3):
        axes[2, joint].plot(data.index, data[f"kpJoint{joint}"], label=f"kpJoint{joint}")
        axes[2, joint].plot(data.index, data[f"kdJoint{joint}"], label=f"kdJoint{joint}", linestyle="--")
        axes[2, joint].set_title(f"Joint {joint} Gains")
        axes[2, joint].legend()
        axes[2, joint].set_xlabel("Time Step")
        axes[2, joint].set_ylabel("Gain")

    # Plot Cartesian positions and desired positions
    for i, axis_label in enumerate(["X", "Y", "Z"]):
        axes[3, i].plot(data.index, data[f"p{i}_leg{leg}"], label=f"p{i}")
        axes[3, i].plot(data.index, data[f"pDes{i}_leg{leg}"], label=f"pDes{i}", linestyle="--")
        axes[3, i].set_title(f"Cartesian Position {axis_label}")
        axes[3, i].legend()
        axes[3, i].set_xlabel("Time Step")
        axes[3, i].set_ylabel("Position")

    # Plot force feedforward
    for i, axis_label in enumerate(["X", "Y", "Z"]):
        axes[4, i].plot(data.index, data[f"forceFF{i}_leg{leg}"], label=f"forceFF{i}", color='r')
        axes[4, i].set_title(f"Force Feedforward {axis_label}")
        axes[4, i].legend()
        axes[4, i].set_xlabel("Time Step")
        axes[4, i].set_ylabel("Force")

    # Adjust layout
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

# Plot data for all legs
for leg in range(4):
    plot_leg_data(leg, data)
