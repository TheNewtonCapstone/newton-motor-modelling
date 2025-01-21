import pandas as pd
import matplotlib.pyplot as plt
import sys


# Define target value
# file_path = 'log/step_responses/csv/r_4.2'
file_path = sys.argv[1]
# Read the CSV file
df = pd.read_csv(file_path)
print(df.head())

# Set up the plotting style
# plt.style.use('seaborn')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
fig.tight_layout(pad=3.0)

# Plot 1: Velocity vs Time with target line
ax1.plot(df["time"], df["amplitude"], linewidth=2, color="#2E86C1", label="Amplitude")
# ax1.plot(df['current_time'], df['output'], linewidth=2, color='#27AE60', label='Output')
ax1.set_xlabel("Time")
ax1.set_ylabel("Amplitude")
ax1.set_title("Amplitude vs Time")
ax1.grid(True)
ax1.legend()

# Plot 2: Error vs Time
ax2.plot(df["time"], df["position"], linewidth=2, color="#E74C3C", label="Position")
ax2.plot(
    df["time"],
    df["target_position"],
    linewidth=2,
    color="#27AE60",
    label="Target Position",
)
ax2.plot(
    df["time"],
    df["velocity"],
    linewidth=2,
    color="#2E86C1",
    label="Velocity",
)

ax2.set_xlabel("Time")
ax2.set_ylabel("Position")
ax2.set_title("Position vs Time")
ax2.grid(True)
ax2.legend()

# Plot 3: Output vs Error
# ax3.plot(df['current_time'], df['normalized_error'], alpha=0.6, color='#27AE60', label='Normalized Error')
# ax3.plot(df['current_time'], df['output'], alpha=0.6, color='#E74C3C', label='Output')
# velocity = df['current_velocity'] / VELOCITY_RANGE
# ax3.plot(df['current_time'], velocity, alpha=0.6, color='#2E86C1', label='Velocity')
# NORMALIZED_TARGET_VELOCITY = TARGET_VELOCITY / VELOCITY_RANGE
# ax3.axhline(y=NORMALIZED_TARGET_VELOCITY, color='#E74C3C', linestyle='--', label='Target Velocity')
# ax3.set_xlabel('Time')
# ax3.set_ylabel('Output')

# ax3.grid(True)
# ax3.legend()

# Add overall title
plt.suptitle(
    "Amplitude Response of the System",
    fontsize=16,
    y=1.02,
)

# Save the plot
plt.show()

# exit(1)
