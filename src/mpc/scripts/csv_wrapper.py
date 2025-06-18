import numpy as np
import pandas as pd

def wrap_to_2pi(angle):
    return (angle + 2 * np.pi) % (2 * np.pi)

# Load your waypoint file
file_path = '/sim_ws/src/mpc/waypointsfinal.csv'  # <-- adjust if needed
df = pd.read_csv(file_path, header=None)

# Wrap the yaw values in the third column (index 2)
df[2] = df[2].apply(wrap_to_2pi)

# Save to a new file
df.to_csv('/sim_ws/src/mpc/waypoints_wrapped_final.csv', index=False, header=False)
print("Wrapped yaw values and saved to waypoints_wrapped.csv ✅")
