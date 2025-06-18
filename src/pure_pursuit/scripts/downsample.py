# import csv

# input_file = "/sim_ws/src/mpc/waypoints.csv"
# output_file = "/sim_ws/src/mpc/waypointsdsplease.csv"
# n = 5  # keep every nth waypoint

# waypoints = []

# # Load the CSV
# with open(input_file, newline='') as csvfile:
#     reader = csv.reader(csvfile)
#     for row in reader:
#         if len(row) >= 2:
#             waypoints.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])

# # Downsample

# downsampled = waypoints[::n]

# # Save the new CSV
# with open(output_file, 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     for wp in downsampled:
#         writer.writerow(wp)

# print(f"Original: {len(waypoints)} points")
# print(f"Downsampled: {len(downsampled)} points (every {n}th point kept)")


import csv

input_file = "/sim_ws/src/mpc/waypoints_wrapped_final.csv"
output_file = "/sim_ws/src/mpc/waypoints_finalyes.csv"

waypoints = []

# Load the CSV
with open(input_file, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if len(row) >= 4:
            waypoints.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])

# Split into early and late segments
split_index = int(len(waypoints) * 0.6)  # First 60%, last 40%
early = waypoints[:split_index]
late = waypoints[split_index:]

# Downsample: early = every 5th, late = every 2nd
early_downsampled = early[::5]
late_downsampled = late[::3]

# Combine
final_waypoints = early_downsampled + late_downsampled

# Save to CSV
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in final_waypoints:
        writer.writerow(row)

print(f"Original: {len(waypoints)} points")
print(f"Final: {len(final_waypoints)} points")

