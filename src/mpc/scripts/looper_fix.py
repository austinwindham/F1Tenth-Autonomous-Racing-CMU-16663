import csv
import math

input_file = "/sim_ws/src/mpc/waypoints_finalyes.csv"
output_file = "/sim_ws/src/mpc/waypoints_finalfixed.csv"

waypoints = []

# Read all waypoints from CSV
with open(input_file, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if len(row) >= 4:
            waypoints.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])

# Modify yaw values after the first yaw > 6
seen_high_yaw = False
for wp in waypoints:
    yaw = wp[2]
    if yaw > 6:
        seen_high_yaw = True
    if seen_high_yaw and yaw < 0.5:
        wp[2] += 2 * math.pi

# Write fixed waypoints back to CSV
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for wp in waypoints:
        writer.writerow(wp)

print("Fixed waypoints saved to:", output_file)
