import json
import csv
from datetime import datetime
import numpy as np
from collections import defaultdict
import random

# Kalman Filter class for smoothing heading angle
class KalmanFilter:
    def __init__(self, process_variance=1e-3, measurement_variance=1e-1):
        self.x = 0  # Initial state (heading angle)
        self.P = 1  # Initial uncertainty
        self.Q = process_variance  # Process variance
        self.R = measurement_variance  # Measurement variance

    def update(self, measurement):
        # Prediction
        self.P = self.P + self.Q

        # Measurement update
        K = self.P / (self.P + self.R)  # Kalman gain
        self.x = self.x + K * (measurement - self.x)  # Update state
        self.P = (1 - K) * self.P  # Update uncertainty
        return self.x

# Function to calculate Euclidean distance between two points
def calculate_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

# Function to parse timestamp to datetime object
def parse_timestamp(ts):
    return datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f")

# Main fusion algorithm
def fuse_sensor_data(json_file_path, csv_file_path, output_file_path="fused_data_latest1.csv", max_distance=200.0):
    # Note: max_distance is set to 200 assuming positions are in centimeters (2 meters = 200 cm).
    # If positions are in meters, change max_distance to 2.0.
    
    # Initialize Kalman filter for heading
    kf = KalmanFilter()

    # Open files
    with open(json_file_path, 'r') as jf, open(csv_file_path, 'r') as cf, open(output_file_path, 'w', newline='') as of:
        json_data = json.load(jf)  # Load JSON data
        csv_reader = csv.DictReader(cf)  # CSV reader
        csv_data = sorted(csv_reader, key=lambda x: parse_timestamp(x['timestamp']))  # Sort CSV by timestamp
        
        # Prepare output CSV
        fieldnames = ['f_timestamp', 'f_id', 'cluster_data', 'heading', 'status']
        writer = csv.DictWriter(of, fieldnames=fieldnames)
        writer.writeheader()

        # Sort JSON data by timestamp
        json_data_sorted = sorted(json_data, key=lambda x: parse_timestamp(x['timestamp']))

        # Track fused object IDs
        fused_objects = {}  # Maps cluster centroid to f_id
        current_f_id = 0

        # Group JSON data by timestamp for clustering
        timestamped_data = defaultdict(list)
        for entry in json_data_sorted:
            timestamp = entry['timestamp']
            timestamped_data[timestamp].append(entry)

        # Process data sequentially
        csv_idx = 0
        for timestamp in sorted(timestamped_data.keys()):
            sensor_entries = timestamped_data[timestamp]

            # Sync with CSV data based on timestamp
            while csv_idx < len(csv_data) and parse_timestamp(csv_data[csv_idx]['timestamp']) < parse_timestamp(timestamp):
                csv_idx += 1
            if csv_idx >= len(csv_data):
                break
            # Use the closest previous IMU entry if no exact match
            if csv_idx == 0:
                imu_entry = csv_data[csv_idx]
            else:
                imu_entry = csv_data[csv_idx - 1]

            # Smooth heading angle using Kalman filter
            noisy_heading = float(imu_entry['heading'])
            smoothed_heading = kf.update(noisy_heading)
            status = imu_entry['state']

            # Cluster objects at this timestamp
            clusters = []
            for entry in sensor_entries:
                pos_xy = [float(entry['position_x']), float(entry['position_y'])]
                sensor_id = entry['sensor_id']
                added_to_cluster = False

                for cluster in clusters:
                    # Check distance to all points in the cluster, not just the centroid
                    can_add = True
                    for existing_pos in cluster['positions']:
                        dist = calculate_distance([existing_pos[0], existing_pos[1]], pos_xy)
                        if dist > max_distance:
                            can_add = False
                            break
                    if can_add:
                        cluster['positions'].append([pos_xy[0], pos_xy[1], sensor_id])
                        # Update centroid (average position)
                        all_positions = [p[:2] for p in cluster['positions']]
                        cluster['centroid'] = [
                            sum(p[0] for p in all_positions) / len(all_positions),
                            sum(p[1] for p in all_positions) / len(all_positions)
                        ]
                        added_to_cluster = True
                        break

                if not added_to_cluster:
                    clusters.append({
                        'centroid': pos_xy,
                        'positions': [[pos_xy[0], pos_xy[1], sensor_id]]
                    })

            # Assign f_id and write to output
            for cluster in clusters:
                centroid_key = tuple(cluster['centroid'])  # Use centroid as key for consistency
                if centroid_key not in fused_objects:
                    fused_objects[centroid_key] = current_f_id
                    current_f_id += 1
                f_id = fused_objects[centroid_key]

                writer.writerow({
                    'f_timestamp': timestamp,
                    'f_id': f_id,
                    'cluster_data': str(cluster['positions']),  # Convert to string for CSV
                    'heading': smoothed_heading,
                    'status': status
                })

# Example usage with file paths
if __name__ == "__main__":
    # Replace these paths with the actual paths to your files
    json_file_path = "camera_data_refine.json"  # e.g., "C:/Users/YourName/Documents/sensor_data.json"
    csv_file_path = "updated_data.csv"       # e.g., "C:/Users/YourName/Documents/imu_data.csv"
    
    # Run the fusion algorithm
    fuse_sensor_data(json_file_path, csv_file_path)