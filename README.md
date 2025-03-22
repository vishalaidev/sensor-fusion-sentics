# sensor-fusion-sentics

## Sensor Fusion Explanation

### What Does This Code Do?
The code fuses data from two different sensors:

1. **Camera Data (JSON file)** – Contains object positions.
2. **IMU Data (CSV file)** – Provides the heading angle and system status.

**Goal:**
- Combine (fuse) this data based on time and object positions.
- Cluster objects that are close to each other.
- Smooth the heading angle using a **Kalman Filter**.
- Save the fused results to a CSV file (`fused_data_latest1.csv`).

---

### Files Used:
1. **Input Files:**
   - `camera_data_refine.json` – Contains object positions and timestamps.
   - `updated_data.csv` – Contains heading angle, status, and timestamps.

2. **Output File:**
   - `fused_data_latest1.csv` – The fused result.

---

### File Formats:

#### 1. JSON (Camera Data) – Example:
```json
[
  {
    "timestamp": "2025-03-22 12:00:00.123",
    "sensor_id": "cam_1",
    "object_class": "car",
    "position_x": 10.5,
    "position_y": 20.3
  },
  {
    "timestamp": "2025-03-22 12:00:00.123",
    "sensor_id": "cam_2",
    "object_class": "person",
    "position_x": 11.0,
    "position_y": 21.1
  }
]
```

#### 2. CSV (IMU Data) – Example:
```
timestamp,heading,state
2025-03-22 12:00:00.100,45.3,active
2025-03-22 12:00:00.200,46.1,active
```

#### 3. Output CSV (Fused Data) – Example:
```
f_timestamp,f_id,cluster_data,heading,status
2025-03-22 12:00:00.123,0,"[[10.5, 20.3, 'cam_1'], [11.0, 21.1, 'cam_2']]",45.5,active
```

---

### How the Code Works (Step by Step)

#### 1. Kalman Filter Class
This class smooths the **heading angle**. The IMU data gives a noisy (inaccurate) heading, so the Kalman filter reduces noise.

**How it works:**
- `update()` method takes a noisy heading.
- It estimates a smoother value by balancing **measurement** and **prediction**.

#### 2. Helper Functions

- **`calculate_distance()`** – Computes the Euclidean distance between two points:  
  \[ \text{Distance} = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2} \]

- **`parse_timestamp()`** – Converts timestamp text into a datetime object for easy comparison.

#### 3. Main Fusion Function: `fuse_sensor_data()`

✅ **Step 1: Read Input Files**
- Load **camera data (JSON)** and **IMU data (CSV)**.
- Sort both datasets by timestamp to ensure synchronization.

✅ **Step 2: Initialize Variables**
- **`kf`** – Kalman filter for smoothing heading.
- **`fused_objects`** – Keeps track of existing clusters and their unique IDs (`f_id`).
- **`current_f_id`** – Counter for assigning new object IDs.

✅ **Step 3: Process Data by Timestamps**

1. **Match Camera and IMU Data**  
   - For each timestamp in the camera data, find the closest IMU entry.  
   - If no exact match is found, take the most recent IMU record.

2. **Smooth Heading**  
   - Use the Kalman filter to reduce noise in the heading.

✅ **Step 4: Cluster Objects**
- For each object at the current timestamp:
   1. Check if it is within **200 cm (2 meters)** of an existing cluster.  
   2. If yes → Add it to that cluster and update the cluster center.  
   3. If no → Create a new cluster.

✅ **Step 5: Assign `f_id` to Clusters**
- Each unique cluster is given a fixed **f_id**.
- If a cluster is new, assign a new `f_id`.

✅ **Step 6: Write Fused Data**
- Save the fused information to the output CSV.

---

