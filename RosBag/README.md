# 🎒 RosBag Directory

This folder contains all ROS 2 bag recordings. **All new bag recordings must be saved inside this folder.**

---

## 📁 Folder Structure

```
RosBag/
├── README.md               ← You are here
├── my_bag_1/
│   ├── metadata.yaml
│   └── my_bag_1_0.db3
├── my_bag_2/
│   ├── metadata.yaml
│   └── my_bag_2_0.db3
└── ...
```

---

## ⏺️ Recording Bags

Always use the `-o` flag with a full path to make sure bags are saved here. Make sure your robot/nodes are running before recording so all topics are active.

**Record a single topic:**
```bash
ros2 bag record -o /workspace/RosBag/my_bag /topic_name
```

**Record multiple topics:**
```bash
ros2 bag record -o /workspace/RosBag/my_bag /topic1 /topic2 /topic3
```

**Record all topics at once:**
```bash
ros2 bag record -o /workspace/RosBag/my_bag -a
```

**Record with a time limit (auto-stops after 60 seconds):**
```bash
ros2 bag record -o /workspace/RosBag/my_bag -a -d 60
```

> Stop recording at any time with `Ctrl+C`.

**Don't know your topic names? List them first:**
```bash
ros2 topic list        # list all topics
ros2 topic list -t     # list topics with message types
```

---

## 🗜️ Compression (Recommended)

The workspace has limited disk space (~6.9 GB free). Always use compression to save space.

**Record all topics with compression:**
```bash
ros2 bag record -o /workspace/RosBag/my_bag -a --compression-mode file --compression-format zstd
```

**Record specific topics with compression:**
```bash
ros2 bag record -o /workspace/RosBag/my_bag --compression-mode file --compression-format zstd /topic1 /topic2
```

**Two compression modes:**

| Mode | Description |
|------|-------------|
| `file` | Compresses the whole `.db3` file — better compression, recommended |
| `message` | Compresses each message individually — faster to seek |

**Approximate uncompressed sizes (per minute):**

| Topic Type | Approx Size/min |
|------------|----------------|
| Pose, Twist, small msgs at ~30Hz | 1–5 MB |
| IMU at 200Hz | 10–50 MB |
| LiDAR pointclouds at 10Hz | 100–500 MB |
| Camera images (1080p) at 30Hz | 500 MB–2 GB |

**Check disk space before recording:**
```bash
df -h
```

**Monitor disk space in real time while recording:**
```bash
watch df -h
```

---

## ℹ️ Inspecting a Bag

Get a full summary of a bag (topics, message count, duration, size):
```bash
ros2 bag info RosBag/my_bag
```

Example output:
```
Files:             my_bag_0.db3
Bag size:          50.2 MiB
Storage id:        sqlite3
Duration:          30.5s
Start:             Jan  1 2000 01:46:46
End:               Jan  1 2000 02:17:16
Messages:          3050
Topic information: Topic: /topic1 | Type: std_msgs/msg/String | Count: 1525
                   Topic: /topic2 | Type: sensor_msgs/msg/Image | Count: 1525
```

> **Note:** If the timestamp shows `Jan 1 2000` it means the robot's clock wasn't synced — the data is still valid and will play back correctly.

---

## ▶️ Playing Back a Bag

When you play a bag, ROS 2 **re-publishes all recorded messages back onto their original topics** as if the robot were running live. Any node subscribed to those topics will receive the data normally.

**Terminal 1 — Play the bag:**
```bash
ros2 bag play RosBag/my_bag
```

**Terminal 2 — Echo the topic at the same time:**
```bash
ros2 topic echo /topic_name
```

> Stop playback at any time with `Ctrl+C`.

### 🎛️ Useful Play Flags

| Flag | Description | Example |
|------|-------------|---------|
| `--rate` | Play at a different speed (1.0 = normal) | `--rate 0.5` |
| `--start-offset` | Skip the first N seconds | `--start-offset 10` |
| `--loop` | Loop the bag continuously | `--loop` |
| `--topics` | Only replay specific topics | `--topics /topic1 /topic2` |
| `--clock` | Publish a `/clock` topic for time sync | `--clock` |
| `--delay` | Wait N seconds before starting playback | `--delay 3` |

### 🧪 Playback Examples

**Play at half speed:**
```bash
ros2 bag play RosBag/my_bag --rate 0.5
```

**Play at double speed:**
```bash
ros2 bag play RosBag/my_bag --rate 2.0
```

**Skip the first 10 seconds:**
```bash
ros2 bag play RosBag/my_bag --start-offset 10
```

**Only replay specific topics:**
```bash
ros2 bag play RosBag/my_bag --topics /odom /cmd_vel
```

**Loop continuously:**
```bash
ros2 bag play RosBag/my_bag --loop
```

**Play with clock (useful for nodes that rely on `/clock`):**
```bash
ros2 bag play RosBag/my_bag --clock
```

**Combine flags — play at half speed, skip 5 seconds, loop:**
```bash
ros2 bag play RosBag/my_bag --rate 0.5 --start-offset 5 --loop
```

---

## 🐍 Reading Bags in Python

### Option 1 — Using the `rosbag2_py` API (recommended)

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "/workspace/RosBag/my_bag"

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', '')
)

# Print all topic metadata
topic_types = reader.get_all_topics_and_types()
for t in topic_types:
    print(f"Topic: {t.name} | Type: {t.type}")

# Read and deserialize messages
type_map = {t.name: t.type for t in topic_types}

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    msg_type = get_message(type_map[topic])
    msg = deserialize_message(data, msg_type)
    print(f"[{timestamp}] {topic}: {msg}")
```

### Option 2 — Raw SQLite (quick and simple)

```python
import sqlite3

db_path = "/workspace/RosBag/my_bag/my_bag_0.db3"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# List all topics stored in the bag
cursor.execute("SELECT * FROM topics")
for row in cursor.fetchall():
    print(row)

# Read the first 10 messages
cursor.execute("SELECT timestamp, topic_id, data FROM messages LIMIT 10")
for row in cursor.fetchall():
    print(row)

conn.close()
```

---

## 🔧 Fixing Permission Issues

If you get `Permission denied` when moving bags into this folder:

```bash
# Take ownership of the RosBag folder
sudo chown -R $USER:$USER /workspace/RosBag/

# Then move normally
mv rosbag2_*/ /workspace/RosBag/
```

---

## 🗑️ Deleting Old Bags

Since disk space is limited, delete bags you no longer need:

```bash
rm -rf /workspace/RosBag/old_bag_name
```

Check space after deleting:
```bash
df -h
```

---

## 📌 Quick Reference

```bash
ros2 topic list                                                    # list available topics
ros2 bag record -o RosBag/my_bag /topic                           # record a topic
ros2 bag record -o RosBag/my_bag -a                               # record all topics
ros2 bag record -o RosBag/my_bag -a -d 60                         # record all topics for 60s
ros2 bag record -o RosBag/my_bag -a --compression-mode file \
  --compression-format zstd                                        # record with compression
ros2 bag info RosBag/my_bag                                        # inspect a bag
ros2 bag play RosBag/my_bag                                        # play a bag
ros2 bag play RosBag/my_bag --rate 0.5                            # play at half speed
ros2 bag play RosBag/my_bag --topics /topic1                      # play specific topics
ros2 topic echo /topic_name                                        # read a topic during playback
watch df -h                                                        # monitor disk space
```
