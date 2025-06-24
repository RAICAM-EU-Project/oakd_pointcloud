# oakd_pointcloud

`oakd_pointcloud` is a ROS 2 package that publishes **RGB images**, **depth images**, and **colored 3D point clouds** from a [Luxonis OAK-D](https://docs.luxonis.com/en/latest/pages/products/bw1098oak/) camera in real time using the DepthAI API.

---

## ✅ Features

- Publishes:
  - `/oakd/rgb/image_raw` – RGB image (`sensor_msgs/Image`)
  - `/oakd/depth/image_raw` – Depth image in millimeters (`mono16`)
  - `/oakd/pointcloud` – Colored `sensor_msgs/PointCloud2`
  - `/oakd/rgb/camera_info` – Camera intrinsics (`sensor_msgs/CameraInfo`)
  - `/tf` – Static transform from `oakd_link` → `oakd_rgb_camera_frame`
- Real-time performance:
  - **RGB/Depth**: ~15 Hz
  - **PointCloud2**: ~2 Hz (downsampled for speed)
- Based on **DepthAI**, compatible with [OAK-D](https://docs.luxonis.com/)

---

## 📦 Installation

```bash
cd ~/raicam_ws/src
git clone <this-repo-url> oakd_pointcloud
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select oakd_pointcloud
source install/setup.bash
```

## 🚀 Usage
Run the node:

```bash
ros2 run oakd_pointcloud oakd_pointcloud_publisher
```

Or launch it:

```bash
ros2 launch oakd_pointcloud oakd_pointcloud.launch.py
```

## 🧪 Visualization

Use rviz2 to visualize:

```bash
rviz2
```
    Set Fixed Frame: oakd_link

    Add:

        PointCloud2 → /oakd/pointcloud

        Image → /oakd/rgb/image_raw

        TF → enable to see camera frame

## 🛠️ Notes

    Replace hardcoded intrinsics (fx, cx, etc.) with real calibration values if needed.

    The point cloud is downsampled (stride = 4) for performance.
