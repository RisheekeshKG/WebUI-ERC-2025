# Team Aurora ‑ ROS 2 Jazzy Web UI Setup

These instructions get you from a blank ROS 2 Jazzy install to a running browser‑based dashboard 

## 1  Source ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
```

---

## 2  Create a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 3  Add rosbridge_suite and web_video_server

```bash
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git
git clone -b ros2 https://github.com/RobotWebTools/web_video_server.git

```

Install deps & rebuild:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash           # re‑source after build
```

---

## 4  Run rosbridge WebSocket ( 1st Terminal )

make sure u are in ros workspace u created 'ros2_ws' in my case 
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# → “Rosbridge WebSocket server started on port 9090”
```

---

## 5  Run Web_video_server ( 2st Terminal )

in same directory (ros2_ws) in different terminal run this 

```bash
ros2 run web_video_server web_video_server

# Streams now on http://localhost:8080/stream?topic=/image_raw

```
---

## 7  Publish Laptop Webcam (`/dev/video0`) with v4l2 (3rd Terminal )

> **Install once**  
> `sudo apt install ros-jazzy-v4l2-camera`

```bash
ros2 run v4l2_camera v4l2_camera_node                       
```
---

## 8  Create Web UI Folder & Clone this Repo

```bash
mkdir Web-UI
cd ~/Web-UI
git clone https://github.com/RisheekeshKG/WebUI-ERC-2025.git
```

## 9  Serve the Web UI (4th Terminal)

```bash
cd ~/Web-UI
python3 -m http.server 8000
```

Open **http://localhost:8000/index.html** ⇒ You should see:

- **Connected ✅**  
- Live webcam image
---

