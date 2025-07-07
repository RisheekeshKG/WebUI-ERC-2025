# Step 1: Source ROS 2 Jazzy Environment

Open a new terminal:

source /opt/ros/jazzy/setup.bash

</br>

# Step 2: Create a ROS 2 Workspace (if not already)

<ul><li>mkdir -p ~/ros2_ws/src</li>
<li>cd ~/ros2_ws</li>
<li>colcon build</li>
<li>source install/setup.bash</li>
</ul>
</br>

# Step 3: Clone and Build rosbridge_suite (needed for Jazzy)

In the src folder:

<li>cd ~/ros2_ws/src</li>
<li>git clone https://github.com/RobotWebTools/rosbridge_suite.git </li>
</br>

# Step 4: Install Dependencies for rosbridge_suite

<li>cd ~/ros2_ws</li>
<li>rosdep install --from-paths src --ignore-src -r -y</li>

    If rosdep throws an error, fix it with:

<li>sudo apt install python3-rosdep</li>
<li>sudo rosdep init</li>
<li>rosdep update</li>
</br>

# Step 5: Build the Workspace

<li>colcon build</li>
<li>source install/setup.bash</li>
</br>

# Step 6: Launch rosbridge WebSocket Server

<li>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</li>

You should see output like:

[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
</br>

# Step 7: Test the ROS Topic with a Publisher (New Terminal)

Open a new terminal and run:

<li>source /opt/ros/jazzy/setup.bash</li>
<li>ros2 topic pub /status_text std_msgs/String "data: 'Hello from ROS 2 Jazzy'" --rate 1</li>
</br>

# Step 8: Create a Web UI Folder

<li>mkdir -p ~/ros2_ws/web_ui</li>
<li>cd ~/ros2_ws/web_ui</li>
</br>

# Step 9: Create the HTML File

<li>nano index.html</li>
</br>

# Step 10: Serve the HTML File (New Terminal)

<li>cd ~/ros2_ws/web_ui</li>
<li>python3 -m http.server 8000</li>
</br>

# Step 11: Open the Web Interface in Browser

Go to this URL in your browser:

http://localhost:8000/index.html

✅ You should see:

    “Connected!” status

    The message from /status_text updating in real time