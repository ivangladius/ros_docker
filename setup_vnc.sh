#!/bin/bash

# Update package list and install necessary packages
apt-get update
apt-get install -y xvfb x11vnc novnc websockify ros-noetic-turtlesim

# Set up VNC password
mkdir -p /root/.vnc
x11vnc -storepasswd harun123 /root/.vnc/passwd

# Source ROS setup script
source /opt/ros/noetic/setup.bash

# Create a script to start the VNC server, noVNC, and the turtlesim application
cat << 'EOF' > /usr/local/bin/start_vnc.sh
#!/bin/bash

# Start Xvfb
Xvfb :1 -screen 0 1024x768x16 &

# Wait for Xvfb to start
sleep 2

# Start x11vnc
x11vnc -display :1 -usepw -forever -shared &

# Start websockify
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &

# Start the ROS core and turtlesim
export DISPLAY=:1
source /opt/ros/noetic/setup.bash
roscore &
sleep 5
rosrun turtlesim turtlesim_node &
EOF

# Make the script executable
chmod +x /usr/local/bin/start_vnc.sh

# Start the VNC server, noVNC, and the turtlesim application
/usr/local/bin/start_vnc.sh