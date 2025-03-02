#!/bin/bash

# Create ros2_ws directory in home if it doesn't exist
mkdir -p ~/ros2_ws/src

# Copy the movement controller package to ros2_ws
cp -r ros2_ws/src/movement_controller ~/ros2_ws/src/

# Build and start the movement controller container
docker compose build --no-cache  # Ensure we get the latest dependencies
docker compose up -d

# Start the micro-ROS agent container
echo "Starting micro-ROS agent..."
docker run -d --rm --net=host --name micro-ros-agent microros/micro-ros-agent:iron udp4 --port 8888

# Print instructions
echo "Both containers are running!"
echo ""
echo "micro-ROS agent is running on UDP port 8888"
echo ""
echo "To attach to the movement controller, run:"
echo "docker attach ros2_movement"
echo ""
echo "Once inside the movement controller, run:"
echo "source /opt/ros/iron/setup.bash"
echo "cd /ros2_ws && colcon build"
echo "source /ros2_ws/install/setup.bash"
echo "ros2 run movement_controller movement_controller"
echo ""
echo "To detach from the container without stopping it, press: Ctrl+P, Ctrl+Q"
echo ""
echo "To view micro-ROS agent logs:"
echo "docker logs micro-ros-agent"
echo ""
echo "To stop everything:"
echo "docker compose down && docker stop micro-ros-agent" 