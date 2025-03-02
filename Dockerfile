FROM ros:iron

# Install Python and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    ros-iron-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Install required Python packages
RUN pip3 install --no-cache-dir \
    websockets \
    asyncio

# Create workspace directory
WORKDIR /ros2_ws

# Source ROS2 setup in .bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Set default command to bash
CMD ["/bin/bash"] 