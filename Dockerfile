FROM ros:humble

# Install Python and pip
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install required Python packages
RUN pip3 install websockets

# Create workspace directory
WORKDIR /ros2_ws

# Copy the movement controller package
COPY . /ros2_ws/src/movement_controller/

# Build the ROS2 workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash && \
    colcon build'

# Source the ROS2 workspace in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Command to run when container starts
CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run movement_controller movement_controller"] 