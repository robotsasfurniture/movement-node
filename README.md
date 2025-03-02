# ROS 2 Movement Controller

A ROS 2 node that controls robot movement through WebSocket commands, designed to work with micro-ROS enabled robots.

## Overview

This system consists of three main components:

1. A ROS 2 movement controller node that accepts WebSocket commands
2. A micro-ROS agent that bridges communication with the robot
3. A Docker setup for easy deployment

The movement controller accepts distance and angle commands via WebSocket, converts them into appropriate velocity commands, and publishes them to the robot through ROS 2 topics.

## Prerequisites

- Docker Desktop for Mac
- Git
- Terminal access

## Quick Start

1. Clone this repository:

```bash
git clone <repository-url>
cd movement-node
```

2. Run the setup script:

```bash
./setup.sh
```

This will:

- Set up the necessary ROS 2 workspace
- Start the movement controller container
- Start the micro-ROS agent container

## Architecture

### Components

1. **Movement Controller Node**

   - Listens for WebSocket commands on port 9090
   - Publishes to ROS 2 topics:
     - `rcm/cmd_vel` (geometry_msgs/Twist): Robot velocity commands
     - `rcm/enabled` (std_msgs/Bool): Robot enable status

2. **micro-ROS Agent**
   - Runs on UDP port 8888
   - Bridges communication between ROS 2 and the micro-ROS enabled robot

### WebSocket API

The WebSocket server accepts JSON messages in the following format:

```json
{
    "distance": float,  // Distance to move in meters
    "angle": float     // Angle to turn in degrees
}
```

Response format:

```json
{
    "status": string,  // "ok", "busy", or "error"
    "message": string  // Status description
}
```

## Usage

### Starting the System

1. Run the setup script:

```bash
./setup.sh
```

2. Attach to the movement controller container:

```bash
docker attach ros2_movement
```

3. Inside the container, build and run the node:

```bash
source /opt/ros/iron/setup.bash
cd /ros2_ws && colcon build
source /ros2_ws/install/setup.bash
ros2 run movement_controller movement_controller
```

### Controlling the Robot

You can send movement commands using any WebSocket client. Example using Python:

```python
import websockets
import asyncio
import json

async def send_movement():
    uri = "ws://localhost:9090"
    async with websockets.connect(uri) as websocket:
        # Move forward 1 meter and turn 90 degrees
        command = {
            "distance": 1.0,
            "angle": 90.0
        }
        await websocket.send(json.dumps(command))
        response = await websocket.recv()
        print(response)

asyncio.run(send_movement())
```

### Movement Parameters

- Linear velocity: 0.3 m/s
- Angular velocity: 0.5 rad/s
- Movement sequence:
  1. Wait period (2 seconds)
  2. Rotation to target angle
  3. Buffer period (1 second)
  4. Forward movement to target distance
  5. Stop

## Docker Container Management

### Viewing Logs

For micro-ROS agent logs:

```bash
docker logs micro-ros-agent
```

For movement controller logs:

```bash
docker logs ros2_movement
```

### Stopping the System

To stop all containers:

```bash
docker compose down && docker stop micro-ros-agent
```

### Development

To detach from the container without stopping it:

- Press: Ctrl+P, Ctrl+Q

To reattach to the container:

```bash
docker attach ros2_movement
```

## Troubleshooting

1. **WebSocket Connection Issues**

   - Ensure port 9090 is not in use by another application
   - Check the movement controller logs for connection errors

2. **Robot Communication Issues**

   - Verify the micro-ROS agent is running (`docker logs micro-ros-agent`)
   - Ensure the robot is properly configured for micro-ROS
   - Check UDP port 8888 is accessible

3. **Build Issues**
   - Try rebuilding the container: `docker compose build --no-cache`
   - Verify all ROS 2 dependencies are installed

## License

[Your License Here]

## Contributing

[Your Contributing Guidelines Here]
