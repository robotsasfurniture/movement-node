#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from example_interfaces.msg import Bool
import asyncio
import websockets
import json
import threading
import math


class MovementController(Node):
    def __init__(self):
        super().__init__("movement_controller")

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "rcm/cmd_vel", 1)
        self.enable_publisher = self.create_publisher(Bool, "rcm/enabled", 1)

        # Movement control variables
        self.movement_request = {"distance": 0, "angle": 0, "executed": False}
        self.executing = False
        self.time = 0.0

        # Timer for movement control loop
        self.loop_time_period = 1.0 / 10.0  # 10Hz
        self.loop_timer = self.create_timer(self.loop_time_period, self.loop)

        # Start WebSocket server in a separate thread
        self.ws_thread = threading.Thread(target=self.start_websocket_server)
        self.ws_thread.daemon = True
        self.ws_thread.start()

        self.get_logger().info("Movement Controller Node Started")

    def calculate_time_xyz(self, distance, velocity):
        return distance / (velocity * 1.03)

    def calculate_time_ang(self, angle, ang_velocity):
        radians = angle * math.pi / 180
        return radians / (ang_velocity * 0.9881)

    async def handle_websocket(self, websocket):
        self.get_logger().info("New WebSocket connection established")

        async for message in websocket:
            try:
                data = json.loads(message)
                # Convert linear/angular velocities to distance/angle
                distance = data.get("distance", 0.0)
                angle = data.get("angle", 0.0)

                if not self.executing:
                    self.movement_request = {
                        "distance": distance,
                        "angle": angle,
                        "executed": False,
                    }
                    self.time = 0.0
                    self.executing = True
                    await websocket.send(
                        json.dumps({"status": "ok", "message": "Movement started"})
                    )
                    self.get_logger().info(
                        f"Movement started: distance={distance}, angle={angle}"
                    )
                else:
                    await websocket.send(
                        json.dumps(
                            {
                                "status": "busy",
                                "message": "Previous movement in progress",
                            }
                        )
                    )
                    self.get_logger().info("Previous movement in progress")
            except json.JSONDecodeError:
                await websocket.send(
                    json.dumps({"status": "error", "message": "Invalid JSON"})
                )
            except Exception as e:
                await websocket.send(json.dumps({"status": "error", "message": str(e)}))

    def start_websocket_server(self):
        async def start_server():
            async with websockets.serve(self.handle_websocket, "0.0.0.0", 9090):
                await asyncio.Future()  # run forever

        try:
            asyncio.run(start_server())
        except Exception as e:
            self.get_logger().error(f"WebSocket server error: {str(e)}")

    def loop(self):
        # something to stop time from incrementing or reset it when we get a new input
        self.time = self.time + self.loop_time_period

        enableMsg = Bool()
        enableMsg.data = True
        self.enable_publisher.publish(enableMsg)

        velocityMsg = Twist()
        velocityMsg.linear.z = 0.0
        velocityMsg.angular.x = 0.0
        velocityMsg.angular.y = 0.0
        velocityMsg.linear.y = 0.0
        velocityMsg.angular.z = 0.0
        velocityMsg.linear.x = 0.0

        if self.movement_request["executed"]:
            return

        # Set speeds
        spdx = 0.3  # Linear speed in m/s
        spdang = 0.5  # Angular speed in rad/s

        # Adjust angular speed based on target direction
        if self.movement_request["angle"] < 0:
            spdang = -spdang

        # Calculate movement times
        time_xyz = self.calculate_time_xyz(self.movement_request["distance"], spdx)
        time_ang = self.calculate_time_ang(self.movement_request["angle"], spdang)
        buff = 1
        wait = 2

        time_ang = time_ang + wait

        # Set angular velocity
        if self.time <= time_ang and self.time > wait:
            velocityMsg.angular.z = spdang
        else:
            velocityMsg.angular.z = 0.0

        # Set linear velocity
        if self.time <= time_xyz + time_ang + buff and self.time > time_ang + buff:
            velocityMsg.linear.x = spdx

        # Stop the robot after movement
        if self.time > (time_xyz + time_ang + buff):
            self.movement_request["distance"] = 0
            self.movement_request["angle"] = 0
            velocityMsg.linear.x = 0.0
            velocityMsg.angular.z = 0.0
            self.time = 0  # not sure if needed
            self.executing = False
            self.movement_request["executed"] = True

        self.cmd_vel_publisher.publish(velocityMsg)


def main(args=None):
    rclpy.init(args=args)
    movement_controller = MovementController()
    rclpy.spin(movement_controller)
    movement_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
