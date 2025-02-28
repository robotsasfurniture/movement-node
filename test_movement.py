import asyncio
import websockets
import json


async def send_movement():
    uri = "ws://localhost:9090"
    async with websockets.connect(uri) as websocket:
        # Example movement command
        movement = {
            "distance": 1.0,  # Move forward 1 meter
            "angle": 90.0,  # Turn 90 degrees
        }

        await websocket.send(json.dumps(movement))
        response = await websocket.recv()
        print(f"Received response: {response}")


asyncio.get_event_loop().run_until_complete(send_movement())
