import asyncio
import websockets

async def test_socket():
    uri = "ws://192.168.31.12:6547"
    async with websockets.connect(uri) as websocket:
        # Send a message
        await websocket.send("Hello, World!")
        print("Sent: 'Hello, World!'")

        # Receive echo
        response = await websocket.recv()
        print(f"Received: {response}")

# Run the test
asyncio.get_event_loop().run_until_complete(test_socket())