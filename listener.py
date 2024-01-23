#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading
import websockets
import asyncio
import json
from pymongo import MongoClient
from pymongo.errors import ServerSelectionTimeoutError
import time
# Create a MongoDB client
try:
    client = MongoClient('mongodb://192.168.31.4:27017/', serverSelectionTimeoutMS=5000)
    # The ismaster command is cheap and does not require auth.
    client.admin.command('ismaster')
    print("MongoDB connection established!")
except ServerSelectionTimeoutError:
    print("Unable to connect to the MongoDB server. Check your connection settings and ensure the server is running.")

# Connect to your database
db = client['CAN_Data']
# Choose your collection
collection = db['all_data']
def callback(data):
    parsed_data = json.loads(data.data)
    parsed_data['timestamp'] = time.time_ns() // 1_000_000  # Add timestamp key
    # print(parsed_data)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # Insert the data into MongoDB
    collection.insert_one(parsed_data)
    asyncio.run(send_data_to_clients(data.data))

async def send_data_to_clients(data):
    disconnected_clients = []
    for ws in websocket_clients:
        try:
            await ws.send(data)
        except websockets.exceptions.ConnectionClosed:
            rospy.logwarn("Client disconnected")
            disconnected_clients.append(ws)

    for dc in disconnected_clients:
        websocket_clients.remove(dc)

def listener():
    rospy.Subscriber("/CAN_Data", String, callback)
    rospy.spin()

async def start_websocket_server():
    async with websockets.serve(handler, '0.0.0.0', 3636):
        await asyncio.Future()  # Run forever

async def handler(websocket, path):
    websocket_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        websocket_clients.remove(websocket)

if __name__ == '__main__':
    websocket_clients = set()
    rospy.init_node('listener', anonymous=True)

    # Start the ROS listener in a separate thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    # Run the WebSocket server in the main thread
    asyncio.run(start_websocket_server())