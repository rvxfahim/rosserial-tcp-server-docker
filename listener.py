#!/usr/bin/env python3
import asyncio
import json
import time
import threading
import websockets
import rospy
from std_msgs.msg import String
from pymongo import MongoClient, errors
# Constants
SEND_INTERVAL_MS = 300  # Example rate limit interval in milliseconds
last_send_times = {}  # Dictionary to store the last send time for each msg_id

# MongoDB setup
try:
    client = MongoClient('mongodb://mongodb:27017/', serverSelectionTimeoutMS=5000)
    client.admin.command('ismaster')
    print("MongoDB connection established!")
except errors.ServerSelectionTimeoutError:
    print("Unable to connect to the MongoDB server. Check your connection settings and ensure the server is running.")

db = client['CAN_Data']
collection = db['all_data']

# Asyncio queues for document types
queue_with_data = asyncio.Queue()
queue_without_data = asyncio.Queue()

async def batch_writer(queue):
    buffer = []
    last_doc_time = None

    while True:
        try:
            # Wait for new doc or timeout
            doc = await asyncio.wait_for(queue.get(), timeout=1)
            buffer.append(doc)
            last_doc_time = time.time()
        except asyncio.TimeoutError:
            if last_doc_time and time.time() - last_doc_time >= 1:
                if buffer:
                    collection.insert_many(buffer)
                    buffer.clear()
                last_doc_time = None
        else:
            if len(buffer) >= 100:
                collection.insert_many(buffer)
                buffer.clear()

def should_send_for_msg_id(msg_id):
    current_time_ms = time.time_ns() // 1_000_000
    if msg_id not in last_send_times or (current_time_ms - last_send_times[msg_id]) >= SEND_INTERVAL_MS:
        last_send_times[msg_id] = current_time_ms
        return True
    return False

def callback(data):
    try:
        # Split the received data into individual JSON strings
        json_strings = data.data.split('\n')[:-1]  # Assuming '\n' is used as a delimiter
        for json_str in json_strings:
            parsed_data = json.loads(json_str)
            msg_id = parsed_data.get('msg_id')
            # parsed_data['timestamp'] = time.time_ns() // 1_000_000
            # print(parsed_data)
            if msg_id is not None and should_send_for_msg_id(msg_id):
                asyncio.run_coroutine_threadsafe(send_data_to_clients(json_str), loop)
            
            if 'data' in parsed_data:
                asyncio.run_coroutine_threadsafe(queue_with_data.put(parsed_data), loop)
            else:
                asyncio.run_coroutine_threadsafe(queue_without_data.put(parsed_data), loop)
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON: {e}")
        print(f"Received data: {data.data}")  # Print the received data


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
    rospy.Subscriber("/Unknown_Data", String, callback)
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
    loop = asyncio.get_event_loop()

    # Start the ROS listener in a separate thread
    listener_thread = threading.Thread(target=listener)
    listener_thread.start()

    # Start batch writers
    loop.create_task(batch_writer(queue_with_data))
    loop.create_task(batch_writer(queue_without_data))

    # Run the WebSocket server in the main thread
    loop.run_until_complete(start_websocket_server())
