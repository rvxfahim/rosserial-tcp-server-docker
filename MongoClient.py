import tkinter as tk
from tkinter import scrolledtext
from pymongo import MongoClient
import threading
import time

# MongoDB connection details
mongo_uri = "mongodb://192.168.31.4:27017/"
client = MongoClient(mongo_uri)
db = client['CAN_Data']
collection = db['all_data']

# Function to get the latest data for each msg_id
def get_latest_data():
    msg_ids = collection.distinct("msg_id")
    latest_data = {}
    for msg_id in msg_ids:
        latest_doc = collection.find_one(
            {"msg_id": msg_id},
            sort=[("timestamp", -1)]
        )
        if latest_doc:
            latest_data[msg_id] = latest_doc
    return latest_data

# Function to update the GUI
def update_gui():
    while True:
        latest_data = get_latest_data()
        text_area.delete('1.0', tk.END)
        for msg_id, data in latest_data.items():
            text_area.insert(tk.END, f"msg_id: {msg_id}\n")
            for key, value in data.items():
                text_area.insert(tk.END, f"  {key}: {value}\n")
            text_area.insert(tk.END, "\n")
        time.sleep(5)  # Update every 5 seconds

# Setting up the Tkinter window
root = tk.Tk()
root.title("MongoDB Data Viewer")

text_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=40, height=10)
text_area.pack(padx=10, pady=10)

# Run the GUI update in a separate thread
thread = threading.Thread(target=update_gui)
thread.daemon = True  # Daemonize thread
thread.start()

# Start the Tkinter event loop
root.mainloop()
