import tkinter as tk
from tkinter import ttk
import json
import threading
import socket

def add_signal_widgets(message_frame):
    # Create a new frame for the group of widgets
    group_frame = tk.Frame(message_frame)
    group_frame.grid(sticky='ew')  # Use grid instead of pack

    # Configure grid columns
    group_frame.grid_columnconfigure(0, weight=1)  # Make the text box column expandable

    # Text Box
    text_box = tk.Entry(group_frame)
    text_box.grid(row=0, column=0, sticky='w', padx=5)

    # Combo Boxes with different choices
    start_bit_choices = [str(i) for i in range(64)]
    length_choices = [str(i) for i in range(1, 65)]
    byte_order_choices = ["Intel", "Motorola"]
    signedness_choices = ["Signed", "Unsigned"]

    labels = ["Start bit", "Length", "Byte Order", "Signedness"]
    choices = [start_bit_choices, length_choices, byte_order_choices, signedness_choices]

    comboboxes = []
    label_col = 1  # Starting column for the labels and comboboxes
    for i in range(4):
        label = tk.Label(group_frame, text=labels[i])
        label.grid(row=0, column=label_col, sticky='w', padx=5)
        
        # Create and grid each combobox
        combo = ttk.Combobox(group_frame, values=choices[i])
        combo.grid(row=0, column=label_col + 1, sticky='w', padx=5)
        comboboxes.append(combo)

        label_col += 2

    # Bind function to length combobox
    def on_length_selected(event):
        if comboboxes[1].get() == "1":
            comboboxes[3].set("Unsigned")

    comboboxes[1].bind("<<ComboboxSelected>>", on_length_selected)

    # Bind function to signedness combobox
    def on_signedness_selected(event):
        if comboboxes[1].get() == "1":
            comboboxes[3].set("Unsigned")

    comboboxes[3].bind("<<ComboboxSelected>>", on_signedness_selected)

    # Delete Signal Button
    delete_button = tk.Button(group_frame, text="Delete Signal", command=lambda: [group_frame.destroy(), update_scroll_region()])
    delete_button.grid(row=0, column=label_col, sticky='w', padx=5)

    update_scroll_region()

def are_comboboxes_set(comboboxes):
    # Assuming comboboxes[0], comboboxes[1], comboboxes[2] are the ones to check
    return all(combo.get() for combo in comboboxes[:3])

def add_message_frame(msg_id, data):
    # Create a main frame for each message
    main_frame = tk.Frame(message_list_frame)
    main_frame.pack(fill=tk.X, expand=True)

    # Use a grid layout inside the main frame
    main_frame.grid_columnconfigure(1, weight=1)  # Make the second column (message label) expandable

    # Message Label with fixed width and wraplength
    label = tk.Label(main_frame, text=f"ID: {msg_id}, Data: {data}", width=40, wraplength=500, anchor="w")
    label.grid(row=0, column=1, sticky="nw", padx=(10, 5))

    # Add Signal Button
    add_button = tk.Button(main_frame, text="Add Signal", command=lambda: add_signal_widgets(main_frame))
    add_button.grid(row=0, column=2, padx=5)

    return main_frame, {"label": label, "button": add_button}

def update_scroll_region():
    canvas.update_idletasks()  # Update the layout
    canvas.config(scrollregion=canvas.bbox("all"))  # Update the scroll region

def update_list(msg_id, data):
    if msg_id in msg_dict:
        # Update existing message
        message_widget = msg_dict[msg_id]
        message_widget['label'].config(text=f"ID: {msg_id}, Data: {data}")  # Update label text
        message_frame = message_widget['frame']  # Access the frame from the stored dictionary
    else:
        # Add new message frame
        message_frame, message_widget = add_message_frame(msg_id, data)
        msg_dict[msg_id] = message_widget
        msg_dict[msg_id]['frame'] = message_frame  # Store the frame in the dictionary for later access

    # Iterate through each group_frame in the message_frame
    for group_frame in message_frame.winfo_children():
        if isinstance(group_frame, tk.Frame):
            children = group_frame.winfo_children()
            if children:
                text_box = children[0]  # The first child is the text box
                comboboxes = [widget for widget in children if isinstance(widget, ttk.Combobox)]
                if are_comboboxes_set(comboboxes):
                    new_value = get_new_value_for_textbox()  # Function to get the new value
                    text_box.delete(0, tk.END)  # Clear the existing text
                    text_box.insert(0, new_value)  # Insert the new value

    update_scroll_region()

def get_new_value_for_textbox():
    # Implement logic to determine the new value for the text box
    # For example, it could be based on the latest received message or some calculation
    return "New Value"

def receive_messages():
    SERVER_IP = '192.168.31.4'
    SERVER_PORT = 3636

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect((SERVER_IP, SERVER_PORT))

        while True:
            try:
                data_length = int.from_bytes(client_socket.recv(4), 'big')
                if data_length:
                    data = bytearray()
                    while len(data) < data_length:
                        packet = client_socket.recv(data_length - len(data))
                        if not packet:
                            break
                        data.extend(packet)
                    message = json.loads(data.decode())
                    root.after(0, update_list, message['msg_id'], message['data'])
            except Exception as e:
                print("Error receiving data:", e)
                break

def start_client_thread():
    client_thread = threading.Thread(target=receive_messages, daemon=True)
    client_thread.start()

# Tkinter GUI setup
root = tk.Tk()
root.title("Message Viewer")

# Set minimum width
root.minsize(width=400, height=300)

# Create a canvas with a scrollbar
canvas = tk.Canvas(root)
canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

# Add a scrollbar to the canvas
scrollbar = ttk.Scrollbar(root, orient=tk.VERTICAL, command=canvas.yview)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

# Configure the canvas to use the scrollbar
canvas.configure(yscrollcommand=scrollbar.set)
canvas.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

# Create a frame inside the canvas for the message list
message_list_frame = tk.Frame(canvas)

# Add the message list frame to the canvas
canvas.create_window((0, 0), window=message_list_frame, anchor="nw")

msg_dict = {}  # Dictionary to track message IDs and their list index

start_client_thread()  # Start the client thread

root.mainloop()