import tkinter as tk
from tkinter import ttk, simpledialog
import json
import threading
import asyncio
import websockets
import traceback

async def receive_messages_async(server_uri, update_gui_callback, on_connection_error_callback):
    retry_count = 0
    while True:
        try:
            async with websockets.connect(server_uri) as websocket:
                retry_count = 0  # Reset retry count on successful connection
                while True:
                    try:
                        data = await websocket.recv()
                        message = json.loads(data)
                        update_gui_callback(message['msg_id'], message)
                    except Exception as e:
                        print(f"Error receiving data: {e} of type {type(e).__name__}")
                        traceback.print_exc()
                        break
        except Exception as e:
            print(f"Error connecting to server: {e} of type {type(e).__name__}")
            traceback.print_exc()
            retry_count += 1
            on_connection_error_callback(retry_count)
            if retry_count > 5:
                print("Maximum retry count reached. Exiting...")
                break
            else:
                await asyncio.sleep(5)  # Wait for 5 seconds before retrying

def receive_messages(server_uri):
    asyncio.run(receive_messages_async(
        server_uri,
        lambda msg_id, message: root.after(0, update_list, msg_id, message),
        lambda retry_count: root.after(0, show_retry_dialog, retry_count)
    ))

def unpack_signal(can_buffer, signal_info, dlc):
    bit_position = signal_info['start_bit_position']
    bit_offset = bit_position % 8
    current_byte_index = bit_position // 8
    remaining_bits = signal_info['length']

    signal_value = 0
    shift_amount = 0

    while remaining_bits > 0 and current_byte_index < dlc:
        bits_in_current_byte = min(remaining_bits, 8 - bit_offset)
        bit_mask = (1 << bits_in_current_byte) - 1
        extracted_bits = (can_buffer[current_byte_index] >> bit_offset) & bit_mask
        signal_value |= extracted_bits << shift_amount

        remaining_bits -= bits_in_current_byte
        shift_amount += bits_in_current_byte
        bit_offset = 0
        current_byte_index += 1 if signal_info['order'] == 'INTEL' else -1

    # Adjust for signed values
    if signal_info['signed'] and (signal_value & (1 << (signal_info['length'] - 1))):
        signal_value -= (1 << signal_info['length'])

    return signal_value

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
    # Assuming comboboxes[0], comboboxes[1], comboboxes[2], comboboxes[3] are the ones to check
    return all(combo.get() for combo in comboboxes[:4])

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

def update_list(msg_id, message):
    # Exclude 'msg_id' from the data to be displayed
    data_to_display = {k: v for k, v in message.items() if k != 'msg_id'}

    if msg_id in msg_dict:
        # Update existing message
        message_widget = msg_dict[msg_id]
        display_text = f"ID: {msg_id}, " + ", ".join([f"{k}: {v}" for k, v in data_to_display.items()])
        message_widget['label'].config(text=display_text)  # Update label text
        message_frame = message_widget['frame']  # Access the frame from the stored dictionary
    else:
        # Add new message frame
        display_text = f"ID: {msg_id}, " + ", ".join([f"{k}: {v}" for k, v in data_to_display.items()])
        message_frame, message_widget = add_message_frame(msg_id, display_text)
        msg_dict[msg_id] = message_widget
        msg_dict[msg_id]['frame'] = message_frame  # Store the frame in the dictionary for later access

    # Iterate through each group_frame in the message_frame
    for group_frame in message_frame.winfo_children():
        if isinstance(group_frame, tk.Frame):
            children = group_frame.winfo_children()
            if children:
                text_box = children[0]  # The first child is the text box
                comboboxes = [widget for widget in children if isinstance(widget, ttk.Combobox)]
                # Check for a specific key in the message for signal processing (e.g., 'data')
                if 'data' in message and are_comboboxes_set(comboboxes):
                    new_value = get_new_value_for_textbox(comboboxes[0].get(), comboboxes[1].get(), comboboxes[2].get(), comboboxes[3].get(), message['data'])
                    text_box.delete(0, tk.END)  # Clear the existing text
                    text_box.insert(0, new_value)  # Insert the new value

    update_scroll_region()

def get_new_value_for_textbox(startbit, length, byteorder, signedness, data):
    # Convert data to a bytearray
    data = bytearray(data)

    # Convert startbit and length to integers
    startbit = int(startbit)
    length = int(length)

    # Convert byteorder to a boolean
    byteorder = True if byteorder == "Intel" else False

    # Convert signedness to a boolean
    signedness = True if signedness == "Signed" else False

    # Unpack the signal
    signal_info = {
        'start_bit_position': startbit,
        'length': length,
        'order': 'INTEL' if byteorder else 'MOTOROLA',
        'signed': signedness
    }
    return unpack_signal(data, signal_info, len(data))

async def receive_messages_async(server_uri, update_gui_callback, on_connection_error_callback):
    retry_count = 0
    while True:
        try:
            async with websockets.connect(server_uri) as websocket:
                retry_count = 0  # Reset retry count on successful connection
                while True:
                    try:
                        data = await websocket.recv()
                        message = json.loads(data)
                        update_gui_callback(message['msg_id'], message)
                    except Exception as e:
                        print(f"Error receiving data: {e} of type {type(e).__name__}")
                        traceback.print_exc()
                        break
            # Connection closed, call error callback
            on_connection_error_callback(retry_count)
        except Exception as e:
            print(f"Error connecting to server: {e} of type {type(e).__name__}")
            traceback.print_exc()
            retry_count += 1
            if retry_count > 5:
                print("Maximum retry count reached. Exiting...")
                break
            else:
                print("Retrying connection...")
                await asyncio.sleep(5)  # Wait for 5 seconds before retrying

def start_client_thread(server_uri):
    client_thread = threading.Thread(target=receive_messages, args=(server_uri,), daemon=True)
    client_thread.start()

def show_retry_dialog(retry_count):
    response = simpledialog.askstring("Connection Lost", f"Connection lost. Retrying... (Attempt {retry_count})\nChange server URI or cancel to stop:")
    if response is not None:
        start_client_thread(response)  # Start with new URI
    else:
        print("User canceled the connection retry.")

def show_connection_config():
    server_uri = simpledialog.askstring("Connection Configuration", "Enter server URI:", initialvalue="ws://localhost:3636")
    if server_uri:
        start_client_thread(server_uri)

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

menu_bar = tk.Menu(root)
root.config(menu=menu_bar)

file_menu = tk.Menu(menu_bar, tearoff=0)
menu_bar.add_cascade(label="File", menu=file_menu)
file_menu.add_command(label="Configure Connection", command=show_connection_config)

msg_dict = {}  # Dictionary to track message IDs and their list index

show_connection_config()  # Open connection configuration on start

root.mainloop()