#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading
import socket

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    send_data_to_clients(data.data)

def send_data_to_clients(data):
    disconnected_clients = []
    encoded_data = data.encode()
    data_length = len(encoded_data)
    prefixed_data = data_length.to_bytes(4, 'big') + encoded_data  # 4-byte length prefix
    for client_socket in client_sockets:
        try:
            client_socket.sendall(prefixed_data)
        except BrokenPipeError:
            rospy.logwarn("Client disconnected")
            disconnected_clients.append(client_socket)
        except Exception as e:
            rospy.logwarn("Error sending data to client: %s", e)
            disconnected_clients.append(client_socket)

    # Remove disconnected clients
    for dc in disconnected_clients:
        client_sockets.remove(dc)
        dc.close()

def listener():
    rospy.Subscriber("/CAN_Data", String, callback)
    rospy.spin()

def start_tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 3636))
    server_socket.listen(5)

    while not rospy.is_shutdown():
        try:
            client_socket, address = server_socket.accept()
            client_sockets.append(client_socket)
            print('New client connected:', address)
        except socket.error as e:
            rospy.logwarn("Socket error: %s", e)
            break

    server_socket.close()

if __name__ == '__main__':
    client_sockets = []
    rospy.init_node('listener', anonymous=True)
    listener_thread = threading.Thread(target=listener)
    server_thread = threading.Thread(target=start_tcp_server)

    listener_thread.start()
    server_thread.start()

    listener_thread.join()
    server_thread.join()