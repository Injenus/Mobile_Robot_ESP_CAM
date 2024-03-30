import socket
import time

server_ip = "192.168.4.4"
server_port = 2903

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_ip, server_port))
while True:

    data_to_send = "qwerty"+str(time.time())+"\n"
    print(data_to_send)
    client_socket.sendall(data_to_send.encode())
    time.sleep(0.05)
    # data_received = client_socket.recv(1024)
    # print("Received:", data_received.decode())

    # client_socket.close()
