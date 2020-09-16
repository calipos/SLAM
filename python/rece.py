import socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
localaddr = ("",2368)
udp_socket.bind(localaddr)
recv_data = udp_socket.recvfrom(1206)
print(recv_data)
udp_socket.close()