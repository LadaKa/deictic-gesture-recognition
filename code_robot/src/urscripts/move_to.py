import socket
import time

HOST = "192.168.0.104" 
PORT = 30002

tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.connect((HOST, PORT))
rcvd = tcp_socket.recv(1024)
print(rcvd)
start_point = 'movel(p[53, -283, 598],a=1.2, v=0.25, t=0, r=0))\n'
tcp_socket.send(str.encode(start_point))
time.sleep(3)


tcp_socket.close()