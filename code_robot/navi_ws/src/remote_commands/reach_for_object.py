import socket
import time

HOST = "192.168.0.104" 
PORT = 29999

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((HOST, PORT))
rcvd = s.recv(4096)
print(rcvd)
time.sleep(10)

commLoad = 'load reach_for_object.urp\n'
print(commLoad)
s.send((commLoad).encode())
rcvd = s.recv(4096)
print(rcvd)
time.sleep(10)

commPlay = 'play\n'
print(commPlay)
s.send((commPlay).encode())
rcvd = s.recv(4096)
print(rcvd)

s.close
