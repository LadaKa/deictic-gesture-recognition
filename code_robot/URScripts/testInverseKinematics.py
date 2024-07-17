import socket
import time

ROBOT_IP = '192.168.0.104'
ROBOT_PORT = 30002

print('Starting program')
s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((ROBOT_IP,ROBOT_PORT))
time.sleep(3)

f = open('/home/ladak/Desktop/URScripts/ok_testInverseKinematics.txt', 'rb')

#f = open('/home/ladak/Desktop/URScripts/test_if.txt', 'rb')
text = f.read(1024)
print(text.decode())
s.sendall(text)
time.sleep(5)
print('Script text sent')

data=s.recv(1024)
s.close()
#print('Received 2: ',str(data.decode()))
#print('Received 2: ',str(data.decode('utf-8')))
#
print('Received 2: ',repr(data))
#print('Received 2: ',str(data.decode('utf-8')))