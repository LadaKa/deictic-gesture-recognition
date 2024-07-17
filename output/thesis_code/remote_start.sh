#!/bin/bash



REMOTE_FILE='/home/ladak/Desktop/result.txt'
REMOTE_ROS_SCRIPT='/home/ladak/Desktop/WS_2024/src/remote_commands/remote_launch.sh'

REMOTE_PC=ladak@10.42.0.88
MAX_TIME=10

FILE_CONTENT=$(cat /home/robot/Desktop/result.txt)

echo "Starting ssh connection."
echo "$FILE_CONTENT" | sshpass -p “ros” ssh $REMOTE_PC 'cat > /home/ladak/Desktop/result.txt'
echo "File content transferred:"

echo "$FILE_CONTENT"

sshpass -p "ros" ssh ladak@10.42.0.88 "cd /home/ladak/Desktop/WS_2024/src/remote_commands; ./remote_launch.sh"

exit

