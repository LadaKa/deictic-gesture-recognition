#!/bin/bash

# $1 ... local file (content will be copied to remote file)
# $2 ... remote filename 



REMOTE_PC=ladak@10.42.0.88
REMOTE_FILE='/home/ladak/Desktop/result.txt'  #TODO

FILE_CONTENT=$(cat /home/robot/Desktop/result.txt) #TODO
 
echo "Starting ssh connection."


cat /home/robot/Desktop/result.txt | ssh $REMOTE_PC 'cat > /home/ladak/Desktop/result.txt'

echo "File content copy done."
	
# exit
