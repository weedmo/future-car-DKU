#!/bin/bash
cd /dev && ls

echo "Enter the Arduino port (press Enter for default 'ttyACM0'):"
read port

# Check if input_variable is empty (i.e., user just pressed Enter)
if [ -z "$port" ]; then
    port="ttyACM0"  # Set default value
fi

echo "The port is set to: $port"

# rosrun rosserial_python serial_node.py _port:=$port _baud:=115200
roslaunch rocketdan init.launch port:=/dev/$port
