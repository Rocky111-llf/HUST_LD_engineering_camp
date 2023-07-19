#!/bin/bash
#Author: David Hu
#Date: 2022-07

# Exit on error
set -e 

echo "Instructions:"
echo "  [product_type] = LD06/STL06P"
echo "  [serial_port_number] = usb device mount files,example:/dev/ttyUSB0."
echo "For Example:"
echo "  input [product_type]=LD06"
echo "  input [serial_port_number]=/dev/ttyUSB0"

read -p "input [product_type]=" PRODUCT_TYPE
read -p "input [serial_port_number]=" PORT_PATH

if [ ! ${PRODUCT_TYPE} ]
then
  echo "ERROR [product_type] input is null"
  exit 0
fi

LOG_NAME="test`date +%Y%m%d-%H-%M`.log"
sudo chmod 777 ${PORT_PATH}
echo "start node exec"
echo "output log to ./${LOG_NAME}"
if [ ! -e "./log" ]
then
  mkdir log
  echo "create ./log/"
fi
./build/ldlidar_stl ${PRODUCT_TYPE} serialcom ${PORT_PATH} > ./log/${LOG_NAME} 
