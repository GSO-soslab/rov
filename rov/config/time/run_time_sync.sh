#!/bin/bash

echo "Stop chrony and gpsd now !"
sudo systemctl stop chronyd
sudo systemctl stop gpsd

echo "Set serial port !"
sudo chmod 666 /dev/ttyTHS0
rate=$(sudo stty -F /dev/ttyTHS0 speed)


while [ $rate != 115200 ]
do
  sleep 2 
  sudo stty -F /dev/ttyTHS0 115200 
  sleep 2 
  rate=$(sudo stty -F /dev/ttyTHS0 speed)
  echo "  Set to 115200, get=$rate"
done


sudo systemctl start chronyd
sudo systemctl start gpsd

echo "Fishined, waitting for sync."
echo "check using 'chronyc sources -v'"
exit
