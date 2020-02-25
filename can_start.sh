sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-bcm
sudo modprobe can-gw  
sudo modprobe can_dev
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 500000 
sudo ip link set up can0
sudo ip link set can1 type can bitrate 500000 
sudo ip link set up can1
