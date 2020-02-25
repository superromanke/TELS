cd ~/Projects/CAN/
sudo ./can_start.sh

while true
do
    sudo nvpmodel -m 0
    sudo ~/jetson_clocks.sh
    cd ~/Projects/TELS/
    python3 main_nearmiss.py
    sleep 5
done
