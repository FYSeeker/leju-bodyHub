#! /bin/bash

# setup for bodyhub
sudo chmod 666 /dev/ttyUSB0
if [ $? -ne 0 ]; then
    echo "Please connect the serial port to the USB port!"
    exit 0
fi

setserial /dev/ttyUSB0 low_latency
if [ $? -ne 0 ]; then
    echo "Set up low_latency failed!"
    exit 0
fi
# locate vrep.sh and the vrep scene file
sudo updatedb

vrep=$(locate vrep.sh)
scenename=$(locate */ikmodule/60cm_talos_rbd1.ttt)

if [ ! $vrep ]; then
    echo "Could not locate vrep!"
    exit 1
fi

if [ ! $scenename ]; then
    echo "Could not locate 60cm_talos_rbd1.ttt!"
    exit 2
fi

# open vrep and load the simulation scene
$vrep $scenename &

# open another terminal and launch the simulation
gnome-terminal -x bash -c "roslaunch bodyhub bodyhub.launch sim:=true " &

wait
echo "BodyHub and Simulation stopped!"
