#!/bin/bash

shutdown() {
	echo "***Shutting down...***"
	kill -INT $PID1
	kill -INT $PID2
	kill -INT $PID3
	kill -INT $PID4
	echo "***Waiting 1 second for processes to terminate...***"
	sleep 1
	kill -9 $PID1
	kill -9 $PID2
	kill -9 $PID3
	kill -9 $PID4
	echo "***Exiting.***"
	exit 1
}

if [ "$1" = 'sim' ]
then
    echo '***Starting simulation mode***'
    #simulation mode
    echo '***Starting system state service***'
    python Common/system_state_service.py &
    PID1=$!
    echo "***PID = $PID1***"

    echo '***Starting haptic task service***'
    python UI/haptic_widget.py &
    PID2=$!
    echo "***PID = $PID2***"

    echo '***Starting simulation***'
    /home/motion/Klampt/SimTest /home/motion/Klampt/data/baxterserial.xml &
    PID3=$!
    echo "***PID = $PID3***"

    #test to see if everything is running OK
    if ! ps -p $PID1 > /dev/null
    then
	echo '***System state service not running properly?***'
	shutdown
    fi
    if ! ps -p $PID2 > /dev/null
    then
	echo '***Haptic service not running properly?***'
	shutdown
    fi
    if ! ps -p $PID3 > /dev/null
    then
	echo '***Simulation service not running properly?***'
	shutdown
    fi

    echo '***Starting controller***'
    ./ControllerService /home/motion/Klampt/data/baxterserial.xml

    echo "***Killed, quitting system state and haptic services, quitting sim***"
    shutdown
else
    echo '***Starting robot mode***'
    #robot mode
    echo '***Starting system state service***'
    python Common/system_state_service.py &
    PID1=$!
    echo "***PID = $PID1***"

    echo '***Starting haptic task service***'
    python UI/haptic_widget.py &
    PID2=$!
    echo "***PID = $PID2***"

    echo '***Enabling robot***'
    rosrun baxter_tools enable_robot.py -e &
    PID3=$!
    echo "***PID = $PID3***"

    echo '***Starting serial->Baxter (ROS) relay***'
    rosrun klampt baxterserialrelay.py -m k2b -r /home/motion/Klampt/data/robots/baxter.rob &
    PID4=$!
    echo "***PID = $PID4***"

    sleep 1
    #test to see if everything is running OK
    if ! ps -p $PID1 > /dev/null
    then
	echo '***System state service not running properly?***'
	shutdown
    fi
    if ! ps -p $PID2 > /dev/null
    then
	echo '***Haptic service not running properly?***'
	shutdown
    fi
    if ! ps -p $PID4 > /dev/null
    then
	echo '***Serial->Baxter relay service not running properly?***'
	shutdown
    fi

    echo '***Starting controller***'
    ./ControllerService /home/motion/Klampt/data/baxterserial.xml

    echo "***Killed, quitting services system state, haptic, serial->baxter***"
    shutdown
fi