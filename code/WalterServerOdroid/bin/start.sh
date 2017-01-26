#!/bin/bash

logfile=/var/log/walter.log
logprefix="start-walter:"
echo "`date` $logprefix init(`whoami`)" 2>&1 >> $logfile

function ensure-being-root {
   w=`whoami`
   if [ "$w" = "root" ]
   then
   		return 0
   else
		echo "`date` $logprefix I need to be root to do this." 2>&1 >> $logfile
		echo "I need to be root to do this."
   	    exit 1
   fi
}

#check if wifi is up and running
if lsusb | grep --quiet Wireless; then
		if iwconfig wlan0 | grep --quiet ESSID; then
			echo "`date` $logprefix connected to "`iwconfig wlan0 | grep "ESSID" | sed  's/\(.*ESSID:\)\(".*"\) \(.*\)/\2/'`"." 2>&1 >> $logfile
		else
	  		sudo modprobe -r 8192cu && sudo modprobe 8192cu
	  		sleep 10
			if iwconfig wlan0 | grep --quiet ESSID; then
	  			echo "`date` $logprefix connected to "`iwconfig wlan0 | grep "ESSID" | sed  's/\(.*ESSID:\)\(".*"\) \(.*\)/\2/'`"."  2>&1 >> $logfile
			else
	  			echo "`date` $logprefix wlan0 not connected" 2>&1 >> $logfile
	  			echo "wlan0 not connected" 
	  		fi
		fi
else 
   echo "`date` $logprefix no wireless device found"
   exit 1
fi

# check if webserver is there
webserver=~/WalterServer/server
if ! [ -e "$webserver" ] ; then
	echo "`date` $logprefix $webserver is not there." 2>&1 $logfile
	echo "$webserver is not there." 

    exit 1
fi

# start webserver
cd ~/WalterServer
$webserver

