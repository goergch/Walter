#!/bin/bash

logfile=/var/log/walter.log
logprefix="init-wlan-device:"
echo "`date` $logprefix init(`whoami`)" 2>&1 >> $logfile
if lsusb | grep --quiet Wireless; then
   if iwconfig wlan0 | grep --quiet ESSID; then
    	echo "`date` $logprefix connected to "`iwconfig wlan0 | grep "ESSID" | sed  's/\(.*ESSID:\)\(".*"\) \(.*\)/\2/'`"." 2>&1 >> $logfile
   else
      echo "init kernel module 8192cu"
      echo "`date` $logprefix modprobe -r 8192cu && modprobe 8192cu" 2>&1 >> $logfile
      sudo modprobe -r 8192cu && sudo modprobe 8192cu
   fi
else
   echo "`date` $logprefix no wireless device detected" 2>&1 >> $logfile
fi
exit 0

