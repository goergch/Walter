#!/bin/bash

function ensure-being-root {
   w=`whoami`
   if [ "$w" = "root" ]
   then
   		return 0
   else
		echo "I need to be root to do this."
   	    exit 1
   fi
}

# check if webserver is there
webserver=./server
if [ -e "$webserver" ] 
then
	echo "ok."
else
	echo "webserver is not there."
    exit 1
fi

# start webserver
./server

