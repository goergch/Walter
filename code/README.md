C++ source code of Walter

* [Cortex](https://github.com/jochenalt/Walter/blob/master/code/BotCortex) 
contains the C++ Teensy code interpolating a given trajectory and running the closed loop for all actuators
* [GLUI](https://github.com/jochenalt/Walter/blob/master/code/GLUI) 
old GLUI framework to enhance openGL by buttons, labels etc.  Used by WalterPlanner.
* [freeglut](https://github.com/jochenalt/Walter/blob/master/code/freeglut) 
old library used by GLUI
* [Common](https://github.com/jochenalt/Walter/blob/master/code/WalterCommon) 
Common parts used by Cortex and Webserver, especially containing the communication protocol in between
* [Kinematics](https://github.com/jochenalt/Walter/blob/master/code/WalterKinematics) 
Library used by Webserver and Planning UI computing kinematics and trajectories
* [WalterPlanner](https://github.com/jochenalt/Walter/blob/master/code/WalterPlanner) 
OpenGL UI for planning trajectories and sending the Webserver
* [WalterServer](https://github.com/jochenalt/Walter/blob/master/code/WalterServer) 
Webserver receiving running on Odroid XU4 receiving commands from WalterPlanner
* [ServerOdroid](https://github.com/jochenalt/Walter/blob/master/code/ServerODroid) 
To be deleted, used before WalterServer was  portable for Windows and Linux
