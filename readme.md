Summary
==============
Simple program that runs wired Polhemus Liberty motion tracker as a stand alone ROS (Indigo) node. Publishes to topic "pol_output". 
Default configuration is a msg file with marker id, x, y, z, thetaX, thetaY, thetaZ, and thetaW. Time stamp can be added if needed.
Default output is markerid, x, y, z, thetaX, thetaY, thetaZ. ThetaW is left blank. 

Using Program
------------
Complie using catkin_make. Can be launched with launch file. or just run "PiServer" with rosrun.

libusb-1.0.0
--------------
Note that polhemus libraries depend on libusb-1.0.0

TODO: 
------------------
fix exit bug (does not exit cleanly)
test and update with wireless polhemus

Tested in 
---------------
Ubuntu 14.04 with Ros Indigo


Most of the scripts are a modified version of the Linux code provided by Polhemus. 
Some Code is based on https://github.com/ana-GT/trackerd
