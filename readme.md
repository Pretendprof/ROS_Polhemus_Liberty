Summary
==============
<p>Simple programs that run wired or wireless Polhemus Liberty motion tracker as a stand alone ROS (Indigo) node. Publishes marker 1 to topic "pol_output" and marker 2 to "pol_output1". Default configuration is a msg file with marker id, x, y, z, thetaX, thetaY, thetaZ, and thetaW. Time stamp can be added if needed and output can be changed according to ascii commands in Polhemus manual (see code for template of how to do this if unsure).Default output is markerid, x, y, z, thetaX, thetaY, thetaZ. ThetaW is left blank. </p>

Using Program
------------
<p>Compile using catkin_make. Wired Polhemus Liberty can be run from rosrun using "PiServer". Wireless Polhemus liberty can be run from rosrun using "wirelessPiServer". Wireless is configured for 2 receptors and 2 markers. You will need to modify the source code for your specific use case. Note especially that there are a number of options for sensor alignment, you will need to determine the best option for your use and change code accordingly. For example, if you have 4 receptors, you'll need to either set autoalign (using commands in Polhemus manual) or manually align 3 receptors relative to the anchor receptor. Manual alignment can be somewhat automated in code by filling in and repeating the relavant commands (currently only one receptor is manually aligned). Similar considerations should taken for marker launching. Currently, in order for two markers to launch, one marker must be set in launch position behind each of the two receptors. Of course, both PiServer and wirelessPiServer can also be launched with a launch file.</p>

libusb-1.0.0
--------------
<p>Note that polhemus libraries depend on libusb-1.0.0</p>

TODO: 
------------------
<p>fix exit bug (does not exit cleanly)
add terminal prompts for more flexible setups</p>

Tested in 
---------------
<p>Ubuntu 14.04 with Ros Indigo</p>


<p>Most of the scripts are a modified version of the Linux code provided by Polhemus. </p>
<p>Some Code is based on https://github.com/ana-GT/trackerd</p>
