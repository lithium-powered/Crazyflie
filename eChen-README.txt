[Created on 10/9/2014]
[Updated on 10/10/2014]

A) SETUP WORKSPACE

1. Create a workspace for catkin:
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

2. Move the crazyflie repo into the /src of the catkin (ros) workspace

3. Compile the package in your catkin workspace with "catkin_make"

4. After successful compling,  type "roscore" in a terminal window

5. And open another terminal window, type "rosrun [the package name of crazyflie - "crazyflie"] crazyflie_node.py", then the program runs 

TROUBLESHOOT
1. If missing python libraries when running "rosrun crazyflie crazyflie_node.py":
	a. Clone "https://github.com/jlamyi/crazyflie_old_library.git" into ~/scripts directory
	b. Clone "https://github.com/bitcraze/crazyradio-firmware.git" into ~/scr directory
	c. Manually install dependencies: http://wiki.bitcraze.se/projects:crazyflie:pc_utils:install
2. If "rosrun crazyflie crazyflie_node.py" does not work:
	a. Run "gedit ~/.bashrc" in your home directory and add:
		export /opt/ros/electric/setup.bash
		export ROS_PACKAGE_PATH=~/ros_workspace:/opt/ros/electric/stacks
		(*Second line is important. You need to add the directry which includes packages to environment variable.)
	b. Run "source devel/setup.bash" in the ~/catkin_ws directory

3. If error after "rosrun crazyflie crazyflie_node.py" with message "[rospack] Error: package 'crazyflie' not found":
	a. Run "source devel/setup.bash" again in the ~/catkin_ws directory

4. If error "ImportError: No module named cflib.crtp", then we must clone these repos into the ~/crazyflie/scripts directory:
	a. Run "git clone https://github.com/jlamyi/crazyflie_old_library.git" in scripts directory
	b. Move repos, "cfclient" and "cflib" into ~/crazyflie/scripts directory:
		b1. Run "mv cfclient ~/catkin_ws/src/crazyflie/scripts/"
		b2. Run "mv cflib ~/catkin_ws/src/crazyflie/scripts/"
		b3. Run "rm -rf crazyflie_old_library" in the ~/crazyflie/scripts directory

B) SETUP USB DONGLE
1. Clone "crazyradio-firmware" Git repo:
https://github.com/bitcraze/crazyradio-firmware.git

Note: For VMs, make sure the USB is readable and checked under the VM Menu "Devices > USB Devices"

2. USB Bootloader: http://wiki.bitcraze.se/projects:crazyradio:programming

> cd crazyradio-firmware
> python usbtools/launchBootloader.py 
Launch bootloader .
Bootloader started
