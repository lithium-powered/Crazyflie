crazyflie-ros
=============
This is a research project in the Biomimetic Millisystems Lab at UC Berkeley under Professor Ronald S. Fearing.

-------------

The repository contains ROS nodes for controlling and interacting with the Bitcraze Crazyflie 10DOF quadcopter.
+ specific motor control
+ crazyradio: "radio://0/10/250K"
+ crazyflie_node.py

Summarized steps to run the crazyflie and webcam:
+ roscore
+ roslaunch ar_track_alvar webcam_track.launch
+ python find.py
+ rosrun crazyflie crazyflie_node.py

*More detailed documentation authored by Emily Chen as of 06/03/15 located in:*
+ https://goo.gl/HbpE0k

-------------

Student Researchers:
+ Liquin Yu | <liyu@berkeley.edu> (2015)
+ Emily Chen | <emilychen55@gmail.com> (2014-2015)
+ Maruchi Kim | <maruchi.kim@gmail.com> (2014)
+ James Lam Yi | <jlamyi@berkeley.edu> (2014)

*README updated by Emily Chen on 06/03/15*

