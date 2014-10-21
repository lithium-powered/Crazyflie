#!/usr/bin/env python
#
# crazyflie_node.py
#
# Copyright (c) 2013 Jesse Rosalia
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import rospy
import logging
import math

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable
from cflib.crazyflie import Crazyflie
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import String
logging.basicConfig(level=logging.DEBUG)
class CrazyflieNode:
    """
    Class is required so that methods can access the object fields.
    """
    def __init__(self):
        """
        Connect to Crazyflie, initialize drivers and set up callback.
 
        The callback takes care of logging the accelerometer values.
        """
        
        self.link_status = "Unknown"
        self.link_quality = 0.0
        self.packetsSinceConnection = 0
        self.motor_status = ""
        self.pitch = 0.0
        self.roll = 0.0
        self.thrust = 0
        self.yaw = 0.0

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.cmd_thrust = 0
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw = 0.0
	
    	self.desired_pitch = 0.0
    	self.pitch_coefficient = 1
        self.desired_roll = 0.0
        self.roll_coefficient = 1
        self.desired_yaw = 2.0
        self.yaw_coefficient = 1

        self.thrust_coefficient = 1
        self.desired_accel_z = 0.9
        self.thrust_kp = -40000
    	self.kp = 0.5
        
        # Init the callbacks for the crazyflie lib
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()

        # Init the published topics for ROS, for this class
        self.link_status_pub  = rospy.Publisher('link_status', String, latch=True)
        self.link_quality_pub = rospy.Publisher('link_quality', Float32)
        self.packet_count_pub = rospy.Publisher('packet_count', UInt32)

        self.motor_status_pub = rospy.Publisher('motors', String)

        self.pitch_pub        = rospy.Publisher('stabilizer/pitch', Float32)
        self.roll_pub         = rospy.Publisher('stabilizer/roll', Float32)
        self.thrust_pub       = rospy.Publisher('stabilizer/thrust', Float32)
        self.yaw_pub          = rospy.Publisher('stabilizer/yaw', Float32)
 
        rospy.Subscriber('pitch', UInt16, self.set_pitch)
        rospy.Subscriber('roll', UInt16, self.set_roll)
        rospy.Subscriber('thrust', UInt16, self.set_thrust)
        rospy.Subscriber('yaw', UInt16, self.set_yaw)

        # Connection callbacks
        #TODO: for a lot of these, we just update the status and/or publish a value
        # it would be sweet if we could create a generic function to do that for us,
        # instead of using all of these callbacks.
        self.crazyflie.connectionInitiated.add_callback(self.connectionInitiated)
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
        self.crazyflie.connected.add_callback(self.connected)
        self.crazyflie.disconnected.add_callback(self.disconnected)
        self.crazyflie.connectionLost.add_callback(self.connectionLost)
        self.crazyflie.connectionFailed.add_callback(self.connectionFailed)

        # Link quality callbacks
        self.crazyflie.linkQuality.add_callback(self.linkQuality)
        self.crazyflie.receivedPacket.add_callback(self.receivedPacket)
        
        #TODO: should be configurable, and support multiple devices
        self.crazyflie.open_link("radio://0/9/250K")
 
    def shut_down(self):
        try:
            self.pitch_log.stop()
        finally:
            self.crazyflie.close_link()

    def connectionInitiated(self, linkURI):
        self.link_status = "Connection Initiated"
        self.link_status_pub.publish(self.link_status)

    def connectSetupFinished(self, linkURI):
        
        self.link_status = "Connect Setup Finished"
        self.link_status_pub.publish(self.link_status)
        
        self.setupStabilizerLog()

        """
        Configure the logger to log accelerometer values and start recording.
 
        The logging variables are added one after another to the logging
        configuration. Then the configuration is used to create a log packet
        which is cached on the Crazyflie. If the log packet is None, the
        program exits. Otherwise the logging packet receives a callback when
        it receives data, which prints the data from the logging packet's
        data dictionary as logging info.
        """
        # Set accelerometer logging config
        accel_log_conf = LogConfig("Accel", 10)
        accel_log_conf.addVariable(LogVariable("acc.x", "float"))
        accel_log_conf.addVariable(LogVariable("acc.y", "float"))
        accel_log_conf.addVariable(LogVariable("acc.z", "float"))
 
        # Now that the connection is established, start logging
        self.accel_log = self.crazyflie.log.create_log_packet(accel_log_conf)
 
        if self.accel_log is not None:
            self.accel_log.dataReceived.add_callback(self.log_accel_data)
            self.accel_log.start()
        else:
            print("acc.x/y/z not found in log TOC")

        motor_log_conf = LogConfig("Motor", 10)
        motor_log_conf.addVariable(LogVariable("motor.m1", "int32_t"))
        motor_log_conf.addVariable(LogVariable("motor.m2", "int32_t"))
        motor_log_conf.addVariable(LogVariable("motor.m3", "int32_t"))
        motor_log_conf.addVariable(LogVariable("motor.m4", "int32_t"))
 
        # Now that the connection is established, start logging
        self.motor_log = self.crazyflie.log.create_log_packet(motor_log_conf)
 
        if self.motor_log is not None:
            self.motor_log.dataReceived.add_callback(self.log_motor_data)
            self.motor_log.start()
        else:
            print("motor.m1/m2/m3/m4 not found in log TOC")
 
    def setupStabilizerLog(self):
        log_conf = LogConfig("Pitch", 10)
        log_conf.addVariable(LogVariable("stabilizer.pitch", "float"))
        log_conf.addVariable(LogVariable("stabilizer.roll", "float"))
        log_conf.addVariable(LogVariable("stabilizer.thrust", "int32_t"))
        log_conf.addVariable(LogVariable("stabilizer.yaw", "float"))
        self.pitch_log = self.crazyflie.log.create_log_packet(log_conf)
 
        if self.pitch_log is not None:
            self.pitch_log.dataReceived.add_callback(self.log_pitch_data)
            self.pitch_log.start()
            print("stabilizer.pitch/roll/thrust/yaw now logging")
        else:
            print("stabilizer.pitch/roll/thrust/yaw not found in log TOC")
        
    def connected(self, linkURI):
        self.packetsSinceConnection = 0
        self.link_status = "Connected"
        self.link_status_pub.publish(self.link_status)

    def disconnected(self, linkURI):
        self.link_status = "Disconnected"
        self.link_status_pub.publish(self.link_status)
     
    def connectionLost(self, linkURI, errmsg):
        self.link_status = "Connection Lost - " + errmsg
        self.link_status_pub.publish(self.link_status)
 
    def connectionFailed(self, linkURI, errmsg):
        self.link_status = "Connection Failed - " + errmsg
        self.link_status_pub.publish(self.link_status)
 
    def linkQuality(self, percentage):
        self.link_quality = percentage

    def receivedPacket(self, pk):
        self.packetsSinceConnection += 1

    def log_accel_data(self, data):
        self.accel_x = data["acc.x"]
        self.accel_y = data["acc.y"]
        self.accel_z = data["acc.z"]
        # rospy.loginfo("Accelerometer: x=%.2f, y=%.2f, z=%.2f" %
        #                 (data["acc.x"], data["acc.y"], data["acc.z"]))

    def log_motor_data(self, data):
        self.motor_status = ("Motors: m1=%d, m2=%d, m3=%d, m4=%d" %
                        (data["m1"], data["m2"], data["m3"], data["m4"]))

    def log_pitch_data(self, data):
        # rospy.loginfo("Gyro: Pitch=%.2f, Roll=%.2f, Yaw=%.2f" %
        #     (data["stabilizer.pitch"], data["stabilizer.roll"], data["stabilizer.yaw"]))
        self.pitch  = data["stabilizer.pitch"]
        self.roll   = data["stabilizer.roll"]
        self.thrust = data["stabilizer.thrust"]
        self.yaw    = data["stabilizer.yaw"]

    #pitch/roll/thrust/yaw setting function
    def set_pitch(self, data):
        rospy.loginfo(rospy.get_name() + ": Setting pitch to: %d" % data.data)
        self.cmd_pitch = data.data
    
    def set_roll(self, data):
        rospy.loginfo(rospy.get_name() + ": Setting roll to: %d" % data.data)
        self.cmd_roll = data.data       
               
    def set_thrust(self, data):
        rospy.loginfo(rospy.get_name() + ": Setting thrust to: %d" % data)
        self.cmd_thrust = data.data      
           
    def set_yaw(self, data):
        rospy.loginfo(rospy.get_name() + ": Setting yaw to: %d" % data.data)
        self.cmd_yaw = data.data

    #motor setting function
    def set_m1(self, thrust):
        self.crazyflie.param.set_value("motors.motorPowerM1", thrust)

    def set_m2(self, thrust):
        self.crazyflie.param.set_value("motors.motorPowerM2", thrust)

    def set_m3(self, thrust):
        self.crazyflie.param.set_value("motors.motorPowerM3", thrust)

    def set_m4(self, thrust):
        self.crazyflie.param.set_value("motors.motorPowerM4", thrust)

    def motors_shut_down(self):
        self.set_m1("0")
        self.set_m2("0")
        self.set_m3("0")
        self.set_m4("0")

    #pitch control function
    def pitch_control(self):
        self.cmd_pitch += self.kp*self.pitch_coefficient*(self.desired_pitch - self.pitch)
        #rospy.loginfo(rospy.get_name() + ": Setting pitch to: %d" % self.cmd_pitch) 

    #roll control function
    def roll_control(self):
        self.cmd_roll += self.kp*self.roll_coefficient*(self.desired_roll - self.roll) 
        #rospy.loginfo(rospy.get_name() + ": Setting roll to: %d" % self.cmd_roll)   
    
    #yaw control function
    def yaw_control(self):
        self.cmd_yaw += self.kp*self.yaw_coefficient*(self.desired_yaw - self.yaw) 
        #rospy.loginfo(rospy.get_name() + ": Setting roll to: %d" % self.cmd_roll) 

    #hover control function
    def z_control(self):
        self.cmd_thrust += self.thrust_kp*self.thrust_coefficient*(self.desired_accel_z - self.accel_z) 
        #rospy.loginfo(rospy.get_name() + ": Setting thrust to: %d" % self.cmd_thrust)
        print str(self.cmd_thrust) + " " + str(self.accel_z)

    # main loop 
    def run_node(self):
        self.cmd_thrust = 36000
        self.link_quality_pub.publish(self.link_quality)
        self.packet_count_pub.publish(self.packetsSinceConnection)
        self.motor_status_pub.publish(self.motor_status)
        self.pitch_pub.publish(self.pitch)
        self.roll_pub.publish(self.roll)
        self.thrust_pub.publish(self.thrust)
        self.yaw_pub.publish(self.yaw)
        # CONTROLLERS
        self.pitch_control()
        self.roll_control()
        self.yaw_control()
        self.z_control()
        # Send commands to the Crazyflie
        #rospy.loginfo(rospy.get_name() + ": Sending setpoint: %f, %f, %f, %d" % (self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust))
        self.crazyflie.commander.send_setpoint(self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust)
        
        #test
        # self.set_m1("20000")
        # self.set_thrust(20000);

     
def run():
    # Init the ROS node here, so we can split functionality
    # for this node across multiple classes        
    rospy.init_node('crazyflie')
    print "yes lets start!!\n"
    #TODO: organize this into several classes that monitor/control one specific thing
    node = CrazyflieNode()
    while not rospy.is_shutdown():
        #pitch changing test
	#if node.cmd_pitch > -40.0:
	   #node.cmd_pitch -= 2.0
        node.run_node()
        rospy.sleep(1)#0.01
    node.motors_shut_down()
    node.shut_down()
        
        
if __name__ == '__main__':
    run()
