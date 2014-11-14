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
from pid import PID

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

        #SENSORS
        self.pitch = 0.0
        self.roll = 0.0
        self.thrust = 0
        self.yaw = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.pressure = 0.0
        self.zSpeed = 0.0
        self.asl = 0.0
        self.vSpeedAcc = 0.0
        self.vSpeedASL = 0.0
        self.logStarted = False
        self.altReference = 0.0

        #ACTUATOR ATTRIBUTES
        self.cmd_thrust = 20000
        self.cmd_pitch = 0.0
        self.cmd_roll = 0.0
        self.cmd_yaw = 0.0
        self.altHold = False
        
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
        self.asl_pub          = rospy.Publisher('baro/asl', Float32)
        self.zSpeed_pub       = rospy.Publisher('altHold/zSpeed', Float32)
        self.altReference_pub = rospy.Publisher('altHold/target', Float32)
 
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
    def getZ(self):
        return self.accel_z

    def shut_down(self):
        try:
            self.pitch_log.stop()
            self.altimeter_log.stop()
            self.accel_log.stop()
            self.motor_log.stop()
        finally:
            self.crazyflie.close_link()

    def connectionInitiated(self, linkURI):
        self.link_status = "Connection Initiated"
        self.link_status_pub.publish(self.link_status)

    def connectSetupFinished(self, linkURI):
        self.link_status = "Connect Setup Finished"
        self.link_status_pub.publish(self.link_status)
        self.setupAltimeterLog()
        self.setupStabilizerLog()
        self.setupAccelLog()
        self.setupMotorLog()

    #Accelerometer Logging
    def setupAccelLog(self):
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

    #Motor Logging
    def setupMotorLog(self):
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

    #Barometer, Height Above Sea Level, and Z-Axis Velocity Logging
    def setupAltimeterLog(self):
        log_conf = LogConfig("Altimeter", 10)
        log_conf.addVariable(LogVariable("baro.pressure", "float")) #see crazyflie-firmware/stabilizer.c file
        log_conf.addVariable(LogVariable("altHold.zSpeed", "float")) 
        log_conf.addVariable(LogVariable("altHold.vSpeedASL", "float")) 
        log_conf.addVariable(LogVariable("altHold.vSpeedAcc", "float")) 
        log_conf.addVariable(LogVariable("baro.asl", "float"))
        log_conf.addVariable(LogVariable("altHold.target", "float"))

        self.altimeter_log = self.crazyflie.log.create_log_packet(log_conf)
 
        if self.altimeter_log is not None:
            self.altimeter_log.dataReceived.add_callback(self.log_altimeter_data)
            self.altimeter_log.start()
            print("barometer now logging")
        else:
            print("barometer ERROR")

    #Gyro Logging
    def setupStabilizerLog(self):
        log_conf = LogConfig("Pitch", 10)
        log_conf.addVariable(LogVariable("stabilizer.pitch", "float"))
        log_conf.addVariable(LogVariable("stabilizer.roll", "float"))
        log_conf.addVariable(LogVariable("stabilizer.thrust", "int32_t"))
        log_conf.addVariable(LogVariable("stabilizer.yaw", "float"))
        #log_conf.addVariable(LogVariable("altimeter.pressure", "float"))

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
        #rospy.loginfo("Gyro: Pitch=%.2f, Roll=%.2f, Yaw=%.2f" %
        #    (data["stabilizer.pitch"], data["stabilizer.roll"], data["stabilizer.yaw"]))
        self.pitch  = data["stabilizer.pitch"]
        self.roll   = data["stabilizer.roll"]
        self.thrust = data["stabilizer.thrust"]
        self.yaw    = data["stabilizer.yaw"]
        self.logStarted = True

    def log_altimeter_data(self, data):
        self.pressure = data["baro.pressure"]
        self.zSpeed = data["altHold.zSpeed"]
        self.asl = data["baro.asl"]
        self.vSpeedAcc = data["altHold.vSpeedAcc"]
        self.vSpeedASL = data["altHold.vSpeedASL"]
        self.altReference = data["altHold.target"]
        rospy.loginfo("ASL: %f" % self.asl)

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


    # TO-DO: Safety Shut Down
    def motors_shut_down(self):
        self.set_m1("0")
        self.set_m2("0")
        self.set_m3("0")
        self.set_m4("0")

    def control_height(self):
        self.altHold = True

    def publish_data(self):
        # self.link_quality_pub.publish(self.link_quality)
        # self.packet_count_pub.publish(self.packetsSinceConnection)
        # self.motor_status_pub.publish(self.motor_status)
        # self.pitch_pub.publish(self.pitch)
        # self.roll_pub.publish(self.roll)
        # self.thrust_pub.publish(self.thrust)
        # self.yaw_pub.publish(self.yaw)
        self.asl_pub.publish(self.asl)
        self.altReference_pub.publish(self.altReference)
        # self.zSpeed_pub.publish(self.zSpeed)

    # main loop 
    def run_node(self, time):
        self.cmd_thrust = 40000
        print "Time since start: ", time
        if (time > 2):
            print "---Stabilizing Height---"
            self.control_height()

        #Send commands to the Crazyflie
        if (self.altHold):
            self.crazyflie.param.set_value("flightmode.althold", "True")
            self.crazyflie.commander.send_setpoint(0, 0, 0, 32767)
        else:
            self.yaw += 0.5
            self.crazyflie.commander.send_setpoint(0.0, 0.0, self.yaw, self.cmd_thrust)

def run():
    # Init the ROS node here, so we can split functionality
    # for this node across multiple classes        
    rospy.init_node('crazyflie')
    print "yes lets start!!\n"
    #TODO: organize this into several classes that monitor/control one specific thing
    node = CrazyflieNode()
    loop_rate = rospy.Rate(100) #100 Hz
    while (node.logStarted == False):
        print "Waiting to for Sensor Callbacks..."
    print "Finished! Launching..."
    start = rospy.get_time()

    while not rospy.is_shutdown():
        node.run_node(rospy.get_time() - start)
        loop_rate.sleep()

    node.motors_shut_down()
    node.shut_down()
               
if __name__ == '__main__':
    run()