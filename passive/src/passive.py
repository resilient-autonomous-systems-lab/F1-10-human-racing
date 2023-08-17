#!/usr/bin/env python3

"""
Publishes the following:
1. publish requried joystick command to MicroNole

Subscribes to the following:
1. subscribe racing cockpit command
2. subscribe camera infor from MicroNole (192.168.2.15)

Functions:
1. display the camera view
"""

import time

import numpy as np
import math
import rospy

from geometry_msgs.msg import Vector3Stamped,Twist,PoseStamped
from sensor_msgs.msg import Joy, Image, CompressedImage
import cv2
import pickle
   
import csv
from std_msgs.msg       import Float64

import math

# import evdev
# from evdev import ecodes, InputDevice
# device = evdev.list_devices()[0]

class carModel():
    def __init__(self):
        super().__init__()
        self.wm = 0
        self.ws = 0
        self.vx = 0
        self.vy = 0
        self.theta1 = 4.5355
        self.theta2 = 12985
        self.theta3 = 191.17
        self.theta4 = 85.018
        self.theta5 = 696.47
        self.theta6 = 370.3704
        self.Cw = 1880.5
        self.Cwd = 1000
        self.J = 22.1185
        self.rg = 9.49
        self.m = 2.7
        self.Rw = 0.024
        self.L = 0.256

    def update(self,alpha,delta):
        A1 = -self.theta1 * self.wm
        A2 = self.theta2 * alpha
        wmd = A1 + A2

        B1 = self.theta3 * self.L * delta * math.cos(delta) * self.vx / 2
        B2 = self.theta3 * self.L * (1 - math.cos(delta)) * self.vy / 2
        B3 = -self.L**2 [(self.theta3 *(1 + math.cos(delta)) / self.L )+ self.theta4] * self.wz / 2
        B4 = self.Rw * self.L * self.theta4 * math.sin(delta) * self.wm / (2* self.rg)
        wzd = B1 + B2 + B3 + B4

        C1 = - self.vx * (2 * self.theta5 + self.theta6 * delta * math.sin(delta))
        C2 = self.theta6 * math.sin(delta) * self.vx
        C3 = -self.wz * self.vx
        C4 = self.L * self.theta6 * math.sin(delta)* self.wz / 2
        C5 = self.theta5 * self.Rw * math.sin(dela) * self.wm / self.rg
        vxd = C1 + C2 + C3 + C4 + C5
        
        D1 = self.theta6 * delta * math.cos(delta) * self.vx 
        D2 = -self.theta6 * (1 - math.cos(delta)) * self.vy
        D3 = -self.wz * self.vx
        D4 = self.L * [self.theta6 *( 1 - math.cos(delta)) - 2 * self.theta5 ] * self.wz
        D5 = self.theta5 * self.Rw * math.sin(delta) * self.wm / self.rg
        vyd = D1 + D2 + D3 + D4 + D5

        self.wm = wmd
        self.wz = wzd
        self.vx = vxd
        self.vy = vyd

    def getVelocity(self):
        return self.vx
        
class racingNode(object):
    """docstring for ClassName"""

    def __init__(self, publish_joy=True, publish_rate= 120.):
        """Constructor for racing simulator"""
        super().__init__()

        self.publish_rate = publish_rate  # /G29/joy topic miximal rate: 35hz
        self.sample_time = 1.0 / self.publish_rate  # s (default = 0.001)
        self.rate = rospy.Rate(self.publish_rate)

        self.ctrl_pub = rospy.Publisher('/racing_cockpit/ctrl_cmd', Vector3Stamped, queue_size=10)
        self.ctrl_sub = rospy.Subscriber("/G29/joy", Joy, self.ctrl_callback, queue_size=10)
        self.adaptive_sub = rospy.Subscriber("/adaptive_response", Vector3Stamped, self.adaptive_callback, queue_size=10)
        self.delay = 0.0
        self.cam_pos_pub = rospy.Publisher('/human_remote/camera_pos', PoseStamped, queue_size=1)
        
        self.axes = np.zeros(6)
        self.button = np.zeros(18)
        self.command = np.zeros(3)
        self.throttle_scale, self.steering_scale = 0.2/2, math.pi/2
        
        self.front_cam = 1
        self.rear_cam = 0
        self.left_cam = 0
        self.right_cam = 0

        #passivity constant
        self.b=8
        self.velocity=0.0
        self.force_feedback = 0.0
        self.tval = 0.0

        #force_feedback
    #     self.evtdev = InputDevice(device)
        
    #     self.ff_val=0

    #     self.prev_steer = 0
    #     self.force_feeback_calculation = 0.

    # def force_calculation(self):
    #     steer = float(self.command[0])
    #     diff = - steer
    #     if diff > 0 : dir=1
    #     else : dir =-1

    #     autocenter_control_p = 0.5
    #     autocenter_control_d = 1.5
    #     wheel_resistance = 0.5
    #     torque = (autocenter_control_p*diff)+(autocenter_control_d*(steer - self.prev_steer))-(wheel_resistance*steer)
    #     self.force_feeback_calculation = min(abs(torque),1) * dir

    #     self.prev_steer = steer

    def publish_data(self):
        while not rospy.is_shutdown():
            # all racing cockpit joystick command
            self.steering = self.axes[0]     # +:left, -:right [-1,1]
            self.throttle = self.axes[1]+1   # [-1,1] --> [0,2] default:-1
            self.brake    = self.axes[2]+1   # [-1,1] --> [0,2] default:-1
            self.clutch   = self.axes[3]     # [-1,1]
            
            self.A, self.B, self.X, self.Y = self.button[:4]
            self.shift_paddle_right = self.button[4]
            self.shift_paddle_left = self.button[5]
            self.RSB = self.button[8]
            self.LSB = self.button[9]
            self.xbox = self.button[10]
            self.gearshift_left_f = self.button[12]
            self.gearshift_middle_f = self.button[14]
            self.gearshift_right_f = self.button[16]
            self.gearshift_left_b = self.button[13]
            self.gearshift_middle_b = self.button[15]
            self.gearshift_right_b = self.button[17]
            
            self.command[0] = np.maximum(np.minimum(self.steering_scale * self.steering,math.pi/6),-math.pi/6)
            self.command[1] = np.maximum(self.throttle_scale * ( np.abs(self.throttle) - np.abs(self.brake) ), 0)
            self.command[2] = 0
            if self.gearshift_left_f==1 or self.gearshift_middle_f==1 or self.gearshift_right_f==1:
                self.command[2] = 1
            if self.gearshift_left_b==1 or self.gearshift_middle_b==1 or self.gearshift_right_b==1:
                self.command[2] = -1
            self.publisher_joy()

            self.tval = 0.0
            if self.command[2] > 0.5:
                self.tval = self.command[1]
            if self.command[2] < -0.5:
                self.tval = -1.0 * self.command[1]

            # val = (int(self.force_feedback)+2)/2 - 1
            # self.ff_val =int(self.force_feedback * 32767)
            
            
            # decide stream camera position
            if self.steering > 0.1:
                self.left_cam = 1
                self.right_cam = 0
            elif self.steering < -0.1:
                self.right_cam = 1
                self.left_cam = 0
            else:
                self.right_cam = 0
                self.left_cam = 0
            
            if self.command[2] < 0:
                self.front_cam = 0
                self.rear_cam = 1
            else:
                self.front_cam = 1
                self.rear_cam = 0
            
            
            # publish stream camera position
            cam_pose = PoseStamped()
            cam_pose.header.stamp = rospy.Time.now()
            cam_pose.header.frame_id = 'remote racing'
            cam_pose.pose.orientation.x = self.front_cam
            cam_pose.pose.orientation.y = self.rear_cam
            cam_pose.pose.orientation.z = self.left_cam
            cam_pose.pose.orientation.w = self.right_cam
            self.cam_pos_pub.publish(cam_pose)

    def ctrl_callback(self, data):
        self.axes = data.axes
        self.button = data.buttons
        
    def adaptive_callback(self,data):
        # self.velocity = (self.b * self.tval +  data.vector.x) /2
        # self.force_feedback = (self.b * self.command[0] +  data.vector.y) /2
        self.velocity = (self.tval +  data.vector.x) *(self.b/2)**0.5
        self.force_feedback = (self.command[0] +  data.vector.y) *(self.b/2)**0.5

        print("Velocity:",str(self.velocity),"----->FF",str(self.force_feedback))
        # self.force_calculation()
        # print("Forcefeedback to device:",str(self.force_feeback_calculation* 32767))
        # self.evtdev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(self.force_feeback_calculation* 32767))

    def publisher_joy(self):
        self.command = (self.b/2)**0.5 * self.command
        joy_command = Vector3Stamped()
        joy_command.header.stamp = rospy.Time.now()
        joy_command.header.frame_id = 'remote racing'
        joy_command.vector.x = float(self.command[0])
        joy_command.vector.y = float(self.command[1])
        joy_command.vector.z = float(self.command[2])
        time.sleep(0.5)
        self.ctrl_pub.publish(joy_command)

    def shutdown(self):
        rospy.loginfo("Beginning shutdown routine...")
        
        rospy.loginfo("Shutting down cleanly...")
        
    def timer_watcher(self, event):
        rospy.loginfo(f"Delay: {self.delay}")


if __name__ == '__main__':
    rospy.init_node('racingNode')
    racing = racingNode()
    rospy.on_shutdown(racing.shutdown)

    try:
        racing.publish_data()
        # rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as e:
        rospy.loginfo(f'Encountered {e}.')


