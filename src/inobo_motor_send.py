#!/usr/bin/python
# -*- coding: utf-8 -*-
# from Adafruit_MotorHAT import Adafruit_MotorHAT

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped,Twist,Vector3
from time import sleep
from std_msgs.msg import String
import numpy as np
import serial   # add uart
import json # add uart
import subprocess
import threading

# mh = Adafruit_MotorHAT(addr=0x60)
# myMotor1 = mh.getMotor(1)
# myMotor2 = mh.getMotor(2)
emergency_stop_flg = 0

NODE_NAME_STR = 'inobo_dcmotor'
CMD_VEL_STR = 'cmd_vel'#'joy_teleop/cmd_vel'
EMERGENCY_STOP_STR = 'emergency_stop'
# SHUTDOWN_STR = 'shutdown'

TREAD = 0.20 #0.25 #TRED 0.20m
LIMIT = 1.00#0.70
SPEED_MAX = 110 #150 #100 #255 #2024.6.26 12v->150 6v->110
SPEED_MIN = 70
ANGULAR_BOOST = 1.5 #3.0 #2.2 #2.0 #1.5 #2024.6.26 12v->2.5 6v->1.5

# タイムアウトの秒数
timeout_duration = 1.0

# タイムアウト処理用の関数
def timeout_callback():
    print("cmd_vel msg time out!!")
    ser.write("emergency_stop".encode('utf-8'))
    ser.write("\r\n".encode())


def callback(msg):
    global timeout_timer
    timeout_timer.cancel()  #タイマーキャンセル

    if emergency_stop_flg == 1:
        rospy.loginfo("emergency stop!!!")
        ser.write("emergency_stop".encode('utf-8'))
        ser.write("\r\n".encode())
        # myMotor1.run(Adafruit_MotorHAT.RELEASE)
        # myMotor2.run(Adafruit_MotorHAT.RELEASE)
        return

    rospy.loginfo("%s ' Linear [%f, %f, %f]"%(CMD_VEL_STR, msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("%s ' Angular [%f, %f, %f]"%(CMD_VEL_STR, msg.angular.x, msg.angular.y, msg.angular.z))


    if abs(msg.angular.z) > 0:
        rospy.loginfo("boost!!!!!!!!")
        msg.angular.z = msg.angular.z*3
        if msg.angular.z > 0.9:
            msg.angular.z = 0.9
        if msg.angular.z < -0.9:
            msg.angular.z = -0.9


    v_r = (-1*msg.angular.z*ANGULAR_BOOST*TREAD/2 + msg.linear.x)*SPEED_MAX*LIMIT
    v_l = (msg.angular.z*ANGULAR_BOOST*TREAD/2 + msg.linear.x)*SPEED_MAX*LIMIT

    # 最低速度の設定 => 値が小さすぎるとモーター出力されないため
    if 0 < v_r < SPEED_MIN:
        v_r = SPEED_MIN
    elif v_r < 0 and v_r > -SPEED_MIN:
        v_r = -SPEED_MIN

    if 0 < v_l < SPEED_MIN:
        v_l = SPEED_MIN
    elif v_l < 0 and v_l > -SPEED_MIN:
        v_l = -SPEED_MIN

    rospy.loginfo("%s ' v_r = %f, v_l = %f]"%(CMD_VEL_STR, v_r, v_l))

    # add uart -->
    json_string = "start$"+json.dumps({"v_r":v_r,"v_l":v_l})
    rospy.loginfo(json_string)
    ser.write(json_string.encode('utf-8'))
    ser.write("\r\n".encode())
    # add uart <--

    # cmd_vel 受信後、タイマー起動（モーター動きっぱなしをガードするため）
    timeout_timer = threading.Timer(timeout_duration, timeout_callback)
    timeout_timer.start()


def callback2(msg):
    rospy.loginfo("get message! [%s]", msg.data)
    global emergency_stop_flg
    if msg.data=="stop":
        emergency_stop_flg = 1
    elif msg.data=="reset":
        emergency_stop_flg = 0


# def callback3(msg):
#     rospy.loginfo("get message! [%s]", msg.data)
#     if msg.data=="jetson":
#         p = subprocess.Popen(["sudo shutdown -h now"],stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)
#         print p.communicate()
#     elif msg.data=="raspi":
#         rospy.loginfo("raspi shutdown!!!")
#         ser.write("shutdown".encode('utf-8'))
#         ser.write("\r\n".encode())

rospy.init_node(NODE_NAME_STR, anonymous=False)
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1) # add uart
try:
    rospy.Subscriber(CMD_VEL_STR, Twist, callback)
except rospy.ROSInterruptException:
    import traceback
    traceback.print_exc()

try:
    rospy.Subscriber(EMERGENCY_STOP_STR, String, callback2)
except rospy.ROSInterruptException:
    import traceback
    traceback.print_exc()

timeout_timer = threading.Timer(timeout_duration, timeout_callback)

# try:
#     rospy.Subscriber(SHUTDOWN_STR, String, callback3)
# except rospy.ROSInterruptException:
#     import traceback
#     traceback.print_exc()

rospy.spin()


    
