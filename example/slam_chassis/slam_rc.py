#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time
import sys
import tty
import termios

import threading

keyQueue = []
old_setting = termios.tcgetattr(sys.stdin)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getKeyBoard():
    global keyQueue
    while True:
        c = readchar()
        keyQueue.append(c)

t1 =threading.Thread(target=getKeyBoard)
t1.setDaemon(True)
t1.start()

def key2Twist(key:str):
    twist = Twist()
    twist.linear.x,  twist.linear.y,  twist.linear.z  = 0.0, 0.0, 0.0
    twist.angular.x, twist.angular.y, twist.angular.z = 0.0, 0.0, 0.0

    if len(key) == 0 or len(key) > 1: return twist
    if key == "w": twist.linear.x =  0.5
    if key == "s": twist.linear.x = -0.5
    if key == "a": twist.angular.z =  0.5
    if key == "d": twist.angular.z = -0.5

    return twist

def main():
    rospy.init_node("slam_rc")
    cmd_vel_pub = rospy.Publisher("/diablo/cmd_vel", Twist, queue_size = 2)
    print("slam_rc: start!")
    while True:
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
        else:
            key = " "
        if key == "1":
            break
        cmd_vel_pub.publish(key2Twist(key))
        time.sleep(0.04) # 25hz
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    print("exit!")


if __name__ == "__main__":
    main()
