# -*- coding: UTF-8 -*-
#! /usr/bin/env python
"""

"""
#1.导包 
import rospy
from std_msgs.msg import Int64
import thread
a = 0
def doMsg(msg):
    global a
    a=msg.data
    print a
def get_topic():
    rospy.spin()
if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("listener_p")
    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/mytopic",Int64,doMsg,queue_size=10)
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    thread.start_new_thread(get_topic,())
    while (True):
       pass 
