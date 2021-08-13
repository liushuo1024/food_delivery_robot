#!/usr/bin/python
# -*- coding: UTF-8 -*-
import json
import rosnode
import os
import requests
import time
from enum import Enum 
import socket
import time
import sys
import errno #捕获socket error
from socket import error as socket_error
def get_sensor_state(str):
    state=os.popen("ls "+str).read()
    if str==state.strip():
        return True
    else:
        return False
def get_multi_sensor_state(dict):
    list=dict.keys()
    for str in list:
        dict[str] = get_sensor_state(str)
    return dict
"""
    分析错误类型
    /amcl
    /base2camera
    /base2laser
    /foot2base
    /laser_filter
    /link_ros
    /main_cpp
    /map_server
    /move_base
    /robot_bringup
    /robot_pose_ekf
    /rosapi
    /rosbridge_websocket
    /rosout
    /rplidarNode
    /tf2_web_republisher
"""
def get_node_error(node_str):
    node_tup=("/amcl","/base2camera","/base2laser","/foot2base","/laser_filter","/link_ros","/main_cpp",
    "/map_server","/move_base","/robot_bringup","/robot_pose_ekf","/rosapi","/rosbridge_websocket",
    "/rosout","/rplidarNode","/tf2_web_republisher")
    for node in node_tup:
        if (node not in node_str):
            return False
    return True

def get_sensor_error(sensor_state_dict):
    for key,values in  dict.items():
        if values == False:
            return False
    return True
    
class status_word(Enum):
    sensor_error = 1
    ROS_error = 2
    Navigation_error = 3
    robot_normal = 4

"""
    机器状态
"""
def get_robot_state(node_state,sensor_state):
    if node_state and sensor_state:
        return status_word.robot_normal.name
    elif node_state != sensor_state:
        if not node_state:
            return status_word.ROS_error.name
        elif not sensor_state:
            return status_word.sensor_error.name 


if __name__ == "__main__":
    list = []          ## 空列表用来存放检索到的状态，转换为josn
    while True:
        sk = socket.socket()
        try:
            sk.connect(('127.0.0.1',9000))
        except socket_error as serr:
            if serr.errno != errno.ECONNREFUSED:
                continue
        while True:
            json_data = {'No': None, 'time': None, 'error_type': None, 'sensor_state': None, 'ROS_nodes': None, 'others': None}
            dict = {"/dev/robot":None,"/dev/imu":None,"/dev/robot":None,'/dev/video0':None,"/dev/video1":None}
            No="0001"#机器编号
            localtime = time.asctime(time.localtime(time.time()))#获取时间戳
            sensor_state_dict = get_multi_sensor_state(dict)#传感器状态
            try:
                node_str = rosnode._sub_rosnode_listnodes()#节点列表
            except:
                node_str = "" # rosmaster 未启动
            sensor_state = get_sensor_error(sensor_state_dict)#检查传感器状态
            node_state = get_node_error(node_str)#检查节点状态
            robot_state=get_robot_state(node_state,sensor_state)
            if robot_state != "robot_normal":
                json_data["No"]=No
                json_data["time"]=localtime    
                json_data["error_type"]=robot_state
                json_data["sensor_state"]=sensor_state_dict
                json_data["ROS_nodes"]=node_str
                data =json.dumps(json_data,encoding='utf-8',ensure_ascii=False)
                try:
                    sk.send(data)
                ### IO operation ###
                except IOError as e:
                    if e.errno == errno.EPIPE:
                        sk.close()
                        break    
                    # msg = sk.recv(1024)  # 最多接受1024字节
                    # print(msg)
            time.sleep(1)
    sk.close()