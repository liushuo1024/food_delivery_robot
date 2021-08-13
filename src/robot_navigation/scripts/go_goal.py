#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
from std_msgs.msg import Int64
import thread
goal = 0
def doMsg(msg):
    global goal
    goal = msg.data
    rospy.loginfo("I heard:%d",goal)
def get_topic():
    rospy.spin()  

if __name__ == '__main__':
    rospy.init_node('exploring_slam', anonymous=True)
    sub = rospy.Subscriber("/mytopic",Int64,doMsg,queue_size=10)  
    # 订阅move_base服务器的消息  
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
    # 60s等待时间限制  
    move_base.wait_for_server(rospy.Duration(60))
    locations = dict()  

    locations['1'] = Pose(Point(0.000, 0.000, 0.000),  Quaternion(0.000, 0.000, -0.447, 0.894))  
    locations['2'] = Pose(Point(1.628, 0.460, 0.000),  Quaternion(0.000, 0.000, 0.702, 0.712))  
    locations['3'] = Pose(Point(1.699, 1.874, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
    locations['4'] = Pose(Point(1.696, 3.554, 0.000), Quaternion(0.000, 0.000, 0.663, 0.749))  
    locations['5'] = Pose(Point(1.803, 6.022, 0.000), Quaternion(0.000, 0.000, 0.703, 0.711))  
    locations['6'] = Pose(Point(1.803, 6.022, 0.000),   Quaternion(0.000, 0.000, 0.703, 0.711))
    thread.start_new_thread(get_topic,())  
    while (True):
        if goal == 1:            # 判断num的值
            print 'goal 1'
                # 设定下一个目标点  
            goal = MoveBaseGoal()  
            goal.target_pose.pose = locations['1']  
            goal.target_pose.header.frame_id = 'map'  
            goal.target_pose.header.stamp = rospy.Time.now()  
            # 向下一个位置进发  
            move_base.send_goal(goal)        
        elif goal == 2:
            print 'goal 2'
                # 设定下一个目标点  
            goal = MoveBaseGoal()  
            goal.target_pose.pose = locations['2']  
            goal.target_pose.header.frame_id = 'map'  
            goal.target_pose.header.stamp = rospy.Time.now()  
            # 向下一个位置进发  
            move_base.send_goal(goal)
        elif goal == 3:
            print 'goal 3'
            # 设定下一个目标点  
            goal = MoveBaseGoal()  
            goal.target_pose.pose = locations['3']  
            goal.target_pose.header.frame_id = 'map'  
            goal.target_pose.header.stamp = rospy.Time.now()  
            # 向下一个位置进发  
            move_base.send_goal(goal)
        elif goal == 3:
            print 'goal 3'
            # 设定下一个目标点  
            goal = MoveBaseGoal()  
            goal.target_pose.pose = locations['3']  
            goal.target_pose.header.frame_id = 'map'  
            goal.target_pose.header.stamp = rospy.Time.now()  
            # 向下一个位置进发  
            move_base.send_goal(goal)
        elif goal == 4:
            print 'goal 4'
            # 设定下一个目标点  
            goal = MoveBaseGoal()  
            goal.target_pose.pose = locations['4']  
            goal.target_pose.header.frame_id = 'map'  
            goal.target_pose.header.stamp = rospy.Time.now()  
            # 向下一个位置进发  
            move_base.send_goal(goal)   
        elif goal == 5:
            print 'goal 5'
                # 设定下一个目标点  
            goal = MoveBaseGoal()  
            goal.target_pose.pose = locations['5']  
            goal.target_pose.header.frame_id = 'map'  
            goal.target_pose.header.stamp = rospy.Time.now()  
            # 向下一个位置进发  
            move_base.send_goal(goal)
        else:
            print 'no goal'     # 条件均不成立时输出
 
        # 五分钟时间限制  
        finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

        # 查看是否成功到达  
        if not finished_within_time:  
            move_base.cancel_goal()  
            rospy.loginfo("Timed out achieving goal")  
        else:  
            state = move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("Goal succeeded!") 