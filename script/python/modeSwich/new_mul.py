# -*- coding:UTF-8 -*-
#原文：https://www.w3cschool.cn/flask/flask_routing.html
from flask import Flask
import roslaunch
from flask import Flask
import multiprocessing
import time
import rospy

out_pipe, in_pipe = multiprocessing.Pipe(True)
def run(out_pipe):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/liushuo/food_delivery_robot/src/robot_navigation/launch/teb_test.launch"])
    msg = 0
    while True:
        if msg == 0 :
            msg= out_pipe.recv()
        else :
            if msg == 1:
                tracking_launch.start()
                msg = 3
            elif msg == 2:
                tracking_launch.shutdown() # tracking_launch 即是上面通过roslaunch获取到的变量
                print "已经关闭"
                msg = 0
            elif msg == 3:
                tracking_launch.spin_once()
            else :
                print "空线程"
                continue
app = Flask(__name__)
@app.route('/buildMap')
def hello_name():
    in_pipe.send(1)#通过管道的端口向子进程写入
    return 'Hello'
@app.route('/navigation')
def navigation():
    print "222222222222222222222222222222222222222222222222222222222222222222222222222222222"
    in_pipe.send(2)#通过管道的端口向子进程写入
    return 'Hello'



if __name__ == '__main__':
    son_run = multiprocessing.Process(target=run,args=(out_pipe,))
    son_run.start()
    app.run()
