# -*- coding:UTF-8 -*-
#原文：https://www.w3cschool.cn/flask/flask_routing.html
from flask import Flask
import roslaunch
from flask import Flask
import multiprocessing
import time

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# tracking_launch = roslaunch.parent.ROSLaunchParent(
#     uuid, ["/home/liushuo/food_delivery_robot/src/robot_navigation/launch/teb_test.launch"])
def run():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/liushuo/food_delivery_robot/src/robot_navigation/launch/teb_test.launch"])
    tracking_launch.start()
    # tracking_launch.spin()
    time.sleep(1)
    tracking_launch.shutdown() # tracking_launch 即是上面通过roslaunch获取到的变量
    # print "111111111111111111111111111111111111111111111111111111111111111111111111111111111111111"
    # tracking_launch.start()
    # tracking_launch.spin_once()
    # tracking_launch.shutdown()
son_run = multiprocessing.Process(target=run,)
def kill():
    tracking_launch.shutdown()
son_kill = multiprocessing.Process(target=kill,)
app = Flask(__name__)
out_pipe, in_pipe = multiprocessing.Pipe(True)
@app.route('/buildMap')
def hello_name():
    son_run.start()
    son_run.join()
    return 'Hello'
@app.route('/navigation')
def navigation():
    print "222222222222222222222222222222222222222222222222222222222222222222222222222222222"
    son_kill.start()
    son_run.terminate()
    son_run.process.signal(signal.SIGINT)
    son_run.process.signal(signal.SIGINT)
    son_run.join()
    # son_kill.start()
    # in_pipe.send(2)
    return 'Hello'

def son_process(x, pipe):
    _out_pipe, _in_pipe = pipe
    # 关闭fork过来的输入端
    _in_pipe.close()
    while True:
        try:
            msg = _out_pipe.recv()
            print msg
        except EOFError:
            # 当out_pipe接受不到输出的时候且输入被关闭的时候，会抛出EORFError，可以捕获并且退出子进程
            break
        if msg == 1:
            # print msg
            tracking_launch.start()
            # msg = 0
        elif msg == 2:
            # print msg
            tracking_launch.shutdown()
            # msg = 0



if __name__ == '__main__':
    # p1 = multiprocessing.Process(target = run)
    # son_p = multiprocessing.Process(target=son_process, args=(100, (out_pipe, in_pipe)))
    # son_p.start()
    app.run()
    # son_p.join()
