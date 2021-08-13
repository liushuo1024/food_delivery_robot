# -*- coding:UTF-8 -*-
import roslaunch
import time
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/liushuo/food_delivery_robot/src/robot_navigation/launch/teb_test.launch"])
tracking_launch.start()
# tracking_launch.spin()
time.sleep(5)
def kill():
    tracking_launch.shutdown() # tracking_launch 即是上面通过roslaunch获取到的变量