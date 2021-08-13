from flask import Flask
app = Flask(__name__)
flag = 0 
@app.route('/buildMap')
def hello_name():
    global flag
    flag = 1
    print flag
    return 'Hello'
@app.route('/navigation')
def navigation():
    global flag
    print flag
    return 'Hello'
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
tracking_launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/liushuo/food_delivery_robot/src/robot_navigation/launch/teb_test.launch"])
if __name__ == '__main__':
   app.run(debug = True)