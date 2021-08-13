# -*- coding:UTF-8 -*-
#原文：https://www.w3cschool.cn/flask/flask_routing.html
from flask import Flask
import os
app = Flask(__name__)
@app.route('/buildMap')
def hello_name():
    os.system('echo buildMap')
    os.system('/home/liushuo/food_delivery_robot/script/web/build_map/start')
    return 'Hello'
@app.route('/navigation')
def navigation():
    os.system('echo navigation')
    os.system('/home/liushuo/food_delivery_robot/script/web/fallback_navigation/start')
    return 'Hello'



if __name__ == '__main__':
    app.run()
