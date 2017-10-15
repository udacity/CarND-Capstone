#!/usr/bin/env python

import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf

# this would help for lag problem in the simulator
# if the lag is still huge, please check the cpu load monitoring and
# make sure the resource is sufficient for running ros node and simulator both.
eventlet.monkey_patch()

sio = socketio.Server(async_mode='eventlet')
app = Flask(__name__)
# Changed to only send the latest message for each topic, rather
# than queuing out of date messages. Based on
# https://github.com/amakurin/CarND-Capstone/commit/9809bc60d51c06174f8c8bfe6c40c88ec1c39d50
msgs = {}

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    msgs[topic] = data

bridge = Bridge(conf, send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
    bridge.publish_odometry(data)
    for i in range(len(msgs)):
        topic, data = msgs.popitem()
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)

@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)

@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)

@sio.on('image')
def image(sid, data):
    bridge.publish_camera(data)

if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
