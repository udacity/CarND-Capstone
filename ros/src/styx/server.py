#!/usr/bin/env python

import eventlet
eventlet.monkey_patch(socket=True, select=True, time=True)

import eventlet.wsgi
import socketio
from flask import Flask

from bridge import Bridge
from conf import conf

'''
Modified msgs from queue list to a dictionary to disable queuing messages sent
to simulator and only send the latest topic data in case of lag while running
in a VM.  This method is based on:
https://carnd.slack.com/archives/C6NVDVAQ3/p1504347106000056?thread_ts=1504061507.000179&cid=C6NVDVAQ3
https://github.com/amakurin/CarND-Capstone/commit/9809bc60d51c06174f8c8bfe6c40c88ec1c39d50
'''

sio = socketio.Server()
app = Flask(__name__)
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
    # Disable publishing obstacle data because these are publishing at high
    # frequency and are not used for anything

    # bridge.publish_obstacles(data)
    pass


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
