#!/usr/bin/env python

import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf

sio = socketio.Server()
app = Flask(__name__)
bridge = Bridge(conf)
# change per a. makurin
# msgs = []
msgs = {}

dbw_enable = False
first_dbw = True

@sio.on('connect')
def connect(sid, environ):
    print("connect", sid)
    pass

@sio.on('disconnect')
def disconnect(sid):
    global first_dbw
    global dbw_enable
    first_dbw = True
    dbw_enable = False
    print("disconnect", sid)
    pass

def send(topic, data):
    # changes per a. makurin
    # rospy.loginfo("t %s d %s", topic, data)
    msgs[topic] = data

bridge.register_server(send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    global first_dbw
    if first_dbw or data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
        first_dbw = False
    bridge.publish_odometry(data)
    global msgs
    # send all 3 messages at once.  If throttle isn't
    # last, simulator sometimes takes its foot off the throttle
    if len(msgs) >= 3:
        for key in ['steer', 'brake', 'throttle']:
            if not key in msgs: 
                print(key,"not defined")
                continue
            mdata = msgs[key]
            sio.emit(key, data=mdata, skip_sid=True)
        msgs = {}


@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)
    # print("c", sid, data)

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
