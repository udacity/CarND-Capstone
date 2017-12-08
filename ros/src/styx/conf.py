from attrdict import AttrDict

conf = AttrDict({
    'subscribers': [
        {'topic':'/vehicle/steering_cmd', 'type': 'steer_cmd', 'name': 'steering'},
        {'topic':'/vehicle/throttle_cmd', 'type': 'throttle_cmd', 'name': 'throttle'},
        {'topic':'/vehicle/brake_cmd', 'type': 'brake_cmd', 'name': 'brake'},
	{'topic':'/final_waypoints', 'type': 'path_draw', 'name': 'path'},
    ],
    'publishers': [
        {'topic': '/current_pose', 'type': 'pose', 'name': 'current_pose'},
        {'topic': '/current_velocity', 'type': 'twist', 'name': 'current_velocity'},
        {'topic': '/vehicle/steering_report', 'type': 'steer', 'name': 'steering_report'},
        {'topic': '/vehicle/throttle_report', 'type': 'float', 'name': 'throttle_report'},
        {'topic': '/vehicle/brake_report', 'type': 'float', 'name': 'brake_report'},
        {'topic': '/vehicle/obstacle', 'type': 'pose', 'name': 'obstacle'},
        {'topic': '/vehicle/obstacle_points', 'type': 'pcl', 'name': 'obstacle_points'},
        {'topic': '/vehicle/lidar', 'type': 'pcl', 'name': 'lidar'},
        {'topic': '/vehicle/traffic_lights', 'type': 'trafficlights', 'name': 'trafficlights'},
        {'topic': '/vehicle/dbw_enabled', 'type': 'bool', 'name': 'dbw_status'},
        {'topic': '/image_color', 'type': 'image', 'name': 'image'},
    ]
})
