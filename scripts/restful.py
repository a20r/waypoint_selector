
import config
import json
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf import transformations
from flask import jsonify, request


def transform_waypoint(waypoint):
    ps = PoseStamped()
    origin = config.occ_grid.info.origin
    quat = [0, 0, 0, 0]
    quat[0] = origin.orientation.w
    quat[1] = origin.orientation.x
    quat[2] = origin.orientation.y
    quat[3] = origin.orientation.z
    rot = transformations.quaternion_matrix(quat)
    wp = np.array([waypoint["x"], waypoint["y"], 0])
    rot_wp = np.array(np.matrix(wp) * np.matrix(rot[:3, :3]))[0]
    rot_wp *= config.occ_grid.info.resolution
    rot_wp[0] += origin.position.x
    rot_wp[1] += origin.position.y
    rot_wp[2] += origin.position.z
    ps.pose.position.x = rot_wp[0]
    ps.pose.position.y = rot_wp[1]
    ps.pose.position.z = rot_wp[2]
    return ps


@config.app.route("/updates", methods=["GET"])
def get_updates():
    updates_to_ship = jsonify(config.occ_grid_updates)
    config.occ_grid_updates = dict()
    return updates_to_ship


@config.app.route("/grid", methods=["GET"])
def get_grid():
    grid = dict()
    try:
        grid["height"] = config.occ_grid.info.height
        grid["width"] = config.occ_grid.info.width
        grid["data"] = np.reshape(np.array(list(config.occ_grid.data)),
                                  (grid["height"], grid["width"])).tolist()
    except AttributeError:
        grid["height"] = 200
        grid["width"] = 200
        grid["data"] = np.zeros((200, 200)).tolist()
    return jsonify(grid)


@config.app.route("/tour", methods=["GET"])
def get_tour():
    while not config.new_tour:
        pass

    tour = list()
    mmd = config.occ_grid.info
    for ps in config.tour.poses:
        pos = dict()
        pos["x"] = int(ps.pose.position.x / mmd.resolution)
        pos["y"] = int(ps.pose.position.y / mmd.resolution)
        tour.append(pos)
    config.new_tour = False
    return json.dumps(tour)


@config.app.route("/waypoints", methods=["POST"])
def post_waypoints():
    waypoints = json.loads(request.form["waypoints"])
    config.waypoints = waypoints
    path = Path()
    for waypoint in waypoints:
        ps = transform_waypoint(waypoint)
        # print ps
        path.poses.append(ps)
    config.waypoints_pub.publish(path)
    return ""
