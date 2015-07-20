
import config
import json
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from flask import jsonify, request


@config.app.route("/updates", methods=["GET"])
def get_updates():
    updates_to_ship = jsonify(config.occ_grid_updates)
    config.occ_grid_updates = dict()
    return updates_to_ship


@config.app.route("/grid", methods=["GET"])
def get_grid():
    grid = dict()
    grid["height"] = config.occ_grid.info.height
    grid["width"] = config.occ_grid.info.width
    grid["data"] = np.reshape(np.array(list(config.occ_grid.data)),
                              (grid["height"], grid["width"])).tolist()
    return jsonify(grid)


@config.app.route("/waypoints", methods=["POST"])
def post_waypoints():
    waypoints = json.loads(request.form["waypoints"])
    config.waypoints = waypoints
    path = Path()
    for waypoint in waypoints:
        ps = PoseStamped()
        ps.pose.position.x = waypoint["x"]
        ps.pose.position.y = waypoint["y"]
        path.poses.append(ps)
    config.waypoints_pub.publish(path)
    return ""
