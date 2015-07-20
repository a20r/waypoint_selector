
import config
import numpy as np
from flask import jsonify


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
    waypoints = request.form["waypoints"]
    config.waypoints = waypoints
    for waypoint in waypoints:
        pass
