#!/usr/bin/env python

__all__ = ["pageserver", "restful"]


import sys
import rospy
import pageserver
import restful
import requests
import config
from flask import request
from nav_msgs.msg import OccupancyGrid


@config.app.route('/shutdown', methods=["GET"])
def shutdown_action():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    return 'Server shutting down...'


def get_shutdown_func(host, port):
    def __inner():
        requests.get("http://{}:{}/shutdown".format(host, port))
    return __inner


def update_occ_grid(occ_grid):
    if config.occ_grid is None:
        config.occ_grid = occ_grid
        print "Updated bromigo"


if __name__ == "__main__":
    if len(sys.argv) == 3:
        host = sys.argv[1]
        port = int(sys.argv[2])
        rospy.init_node("waypoint_selector", anonymous=False)
        sd_func = get_shutdown_func(host, port)
        rospy.on_shutdown(sd_func)
        occ_sub = rospy.Subscriber("topo_occupancy_grid", OccupancyGrid,
                                   update_occ_grid)
        config.app.run(host=host, port=port)
        rospy.spin()
    else:
        raise Exception("Correct argument form not supplied")
