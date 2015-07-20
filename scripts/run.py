#!/usr/bin/env python

__all__ = ["pageserver", "restful"]


import signal
import sys
import rospy
import pageserver
import restful
import config
from nav_msgs.msg import OccupancyGrid, Path


def update_occ_grid(occ_grid):
    # not really correct yet
    for i in xrange(len(occ_grid.data)):
        if config.occ_grid is None or not occ_grid.data[i]\
                == config.occ_grid.data[i]:
            config.occ_grid_updates[i] = occ_grid.data[i]
    config.occ_grid = occ_grid


def update_tour(tour):
    config.tour = tour
    config.new_tour = True


def signal_handler(signal, frame):
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("waypoint_selector", anonymous=False)
    host = rospy.get_param("~host")
    port = rospy.get_param("~port")
    occ_topic = rospy.get_param("~occupancy_grid_topic")
    waypoints_topic = rospy.get_param("~waypoints_topic")
    tour_topic = rospy.get_param("~tour_topic")
    occ_sub = rospy.Subscriber(occ_topic, OccupancyGrid, update_occ_grid)
    tour_sub = rospy.Subscriber(tour_topic, Path, update_tour)
    config.waypoints_pub = rospy.Publisher(waypoints_topic, Path, queue_size=0)
    config.app.run(host=host, port=port, use_reloader=False, debug=True,
                   threaded=False)
    rospy.spin()
