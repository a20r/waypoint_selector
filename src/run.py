#!/usr/bin/env python

__all__ = ["pageserver", "restful"]


import signal
import sys
import sh
import rospy
import pageserver
import restful
import config
from nav_msgs.msg import OccupancyGrid


def update_occ_grid(occ_grid):
    # not really correct yet
    for i in xrange(len(occ_grid.data)):
        if config.occ_grid is None or not occ_grid.data[i]\
                == config.occ_grid.data[i]:
            config.occ_grid_updates[i] = occ_grid.data[i]
    config.occ_grid = occ_grid


def signal_handler(signal, frame):
    sys.exit(0)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        host = sys.argv[1]
        port = int(sys.argv[2])
        signal.signal(signal.SIGINT, signal_handler)
        rospy.init_node("waypoint_selector", anonymous=False)
        occ_sub = rospy.Subscriber("topo_occupancy_grid", OccupancyGrid,
                                   update_occ_grid)
        config.app.run(host=host, port=port, use_reloader=False, debug=True,
                       threaded=False)
        rospy.spin()
    else:
        raise Exception("Correct argument form not supplied")
