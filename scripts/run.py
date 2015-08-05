#!/usr/bin/env python

__all__ = ["pageserver", "restful"]


import point
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import scipy.spatial as spatial
import signal
import sys
import rospy
import pageserver
import restful
import config
from nav_msgs.msg import OccupancyGrid, Path, Odometry


def get(grid, i, j):
    h = grid.info.height
    return grid.data[h * i + j]


def nn_roadmap(N, k, grid, **kwargs):
    w = kwargs["width"]
    data = np.random.uniform(low=0, high=w - 1, size=(N, 2))
    return nn_roadmap_given_points(N, k, grid, data, **kwargs)


def nn_roadmap_given_points(N, k, grid, data, width=1, height=1, dtype=int):
    G = nx.Graph()
    tree = spatial.KDTree(data)
    for i, vec in enumerate(data):
        x = dtype(vec[0])
        y = dtype(vec[1])
        if get(grid, x, y) == 0:
            G.add_node(i, position=point.Point(x, y))
            ds, inds = tree.query(vec, k=k)
            for d, j in zip(ds, inds):
                if get(grid, int(data[j][0]), int(data[j][1])) == 0:
                    p2 = point.Point(data[j][0], data[j][1])
                    if not crow_collision(grid, G.node[i]["position"], p2):
                        G.add_edge(i, j, distance=d)
    return G


def crow_path(p, q):
    path = list()
    uv = (q - p).to_unit_vector()
    inter = p.copy()
    while q.dist_to(inter) > 1:
        inter = inter + uv
        path.append(inter.intify())
    return path


def crow_collision(grid, p, q):
    path = crow_path(p, q)
    for i in path:
        if get(grid, i.get_x(), i.get_y()) > 0:
            return True
    return False


def draw(G):
    positions = dict()
    for node_id in G.nodes():
        positions[node_id] = G.node[node_id]['position'].to_list_2d()
    nx.draw_networkx_edges(G, positions)
    nx.draw_networkx_nodes(G, positions, node_size=30)


def get_skeleton(og, N):
    w = og.info.width
    h = og.info.height
    G = nn_roadmap(N, 20, og, width=w, height=h)
    draw(G)
    plt.savefig("/tmp/test.pdf")
    skel = nx.Graph()
    open_set = [G.nodes()[0]]
    already_seen_n = set()
    already_seen_o = set()
    while len(open_set) > 0:
        i = open_set.pop()
        already_seen_o.add(i)
        neighbour_set = [i]
        while len(neighbour_set) > 0:
            m = neighbour_set.pop()
            already_seen_n.add(m)
            m_pos = G.node[m]["position"]
            for n in G.neighbors(m):
                n_pos = G.node[n]["position"]
                for n_2 in G.neighbors(n):
                    n_2_pos = G.node[n_2]["position"]
                    if crow_collision(og, m_pos, n_2_pos):
                        if not n in already_seen_o:
                            open_set.append(n)
                        skel.add_node(m, position=m_pos)
                        skel.add_node(n, position=n_pos)
                        skel.add_edge(m, n)
                    else:
                        if not n in already_seen_n:
                            neighbour_set.append(n)
    return skel


def update_occ_grid(og):
    skel = get_skeleton(og, 100)
    print skel.number_of_nodes()
    # draw(skel)
    # plt.savefig("/tmp/test.png")
    w = og.info.width
    h = og.info.height

    if config.occ_grid is None:
        config.occ_grid = og
        return

    for i in xrange(w):
        for j in xrange(h):
            if not get(og, i, j) == get(config.occ_grid, i, j):
                if get(og, i, j) > 0:
                    config.occ_grid_updates.append([i, j, 100])
                else:
                    config.occ_grid_updates.append([i, j, 0])


def update_robot_odom(odom):
    config.odom = odom


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
    odom_topic = rospy.get_param("~odom_topic")
    tour_topic = rospy.get_param("~tour_topic")
    occ_sub = rospy.Subscriber(occ_topic, OccupancyGrid, update_occ_grid)
    odom_sub = rospy.Subscriber(odom_topic, Odometry, update_robot_odom)
    tour_sub = rospy.Subscriber(tour_topic, Path, update_tour)
    config.waypoints_pub = rospy.Publisher(waypoints_topic, Path, queue_size=0)
    config.app.run(host=host, port=port, use_reloader=False, debug=True,
                   threaded=False)
    rospy.spin()
