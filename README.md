NRL Waypoint Selector
=====================

An interactive waypoint selector that uses a web-based user interface
to allow a user to select waypoints inside an occupancy grid. The waypoints
selected can then be used by different packages and to plan a route
between them.

![wp](/sandbox/readme.png)


# Installation

    $ sudo pip install -r requirements.txt

# Running
Firstly, you will have to start the ROS nodes that will publish the occupancy
grid and will determine the tour around the waypoints before the waypoint
selector is started. To start the waypoint controller run:

    $ roslaunch waypoint_selector waypoint_selector.launch

This will start the server on your computer using localhost as the
default hostname and 8080 as the default port. Once the node has been
launched, you are able to use the UI by going to your browser and 
going to `http://localhost:8080`.

# Parameters
Here is a list of parameters and their descriptions as used in the 
`waypoint_selector.launch` file.

- `host`, `port`
    - The hostname and port number where the server will run
