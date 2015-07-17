
from flask import Flask

app = Flask(__name__)
app.config.from_object(__name__)

occ_grid = None
occ_grid_updates = dict()
waypoints = list()
