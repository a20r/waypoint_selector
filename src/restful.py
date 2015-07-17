
import config
from flask import request, jsonify


@config.app.route("/say/<name>", methods=["GET"])
def get_say(name):
    try:
        words = config.say_store[name]
        del config.say_store[name]
        return jsonify(words=words)
    except KeyError:
        return jsonify(words="")
