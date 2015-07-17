#!/usr/bin/env python

__all__ = ["pageserver", "restful"]


import sys
import pageserver
import restful
import config


def run(host, port):
    config.app.run(host=host, port=int(port), debug=True)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        run(sys.argv[1], sys.argv[2])
    else:
        raise Exception("Correct argument form not supplied")
