#!/usr/bin/env python3

import argparse
from flask import abort, Flask, render_template, request
import os
import sys
from urllib.parse import urlparse


filename = os.path.abspath(__file__)
scriptsdir = os.path.dirname(filename)
basedir = os.path.dirname(scriptsdir) 
app = Flask(__name__, root_path=basedir)

MODES = ("toggle", "hold", "jog")

@app.route("/")
@app.route("/<mode>")
def hello_world(mode=None):
    if mode is not None and not mode in MODES:
        abort(404)

    if mode is None:
        mode = app.config["MODE"]

    # figure out what url the client computer thinks we are on
    hosturl = urlparse(request.host_url)
    
    return render_template(f'{mode}.html', 
        host=hosturl.hostname, # contact the same server for ws that we used for http
        **app.config.get_namespace("PAGEPARAMS_")) # load in any params we specified in the config (ws_port)


if __name__ == '__main__': 
    # set up parser to get args from command line AND rosparam (if available)
    # todo: make this work also from a config file
    parser = argparse.ArgumentParser(description="launch web-based joystick interface")
    parser.add_argument("-n", "--hostname", help="hostname to launch on", default="127.0.0.1")
    parser.add_argument("-p", "--port", help="port to launch on", default="5000")
    parser.add_argument("-w", "--ws-port", default="9090", help="websocket port to use")
    parser.add_argument("-d", "--debug", action="store_true", help="launch in debug mode")
    parser.add_argument("-m", "--mode", choices=MODES, help="control mode to use", default="toggle")
    parser.add_argument("-s", "--secure", action="store_true", help="using secure (https/wss) mode")
    
    # load default values using rospy, if available
    try:
        import rospy
    except ImportError:
        rospy = None
    else:
        rospy.init_node("joy_server", anonymous=True, disable_signals=True)
        # set all the params on the local parameter server as defaults in the parser
        # NB: ignores keys on the param server that are not already defined as args
        # if there is a ros parameter called "help" this will cause weird behavior, so don't
        # have one or make this smarter if it's for some reason necessary
        if rospy.has_param("~"):
            parser.set_defaults(**rospy.get_param("~"))
        
        sys.argv = rospy.myargv(argv=sys.argv)
    
    args = parser.parse_args()

    # put the args into the app config
    # this probably can be done in a smarter way but that's more complicated
    # do each var explicitly as best practice against weird code injection 

    app.config["SERVER_NAME"] = f"{args.hostname}:{args.port}"
    if rospy:
        rospy.loginfo(f"Control server running on {app.config['SERVER_NAME']}")
    else:
        print(f"Control server running on {app.config['SERVER_NAME']}")

    # default mode, can be overridden
    app.config["MODE"] = args.mode

    # values to be passed directly to the template(s) to render
    # keys must be all-caps but will be turned back to lowercase inside the template
    # to pass a parameter along internally, use the prefix PAGEPARAMS_ and it should
    # automatically show up when the template is rendered
    app.config["PAGEPARAMS_SECURE"] = args.secure
    app.config["PAGEPARAMS_WS_PORT"] = args.ws_port

    app.run()

    # clean up ros
    if rospy:
        rospy.signal_shutdown("sigint")

