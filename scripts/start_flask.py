import argparse
from flask import Flask, render_template, request
import os
from urllib.parse import urlparse


filename = os.path.abspath(__file__)
scriptsdir = os.path.dirname(filename)
basedir = os.path.dirname(scriptsdir) 
app = Flask(__name__, root_path=basedir)

@app.route("/")
def hello_world():
    # figure out what url the client computer thinks we are on
    hosturl = urlparse(request.host_url)
    
    return render_template('index.html', 
        host=hosturl.hostname, # contact the same server for ws that we used for http
        ws_proto="wss" if hosturl.scheme == "https" else "ws", # use wss if secure connection
        **app.config.get_namespace("PAGEPARAMS_")) # load in any params we specified in the config (ws_port)



if __name__ == '__main__': 
    # set up parser to get args from command line AND rosparam (if available)
    # todo: make this work also from a config file
    parser = argparse.ArgumentParser(description="launch web-based joystick interface")
    parser.add_argument("-n", "--hostname", help="hostname to launch on")
    parser.add_argument("-p", "--port", help="port to launch on")
    parser.add_argument("-w", "--ws-port", default="9090", help="websocket port to use")
    parser.add_argument("-d", "--debug", action="store_true", help="launch in debug mode")
    parser.add_argument("--toggle", action="store_true", help="run in button toggle mode rather than hold mouse click mode")
    
    # load default values using rospy, if available
    try:
        import rospy
    except ImportError:
        pass
    else:
        rospy.init_node("joy_server", anonymous=True)
        # set all the params on the local parameter server as defaults in the parser
        # NB: ignores keys on the param server that are not already defined as args
        # if there is a ros parameter called "help" this will cause weird behavior, so don't
        # have one or make this smarter if it's for some reason necessary
        parser.set_defaults(**rospy.get_param("~"))

        # cleanup; have to release manually since import is global
        rospy = None
    
    args = parser.parse_args()

    # put the args into the app config
    # this probably can be done in a smarter way but that's more complicated
    # do each var explicitly as best practice against weird code injection 

    hostname = args.hostname or "127.0.0.1"
    port = args.port or "5000"
    app.config["SERVER_NAME"] = f"{hostname}:{port}"
    print(app.config["SERVER_NAME"])

    # values to be passed directly to the template(s) to render
    # keys must be all-caps but will be turned back to lowercase inside the template
    # to pass a parameter along internally, use the prefix PAGEPARAMS_ and it should
    # automatically show up when the template is rendered
    app.config["PAGEPARAMS_WS_PORT"] = args.ws_port   
    app.config["PAGEPARAMS_TOGGLE"] = bool(args.toggle)

    app.run()

