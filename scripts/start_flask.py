from flask import Flask, render_template, request
import os
from urllib.parse import urlparse


filename = os.path.abspath(__file__)
scriptsdir = os.path.dirname(filename)
basedir = os.path.dirname(scriptsdir) 
app = Flask(__name__, root_path=basedir)

@app.route("/")
def hello_world():
    return render_template('index.html', 
        host=urlparse(request.host_url).hostname, 
        port="9090")

if __name__ == '__main__': 
    app.run(host="10.42.99.24", debug="true")
    # app.run(debug="true")
