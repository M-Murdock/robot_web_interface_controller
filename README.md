# robot_web_interface_controller

## Setup:
1. Clone the repo into your ROS package
2. Install rosbridge: http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge  

## Configuration:
- ``WebXYZMode.yaml`` has the mapping for the web controller. You can change this to be whatever you like. 
- ``start_flask.py`` starts up the flask server. On line 11, change the ``host`` to the correct address.
- ``templates/index.html`` is the web interface. Change line 63 to be whatever address you just assigned to ``host``. Edit this file to change the number/arrangement of buttons, or whatever else you like.

## Running:

### On the robot:
1. Connect the device of your choice to the robot network (``robot2`` or ``robot1``). 
2. On the robot, run ``roslaunch rosbridge_server rosbridge_websocket.launch`` and ``python3 start_flask.py``
3. On your device, navigate to the correct url 

### On your computer:
1. Connect your computer and the device of your choice to the robot network (``robot2`` or ``robot1``)
2. On your computer, run ``roslaunch rosbridge_server rosbridge_websocket.launch`` and ``python3 start_flask.py``
3. On your device, navigate to the correct url