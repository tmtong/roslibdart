# roslibdart

The is a fork of the original roslib by @artrmz, @TimWhiting, @artrmz. roslibdart is a library for communicating to a ROS node over websockets with rosbridge. It is heavily influenced by roslibjs and follows the same structure. This fork is an effort to update the library and make it compatible to ros2 and dart2.

## List of feature implementation statuses (essentially a list of features required to reach roslibjs's level)
- [X] Core:
  - [x] ROS connection object
  - [x] Topic object (subscribe, unsubscribe, publish, advertise, unadvertise)
  - [x] Service object (call, advertise, unadvertise)
  - [x] Request object (provides typing and naming to any potential ROS request)
  - [x] Param object (get, set, delete)
- [ ] Actionlib:
  - [ ] ActionClient
  - [ ] ActionListener
  - [ ] Goal
  - [ ] SimpleActionServer
- [ ] Support TCP connections
- [ ] TFClient (listen to TFs from tf2_web_republisher)
- [ ] URDF (Box, Color, Cylinder, Joint, Material, Mesh, Model, Sphere, Types, Visual)

## Usage
### Install rosbridge
You can either
```
# install using apt-get
sudo apt-get install ros-foxy-rosbridge-server
#  or you can clone from github
cd ros_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite
cd ..
colcon build
```
### Run ros bridge
```
source install/local_setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
### Run topic publisher
```
# Assume ros_ws is your workspace
cp -r rospackage/* ~/ros_ws/src/
cd ~/ros_ws
colcon build
source install/local_setup.bash
ros2 run tutorial publisher
```
### Fire up flutter subscriber
```
cd example/subscriber
flutter run -d linux
```


