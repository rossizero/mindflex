# mindflex
## Setup
Of the Repo

```
git clone --recurse-submodules https://github.com/rossizero/mindflex.git
or if already cloned
git submodule update --init --recursive
```
Of VS Code ESP-IDF
* install extension
* select idf version 5.2.* (!!)
* open the folder eeg-micro-ros in vs code
* in terminal

```
(. $IDF_PATH/export.sh)
pip3 install catkin_pkg lark-parser colcon-common-extensions
# next steps also possible in vs code extension
idf.py set-target esp32 
# Set your micro-ROS configuration and WiFi credentials under micro-ROS Settings
idf.py menuconfig (set ssid, password, microros port and ip of the microros_agent host)
idf.py build
idf.py flash
idf.py monitor
```
Of microros_agent: 
https://micro.ros.org/docs/tutorials/core/first_application_linux/

```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
You should now see the esp connecting via the microros_agent
