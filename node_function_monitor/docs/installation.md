# Installation
The **node_function_monitor** package is used to run nodes that will publish diagnostic messages to the diagnostic topic. The messages from this topic can viewed using the **rqt_runtime_monitor** package.

### node_function_monitor

You may directly clone this project into your catkin workspace's **src** folder; however, we recommend cloning our project into a separate folder and creating a symbolic link from the **node_function_monitor** package to your catkin workspace's **src** folder.

``Note: We assume your catkin workspace is called catkin_ws and is located at ~/catkin_ws``

- Create a gitlab_proj folder.
```sh
mkdir -p ~/gitlab_proj
```

- Go into the gitlab_proj folder and clone our project.
```sh
cd ~/gitlab_proj
git clone git@e300-gitlab.itriicle.tw:RMS/ROS/survey_ros_diagnostic_tool.git
```

- Create symbolic link from the **node_function_monitor** package to your catkin workspace's **src** folder.
```sh
ln -s ~/gitlab_proj/survey_ros_diagnostic_tool/node_function_monitor ~/catkin_ws/src
```

- Go into your catkin workspace folder and build it.
```sh
cd ~/catkin_ws
catkin_make
```

### rqt_runtime_monitor

You may directly clone **rqt_runtime_monitor** into your catkin workspace's **src** folder; however, we recommend cloning it into a separate folder and creating a symbolic link from the **rqt_runtime_monitor** package to your catkin workspace's **src** folder.

``Note: At this point we assume you have created a gitlab projects folder: ~/gitlab_proj``

- Go into the gitlab_proj folder and clone the package.
```sh
cd ~/gitlab_proj
git clone https://github.com/ros-visualization/rqt_runtime_monitor.git
```

- Create symbolic link from the **rqt_runtime_monitor** package to your catkin workspace's **src** folder.
```sh
ln -s ~/gitlab_proj/rqt_runtime_monitor ~/catkin_ws/src
```

- Go into your catkin workspace folder and build it.
```sh
cd ~/catkin_ws
catkin_make
```

## Reference

- http://wiki.ros.org/rqt_runtime_monitor?distro=kinetic
- http://wiki.ros.org/diagnostic_msgs?distro=kinetic