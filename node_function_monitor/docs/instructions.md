# Using node_function_monitor
In this package we show a simple example of functional level node monitoring.

## How it works
A talker node is used to continuoulsy publish messages to a topic, while the listener node will listen for these messages and send an OK message to the diagnostic topic if the messages successfully arrive. If for some reason the messages from the talker node are not being published, the listener node will report an error to the diagnostic topic. The messages published to the diagnostic topic can be viewed using a rqt_plugin called rqt_runtime_monitor.

``Note: In this example we are only monitoring the listener node``

## How to run it
``Note: We assume your catkin workspace is called catkin_ws and is located at ~/catkin_ws``

- Go to your catkin workspace folder, build it and source it if you have not yet.

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

- Run rqt_runtime_monitor
```sh
rosrun rqt_runtime_monitor rqt_runtime_monitor
```

- In another terminal, launch the listener node. You should be able to see an error on the rqt_runtime_monitor because the talker node has not yet been launched.
```sh
roslaunch node_function_monitor listener_test.launch
```

In another terminal, launch the talker node. Now in the rqt_runtime_monitor, there should no longer have an error message but now an OK message.
```sh
roslaunch node_function_monitor talker_test.launch
```
