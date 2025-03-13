# ros-jazzy-ros1-bridge-builder
Create a "*ros-jazzy-ros1-bridge*" package that can be used directly within Ubuntu 24.04 (Noble) ROS2 Jazzy. Currently, only AMD64 architectures is supported due to lack of ARM64 builds in the ppa:ros-for-jammy/noble repository.

- Note1: It takes approximately 10 minutes on my PC, equipped with a 6-core CPU (12 logical cores) and 24GB of memory.

- Note2: It takes about 1 GB of memory per logical CPU core to compile the ROS1 bridge. So, if your system has only 4 GB of memory but 100 logical CPU cores, it will still use only 4 logical cores for the compilation. Now, why does it take so much memory to compile?  Well, you can blame the overuse of C++ templates...

- Note3: To add support for other topics, see also https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder


## How to create this builder docker images:

``` bash
  git clone https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder.git
  cd ros-jazzy-ros1-bridge-builder
  docker build . -t ros-jazzy-ros1-bridge-builder
```

- Note1: Since building a docker image just needs docker, you could do this step on any system that has docker installed -- it doesn't have to on a Ubuntu 24.04 (Noble) and it doesn't need ROS2 neither.

## How to create ros-jazzy-ros1-bridge package:
###  0.) Start from the latest Ubuntu 24.04 (Noble) ROS 2 Jazzy Desktop system, create the "ros-jazzy-ros1-bridge/" ROS2 package:

``` bash
    cd ~/
    apt update; apt upgrade
    apt -y install ros-jazzy-desktop
    docker run --rm ros-jazzy-ros1-bridge-builder | tar xvzf -
```

- Note1: It's **important** that you have **`ros-jazzy-desktop`** installed on your ROS2 Jazzy system because we want to **match it with the builder image as closely as possible**.  So, if you haven't done so already, do:
``` bash
    apt -y install ros-jazzy-desktop
```

- Note1: There is no compilation at this point, the `docker run` command simply spits out a pre-compiled tarball for either amd64 or arm64 architecture, depending on the architecture of the machine you used to created the builder image.

- Note2: The assumption is that this tarball contains configurations and libraries matching your ROS2 Jazzy system very closely, although not identical.

- Note3: We don't really need the builder image anymore, to delete it, do:

``` bash
    docker rmi ros-jazzy-ros1-bridge-builder
```

## How to use ros-jazzy-ros1-bridge:
###  1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:

``` bash
  rocker --x11 --user --privileged \
         --volume /dev/shm /dev/shm --network=host -- ros:noetic-ros-base-focal \
         'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'
```

Tha docker image used above, `ros:noetic-ros-base-focal`, is multi-platform.  It runs on amd64 (eg., Intel and AMD CPUs) or arm64 architecture (eg., Raspberry PI 4B and Nvidia Jetson Orin).  Docker will automatically select the correct platform variant based on the host's architecture.

You may need to install rocker first:
``` bash
  sudo apt install python3-rocker
```
- Note1: It's important to share the host's network and the `/dev/shm/` directory with the container.
- Note2: You can add the `--home` rocker option if you want your home directory to be shared with the docker container.  Be careful though, as the host's `~/.bashrc` will be executed inside the container.
- Note3: You can also use **ROS1 Melodic**.  Just replace `ros:noetic-ros-base-focal` with `ros:melodic-ros-base-bionic` and also replace `ros-noetic-rospy-tutorials` with `ros-melodic-rospy-tutorials`.

###  2.) Then, start "roscore" inside the ROS1 Noetic docker container

``` bash
  source /opt/ros/noetic/setup.bash
  roscore
```

###  3.) Now, from the Ubuntu 24.04 (Noble) ROS2 Jazzy system, start the ros1 bridge node.

``` bash
  source /opt/ros/jazzy/setup.bash
  source ~/ros-jazzy-ros1-bridge/install/local_setup.bash
  ros2 run ros1_bridge dynamic_bridge
  # or try (See Note2):
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
- Note: We need to source `local_setup.bash` and NOT `setup.bash` because the bridge was compiled in a docker container that may have different underlay locations.  Besides, we don't need to source these underlays in the host system again.

- Note2: https://github.com/ros2/ros1_bridge states that: "For efficiency reasons, topics will only be bridged when matching publisher-subscriber pairs are active for a topic on either side of the bridge. As a result **using ros2 topic echo <_topic-name_>**  doesn't work but fails with an error message Could not determine the type for the passed topic if no other subscribers are present **since the dynamic bridge hasn't bridged the topic yet**. As a **workaround** the topic type can be specified explicitly **ros2 topic echo <_topic-name_> <_topic-type_>** which triggers the bridging of the topic since the echo command represents the necessary subscriber. On the ROS 1 side rostopic echo doesn't have an option to specify the topic type explicitly. Therefore it can't be used with the dynamic bridge if no other subscribers are present. As an alternative you can use the **--bridge-all-2to1-topics option** to bridge all ROS 2 topics to ROS 1 so that tools such as rostopic echo, rostopic list and rqt will see the topics even if there are no matching ROS 1 subscribers. Run ros2 run ros1_bridge dynamic_bridge -- --help for more options."
``` bash
    $ ros2 run ros1_bridge dynamic_bridge --help
    Usage:
     -h, --help: This message.
     --show-introspection: Print output of introspection of both sides of the bridge.
     --print-pairs: Print a list of the supported ROS 2 <=> ROS 1 conversion pairs.
     --bridge-all-topics: Bridge all topics in both directions, whether or not there is a matching subscriber.
     --bridge-all-1to2-topics: Bridge all ROS 1 topics to ROS 2, whether or not there is a matching subscriber.
     --bridge-all-2to1-topics: Bridge all ROS 2 topics to ROS 1, whether or not there is a matching subscriber.
```


###  3.) Back to the ROS1 Noetic docker container, run in another terminal tab:

``` bash
  source /opt/ros/noetic/setup.bash
  rosrun rospy_tutorials talker
```

###  4.) Finally, from the Ubuntu 24.04 (Noble) ROS2 Jazzy system, run in another terminal tab:

``` bash
  source /opt/ros/jazzy/setup.bash
  ros2 run demo_nodes_cpp listener
```


## How to make it work with ROS1 master running on a different machine?
- Run `roscore` on the Noetic machine as usual.
- On the Jazzy machine, run the bridge as below (assuming the IP address of the Noetic machine is 192.168.1.208):

``` bash
  source /opt/ros/jazzy/setup.bash
  source ~/ros-jazzy-ros1-bridge/install/local_setup.bash
  ROS_MASTER_URI='http://192.168.1.208:11311' ros2 run ros1_bridge dynamic_bridge
  # Note, change "192.168.1.208" above to the IP address of your Noetic machine.
```

## Troubleshoot

### Fixing "[ERROR] Failed to contact master":

If you have Noetic and Jazzy running on two different machines and have
already set the ROS_MASTER_URI environment variable, you should check the
network to ensure that the Jazzy machine can reach the Noetic machine via
port 11311.

``` bash
$ nc -v -z 192.168.1.208 11311
# Connection to 192.168.1.208 11311 port [tcp/*] succeeded!
```

### Checking tf2 message / service:
``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i tf2
  - 'tf2_msgs/msg/TF2Error' (ROS 2) <=> 'tf2_msgs/TF2Error' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf2_msgs/TFMessage' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf/tfMessage' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```


## References
- https://github.com/ros2/ros1_bridge
- https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst
- https://github.com/smith-doug/ros1_bridge/tree/action_bridge_humble
- https://github.com/smith-doug/ros1_bridge/tree/action_bridge_humble
- https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder
