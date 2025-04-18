# ROS 2 Jazzy to ROS 1 Noetic bag converter

**Warning: messages may get dropped as more messages per sencond are sent**

## Build
```
docker build . -t ros-jazzy-ros1-bridge-builder
```

## Run bridge

#### Terminal ROS 1
```
docker network rm ros1-ros2-bridge-network
docker network create -d bridge ros1-ros2-bridge-network
docker run -it --rm \
           --volume ${PWD}/output:/output:rw \
           --network=ros1-ros2-bridge-network \
           ros:noetic-ros-base-focal 
     
source /opt/ros/noetic/setup.bash
roscore &
```

#### Terminal ROS 2
```
docker run -it --rm \
        --volume ${PWD}/input:/input:ro \
        --network=ros1-ros2-bridge-network \
        ros-jazzy-ros1-bridge-builder bash

source /opt/ros/jazzy/setup.bash
source /ros-jazzy-ros1-bridge/install/local_setup.bash
ROS_MASTER_URI=http://172.18.0.2:11311 ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &
```

## Test
#### Terminal ROS 2
```
ros2 run demo_nodes_cpp listener
```

#### Terminal ROS 1
```
sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials 
rosrun rospy_tutorials talker
```

## Convert rosbag
#### Terminal ROS 2
```
ros2 bag play /input/bag --clock --rate 0.5 --wait-for-all-acked 1000 --read-ahead-queue-size 5000

ros2 bag info /input/
```

#### Terminal ROS 1
```
cd /output
rosbag record --buffsize=1024 --all
rosbag info 
```
