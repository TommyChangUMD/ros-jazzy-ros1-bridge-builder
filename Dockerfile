FROM ros:jazzy-ros-base-noble
# The above base image is multi-platform (works on ARM64 and AMD64):
# Docker will automatically select the correct platform variant based on the host's architecture.

#
# How to build this docker image:
#  docker build . -t ros-jazzy-ros1-bridge-builder
#
# How to build ros-jazzy-ros1-bridge:
#  # 0.) From a Ubuntu 24.04 (Noble) ROS 2 Jazzy system, create a "ros-jazzy-ros1-bridge/" ROS2 package:
#    docker run --rm ros-jazzy-ros1-bridge-builder | tar xvzf -
#
# How to use the ros-jazzy-ros1-bridge:
#  # 1.) First start a ROS1 Noetic docker and bring up a GUI terminal, something like:
#    rocker --x11 --user --privileged \
#         --volume /dev/shm /dev/shm --network=host -- ros:noetic-ros-base-focal \
#         'bash -c "sudo apt update; sudo apt install -y ros-noetic-rospy-tutorials tilix; tilix"'
#
#  # 2.) Then, start "roscore" inside the ROS1 container:
#    source /opt/ros/noetic/setup.bash
#    roscore
#
#  # 3.) Now, from the Ubuntu 24.04 (Noble) ROS2 Desktop Jazzy system, start the ros1 bridge node:
#    apt-get -y install ros-jazzy-desktop
#    source /opt/ros/jazzy/setup.bash
#    source ros-jazzy-ros1-bridge/install/local_setup.bash
#    ros2 run ros1_bridge dynamic_bridge
#
#  # 4.) Back to the ROS1 Noetic container, run in another terminal tab:
#    source /opt/ros/noetic/setup.bash
#    rosrun rospy_tutorials talker
#
#  # 5.) Finally, from the Ubuntu 24.04 (Noble) ROS2 Jazzy system:
#    source /opt/ros/jazzy/setup.bash
#    ros2 run demo_nodes_cpp listener
#

# Make sure bash catches errors (no need to chain commands with &&, use ; instead)
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

###########################
# 1.) Bring system up to the latest ROS desktop configuration
###########################

RUN apt-get update; \
    apt-get -y install ros-jazzy-desktop; \
    rm -rf /var/lib/apt/lists/*


###########################
# 5.) Install ROS1 Noetic desktop
# (Currently, ppa contains AMD64 builds only)
###########################

RUN apt-get update; \
    apt-get -y install software-properties-common; \
    rm -rf /var/lib/apt/lists/*
RUN add-apt-repository ppa:ros-for-jammy/noble
RUN apt-get update; \
    apt -y install ros-noetic-desktop; \
    rm -rf /var/lib/apt/lists/*

# fix ARM64 pkgconfig path issue -- Fix provided by ambrosekwok 
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then                     \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/;   \
    fi

###########################
# 6.) Compile custom msgs
###########################

COPY custom_msgs /custom_msgs
RUN \
    # Build the ros1 workspace
    cd /custom_msgs/custom_msgs_ros1_ws && \
    unset ROS_DISTRO && \
    source /opt/ros/noetic/setup.bash && \
    time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Build the ros2 workspace
    cd /custom_msgs/custom_msgs_ros2_ws && \
    unset ROS_DISTRO && \
    source /opt/ros/jazzy/setup.bash && \
    time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

###########################
# 7.) Compile ros1_bridge
###########################

# g++-11 and needed
RUN apt-get update; \
    apt-get -y install g++-11 gcc-11; \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 11; \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 11; \
    rm -rf /var/lib/apt/lists/*

RUN                                                                                    \
    #-------------------------------------                                             \
    # Get the Bridge code                                                              \
    #-------------------------------------                                             \
    mkdir -p /ros-jazzy-ros1-bridge/src;                                               \
    cd /ros-jazzy-ros1-bridge/src;                                                     \
    git clone -b action_bridge_humble https://github.com/smith-doug/ros1_bridge.git;   \
    cd ros1_bridge/;                                                                   \
                                                                                       \
    #-------------------------------------                                             \
    # Apply the ROS1 and ROS2 underlays                                                \
    #-------------------------------------                                             \
    source /opt/ros/noetic/setup.bash;                                                 \
    source /opt/ros/jazzy/setup.bash;                                                  \
                                                                                       \
    #-------------------------------------                                             \
    # Apply the ROS1 and ROS2 overlays                                                 \
    #-------------------------------------                                             \
    source /custom_msgs/custom_msgs_ros1_ws/install/local_setup.bash;                  \
    source /custom_msgs/custom_msgs_ros2_ws/install/local_setup.bash;                  \
                                                                                       \
    #-------------------------------------                                             \
    # Finally, build the Bridge                                                        \
    #-------------------------------------                                             \
    MEMG=$(printf "%.0f" $(free -g | awk '/^Mem:/{print $2}'));                        \
    NPROC=$(nproc);  MIN=$((MEMG<NPROC ? MEMG : NPROC));                               \
    cd /ros-jazzy-ros1-bridge/;                                                        \
    echo "Please wait...  running $MIN concurrent jobs to build ros1_bridge";          \
    time ROS_DISTRO=humble MAKEFLAGS="-j $MIN" colcon build                            \
        --event-handlers console_direct+                                               \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

###########################
# 9.) Pack all ROS1 dependent libraries
###########################
# fix ARM64 pkgconfig path issue -- Fix provided by ambrosekwok 
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then                    \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/;  \
    fi

RUN ROS1_LIBS="libactionlib.so";                                                \
    ROS1_LIBS="$ROS1_LIBS libroscpp.so";                                        \
    ROS1_LIBS="$ROS1_LIBS librosconsole.so";                                    \
    ROS1_LIBS="$ROS1_LIBS libroscpp_serialization.so";                          \
    ROS1_LIBS="$ROS1_LIBS librostime.so";                                       \
    ROS1_LIBS="$ROS1_LIBS libxmlrpcpp.so";                                      \
    ROS1_LIBS="$ROS1_LIBS libcpp_common.so";                                    \
    ROS1_LIBS="$ROS1_LIBS librosconsole_log4cxx.so";                            \
    ROS1_LIBS="$ROS1_LIBS librosconsole_backend_interface.so";                  \
    ROS1_LIBS="$ROS1_LIBS liblog4cxx.so.15";                                    \
    ROS1_LIBS="$ROS1_LIBS libaprutil-1.so.0";                                   \
    ROS1_LIBS="$ROS1_LIBS libapr-1.so.0";                                       \
    cd /ros-jazzy-ros1-bridge/install/ros1_bridge/lib;                          \
    source /opt/ros/noetic/setup.bash;                                          \
    for soFile in $ROS1_LIBS; do                                                \
      soFilePath=$(ldd libros1_bridge.so | grep $soFile | awk '{print $3;}');   \
      cp -L $soFilePath ./;                                                     \
    done

###########################
# 10.) Spit out ros1_bridge tarball by default when no command is given
###########################
RUN tar czf /ros-jazzy-ros1-bridge.tgz \
     --exclude '*/build/*' --exclude '*/src/*' /ros-jazzy-ros1-bridge 
ENTRYPOINT []
CMD cat /ros-jazzy-ros1-bridge.tgz; sync
