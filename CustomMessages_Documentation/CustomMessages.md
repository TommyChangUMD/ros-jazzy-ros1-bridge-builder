# Custom messages to the ros-jazzy-ros1-bridge
This document will describe the steps you need to take to add a custom message to your bridge.
I assume that you find a message type in ros1 that you need to be bridged to ros2, as a result, I start from ros1.
## Preparing the ros1 custom message package

If your custom messages are created in the same package that uses these messages, then the first step will be separating this package into two packages:

- One for the the custom message only
- One with the source code only (depending on the custom message package)

This way, it is possible to get the custom message package to be used for the bridge building.

> 👉 **Tip:** This guide assumes you only have custom messages not custom services as messages are more common; especially in ROS1. However, all the steps you take for the custom message do not differ for custom services, just some name differences. 


### Separating the custom messages from the remainder of a package

Navigate to the package directory that contains the custom message directory `msg`.

Check if there is a `src` directory with the `msg` directory, if there is, then you have to separate these two directories into two different packages.


> 👉 **Tip:** If your custom message package already has your custom messages directory (`msg`) separated in a stand alone package, then you can skip some of the steps relating to the cleaning of the packages



### Creating a directory to keep the hierarchy clean (if not done already)

In the `src` directory of the ros1 workspace, create a directory called `custom_msgs`.

### Duplicate the package

Copy the entire package into the `custom_msgs` directory that you have just made.

### Cleaning the custom message package

For the custom message package (the package in the `custom_msgs` directory). Change the name of the package directory to have `_msgs` at the end of it’s name. For example, if the original package name was `pseudo_grid_map`, change it to `pseudo_grid_map_msgs`.

> 👉 **Tip:** You can change the name of the package entirely as long as it’s name ends in `_msgs`. However, it is recommended to keep the name descriptive and clear for which package these custom messages are for. 


Inside the package, you should remove the `src` directory that has all the source code, they are not needed for the custom message package anymore.

Open the `CMakeLists.txt` file and change the project name to the new project name (the one ending in `_msgs`). Then remove all the dependencies except the ones relating to the custom messages.

> 👉 **Tip:** If you are not sure which dependencies were for the custom messages and which were for the source code, check the official custom message tutorial from ros1 noetic to see what is necessary. \
> [ROS/Tutorials/DefiningCustomMessages - ROS Wiki](https://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages)

Do something similar but for the `package.xml` file; renaming the package to the name ending in `_msgs`, and removing all dependencies not needed by the custom messages.

### Cleaning the original package

For the original package (the package in the `src` directory of the workspace). Remove the `msg` directory, as it is no longer needed.

Remove all the dependencies that have to do with custom messages in the `CMakeList.txt` and the `package.xml` file. These dependencies should be the opposite to what you removed for the custom message package (removing the dependencies you kept in the custom messages package). 

### Here is a working example of the above process of separating packages:

[Separating Custom msgs in a standalone package](https://github.com/Mohamed-Ahmed-Taha/ros-jazzy-ros1-bridge-builder/blob/main/CustomMessages_Documentation/ROS1_PackageSeparation.md)

### Building the ROS1 workspace

After separating the package into two packages. You should build the workspace again by doing

```bash
cd ~/catkin_ws
catkin_make
```

## Preparing the ros2 custom message package

The custom message package in ros2 will be very similar to the one in ros1. The only differences are the `CMakeList.txt` and the `package.xml` files.

### Creating a directory to keep the hierarchy clean (if not done already)

In the `src` directory of the ros2 workspace, create a directory called `custom_interfaces`.

### Duplicate the package

Copy the ros1 custom message package and paste it in the `custom_interfaces` directory of the ros2 workspace.

### Adjusting the package for ros2

You need to change some things in the `CMakeList.txt` and `package.xml` files for them to work with ros2.

You can follow these guides on how to transfer a package into ros2 from the official ros2 jazzy documentation:

[Migrating Interfaces — ROS 2 Documentation: Jazzy  documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Interfaces.html)

[Migrating your package.xml to format 2 — ROS 2 Documentation: Jazzy  documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Package-XML.html)

### Build the ROS2 workspace

After creating the ros2 custom message package, you need to build the workspace

```bash
cd ~/ros2_ws
colcon build
```

## Building the ros1-bridge

### Clone the ros-jazzy-ros1-bridge-builder (if not done already)

You need to clone the bridge builder [repository from GitHub](https://github.com/Mohamed-Ahmed-Taha/ros-jazzy-ros1-bridge-builder) using.

```bash
cd
git clone --recurse-submodules https://github.com/Mohamed-Ahmed-Taha/ros-jazzy-ros1-bridge-builder
```

### Adding the packages to the bridge

If you have the two custom message packages of ros1 and ros2 ready with no errors (both have been built successfully). Then all you need to do is:

- Copy the ros1 custom messages packages into the `custom_msgs/ustom_msgs_ros1_ws/src` directory that was cloned, beside any other custom messages packages already there.
    
    > 👉 **Tip:** There is an example custom message package there called `pseudo_grid_map_msgs`. If you do not need this custom message you can either just delete this package, or add it to your custom messages packages present in the `custom_interfaces` directory of the ros2 workspace. \
    > Note: Currently, to get this package, add --branch with-example to the clone command to get this example.    
    
- Copy the ros2 custom messages packages into the `custom_msgs/ustom_msgs_ros2_ws/src` directory that was cloned, beside any other custom messages packages already there (also delete this package if you have deleted the one in the ros2 side).

### Building the bridge image

You have to build the bridge using docker (install docker if you have not done so already)

```bash
cd ~/ros-jazzy-ros1-bridge-builder
docker build -t ros-jazzy-ros1-bridge:latest .
```

> 👉 **Tip:** The building of this image takes around 1 hour the first time it is built, then around 20 to 25 mins for rebuilding.


### Running the docker container

To run and export the bridge to your host machine, run:

```bash
cd
docker run --rm ros-jazzy-ros1-bridge:latest | tar xvzf -
```

You should now find a directory called `ros-jazzy-ros1-bridge` at the home directory of your host machine.

## Running the ros1-bridge

To run the ros1 bridge on your host machine, you need to source some things first

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
source ~/ros-jazzy-ros1-bridge/local_setup.bash # Must source the local_setup not setup
export ROS_MASTER_URI=http://localhost:11311 # Optional but recommended
```

Then you can run the bridge with

```bash
ros2 run ros1_bridge dynamic_bridge
```

### Test if the custom messages are available

To see all the available pairs that can be bridged use `--print-pairs`

```bash
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

Then you should see your custom message in the list with a form similar to this custom message

```bash
  # If you can't find the custom message yourself, you could use
  # ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i <name-of-message>
  ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i PseudoGridMap
  - 'pseudo_grid_map_msgs/msg/PseudoGridMap' (ROS 2) <=> 'pseudo_grid_map_msgs/PseudoGridMap' (ROS 1)
```
