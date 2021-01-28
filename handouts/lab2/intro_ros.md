---
layout: default
title: Introduction to ROS
nav_order: 3
permalink: /lab2/ros101
has_toc: true
parent: Lab 2
---


# ROS Intro 
{: .no_toc .text-delta .fs-9 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## ROS file system structure

### General structure

Similar to an operating system, ROS files are also organized in a particular fashion.
The following graph shows how ROS files and folder are organized on the disk:

![ROS FS Structure](https://static.packt-cdn.com/products/9781783987580/graphics/7580OS_02_01.jpg){: .mx-auto .d-block }

The ROS **packages** are the most basic unit of the ROS software.
They contain the ROS runtime process (**nodes**), libraries, configuration files, and so on, which are organized together as a single unit. Packages are the atomic build item and release item in the ROS software.

Inside a package we can find the **package manifest** file, which contains information about the package, author, license, dependencies, compilation flags, and so on. The `package.xml` file inside the ROS package is the manifest file of that package.

The ROS **messages** are a type of information that is sent from one ROS process to the other. They are regular text files with `.msg` extension that define the fields of the messages.

The ROS **service** is a kind of request/reply interaction between processes. The reply and request data types can be defined inside the `srv` folder inside the package.

For example, the package we will develop in this lab will be like

```text
.
└── two_drones_pkg
    ├── CMakeLists.txt
    ├── README.md
    ├── config
    │   └── default.rviz
    ├── launch
    │   └── two_drones.launch
    ├── mesh
    │   └── quadrotor.dae
    ├── package.xml
    └── src
        ├── frames_publisher_node.cpp
        └── plots_publisher_node.cpp
```

### The workspace

In general terms, the workspace is a folder which contains packages, those packages contain our source files and the environment or workspace provides us with a way to compile those packages.
It is useful when we want to compile various packages at the same time and it is a good way of centralizing all of our developments.

## ROS Master, nodes and topics

One of the primary purposes of ROS is to facilitate communication between the ROS modules called nodes. Those nodes can be executed on a single machine or across several machines, obtaining a distributed system. The advantage of this structure is that each node can control one aspect of a system. For example you might have several nodes each be responsible of parsing row data from sensors and one node to process them.

### ROS Master

Communication between nodes is established by the ROS Master. The ROS Master provides naming and registration services to the nodes in the ROS system. It is its job to track **publishers** and **subscribers** to the **topics**.

ROS master works much like a DNS server. Whenever any node starts in the ROS system, it will start looking for ROS master and register the name of the node with ROS master. Therefore, ROS master has information about all the nodes that are currently running on the ROS system. When information about any node changes, it will generate a call back and update with the latest information.

![ROS communication]({{ 'assets/images/lab2/ros_communication.png' | absolute_url }})

ROS Master distributes the information about the topics to the nodes. Before a node can publish to a topic, it sends the details of the topic, such as its name and data type, to ROS master. ROS master will check whether any other nodes are subscribed to the same topic. If any nodes are subscribed to the same topic, ROS master will share the node details of the publisher to the subscriber node.

After receiving the node details, these two nodes will interconnect using the TCPROS protocol, which is based on TCP/IP sockets, and ROS master will relinquish its role in controlling them.
<!-- It is also possible to use  UDPROS (based on UDP/IP) for a low-latency, lossy transport layer, which is best used for tasks like teleoperation and local-host communications. -->

To start ROS master, open a terminal and run

```bash
roscore
```

<!-- ROS master can also be used in **distributed network**, in which different physical computers participate in a ROS computational network. -->
<!-- Thereupon, nodes in the network seek a connection to `ROS_MASTER_URI` (an environmental variable that should be set properly) that will run the ROS master.  -->
Any ROS system must have **only one master**, even in a distributed system, and it should run on a computer that is reachable by all other computers to ensure that remote ROS nodes can access the master.

### ROS nodes

Basically, nodes are regular processes but with the capability to register with the ROS Master node and communicate with other nodes in the system. The ROS design idea is that each node is an independent module that interacts with other nodes using the ROS communication capability.

The nodes can be created in various ways. From a terminal window a node can be created directly by typing a command after the command prompt, as shown in the examples to follow. Alternatively, nodes can be created as part of a program written in Python or C++. In this lab, we will use both the ROS commands in a terminal window and C++ programs to create nodes.

As example let's run the `turtlesim` node, in a new terminal run

```bash
rosrun turtlesim turtlesim_node
```

Yous should see something like

![Turtlesim]({{ 'assets/images/lab2/turtlesim.png' | absolute_url }}){: .mx-auto .d-block .img-size-60 }

Now, you can ask the ROS master about the running nodes with

```bash
$ rosnode list
/rosout
/turtlesim
```

Now, let's run another node

```bash
rosrun turtlesim turtle_teleop_key
```

Now the list of node changed

```bash
$ rosnode list
/rosout
/teleop_turtle
/turtlesim
```

### ROS topics

Topics are the means used by nodes to transmit data, it represents the channel where messages are sent and it has a message type attached to it (you cannot send different types of messages in a topic).
In ROS, data production and consumption are decoupled, this means that a node can publish message (producer) or subscribe to a topic (consumer).
<!-- Data production and consumption are decoupled, this means that a topic can have various subscribers (that consume data) and can also have various publishers (that produce data), but you should be careful when publishing the same topic with different nodes as it can create conflicts. -->

Let's use `rqt_graph` which shows the nodes and topics currently running.

```bash
rosrun rqt_graph rqt_graph
```

If you select _Nodes/Topics (all)_ from the top left and deselect _Debug_ you will see something similar to

![rqt_graph]({{ 'assets/images/lab2/rqt_graph.png' | absolute_url }})

In the graph the ellipses are nodes and the squares are topics. From the picture it's easy to see that `teleop_turtle` is publishing to `/turtle1/cmd_vel` topic. The node `/turtlesim` is subscribed to the topic and uses the incoming messages to move the turtle.

You can also print the messages to the terminal.
Try to run `rostopic echo /turtle1/cmd_vel` and move the turtle, you should get something like

```bash
$ rostopic echo /turtle1/cmd_vel
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -2.0
---
```

These are very useful tools to debug your nodes.

## Anatomy of a ROS node

The simplest C++ ROS node has a structure similar to the following

```cpp
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(50);

  while (ros::ok()) {
    // ... do some useful things ...
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
```

Let's analyze it line by line: the first line

```cpp
#include "ros/ros.h"
```

adds the [header](http://docs.ros.org/melodic/api/roscpp/html/ros_8h.html) containing all the basic ROS functionality.
At the beginning of the main of the program

```cpp
ros::init(argc, argv, "example_node");
```

`ros::init` initialize the node, it is responsible for collecting ROS specific information from arguments passed at the command line and set the node name (remember: names must be **unique** across the ROS system). But it **does not** contact the master.
To contact the master and register the node we need to call

```cpp
ros::NodeHandle n;
```

When the first `ros::NodeHandle` is created it will call `ros::start()`, and when the last ros::NodeHandle is destroyed (e.g. goes out of scope), it will call `ros::shutdown()`. This is the most common way of handling the lifetime of a ROS node.

Usually we want to run our node at a given frequency, to set the node frequency we use

```cpp
ros::Rate loop_rate(50);
```

which is setting the desired rate at 50 Hz.
Then we have the **main loop** of the node.
Since we want to run this node until the ROS we need to the check the various states of shutdown.
The most common way to do it is to call `ros::ok()`.
Once `ros::ok()` returns false, the node has finished shutting down.
That's why we have

```cpp
while (ros::ok()) {
  // ...
}
```

Inside the loop we can make interesting things happen.
In our example we simply run

```cpp
ros::spinOnce();
loop_rate.sleep();
```

The function `ros::spinOnce()` will call all the callbacks waiting to be called at that point in time while.
If you remember we set the node frequency to 50Hz, the code we are running will probably take less than 20ms.
The function `loop_rate.sleep()` will pause the node the remaining time.

## Launch files

Launch files are the preferred way to run ROS nodes. The launch files provide a convenient interface to execute multiple nodes and a master (if is not already running), as well as other initialization requirements such as parameters.

Usually the launch files are located in the `launch` folder of the package and have `.launch` extension. If the package provide one you can use `roslaunch` to use it.

```bash
roslaunch <package_name> <launch_file>
```

**Note:** Pushing <kbd>CTRL</kbd>+<kbd>c</kbd> in a terminal with a launch file running will close all nodes that were started with that launch files.

An example of launch file is

```xml
<launch>
  <node name="map_server" pkg="map_server" type="mapserver" />
  <node name="stageros" pkg="stage" type="stageros" args="$(find navigation_stage)/stage config/worlds/willow-pr2-5cm.world" >
    <param name="base_watchdog_timeout" value="0.2"/>
  </node>
  <include file="$(find navigation_stage)/move_base_config/amcl_node.xml"/>
</launch>
```

This example will run three nodes (plus the master if not already running), each `<node>...</node>` is equivalent to a `rosrun` call, for example 

```xml
<node name="stageros" pkg="stage" type="stageros" args="$(find navigation_stage)/stage_config/worlds/willow-pr2-5cm.world" />
```

is equivalent to

```bash
rosrun stage stageros <path-to-navigation-stage-package>/stage_config/worlds/willow-pr2-5cm.world
```

This tool gives the possibility to add complex and dynamic runtime behavior such as `$(find path)`, `unless` and `if` or include other launch files.

## Transforms (tf package)

The ROS tf library has been developed to provide a standard method to keep track of coordinate frames and transform data within the entire system so that users can be confident about the consistency of their data in a particular coordinate frame without requiring knowledge about all the other coordinate frames in the system and their associations.

tf is distributed across nodes (across machines too, eventually) and there are two types of tf nodes

- Listener: that listen to `/tf` and cache all data that it collected (up to cache limit)
- Broadcaster: that publish transforms between coordinate frames on `/tf`

In tf, transforms and coordinate frames are represented as a graph with the transforms as edges and the coordinate frames as nodes. The advantage of this representation is that the relative pose between two nodes is simply the product of the edges connecting the two nodes.
A tree structure has also the benefit of allowing for dynamic changes easily. tf indeed takes care of ambiguity of transforms not allowing loops in the transforms graph.

<div class="alert alert-info">
  <div class="alert-content">
    <h2 class="alert-title">
      Where to learn TF.
    </h2>
    <div class="alert-body">
      <p>The best resource to learn TF out there is the official <a href="http://wiki.ros.org/tf2/Tutorials">ROS tf tutorials</a>.</p>
      <p>Take your time to familiarize with the Listener/Broadcaster code, you'll need for the exercises.</p>
      <br/>
      <p>Of course do not forget the <a href="http://docs.ros.org/melodic/api/tf/html/c++/namespacetf.html">official documentation.</a></p>
    </div>
  </div>
</div>

### Quick overview of tf tools

Let's see the tools we have to explore the tf tree.

ROS provides a simple demo we are going to use, from a terminal run

```bash
roslaunch turtle_tf turtle_tf_demo.launch
```

Once the turtlesim demo is started, we will drive the center turtle around in turtlesim using the keyboard arrow keys.
We can observe that one turtle will continuously follow the turtle we are driving. In this demo application, the ROS TF library is used to create three coordinate frames: a world frame, a turtle1 frame, and a turtle2 frame, and to create a TF broadcaster to publish the coordinate frames of the first turtle and a TF listener to compute the difference between the first and follower turtle frames, as well as drive the second turtle to follow the first.

#### Using `view_frames`

The `view_frame`s tool creates a diagram of the frames being broadcast by TF over ROS:

```bash
rosrun tf view_frames
evince frames.pdf
```

![view_frames example]({{ 'assets/images/lab2/turtle_tf_view.png' | absolute_url}})

Here, you can see that three frames are broadcast by TF—the `world`, `turtle1`, and `turtle2`, where the `world` frame is the parent of the `turtle1` and `turtle2` frames.

#### Using `rqt_tf_tree`

The rqt_tf_tree tool enables the real-time visualization of the tree of frames being broadcast over ROS

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

![rqt_tf_tree example]({{ 'assets/images/lab2/turtle_rqt_tf_tree.png' | absolute_url}})

#### Using `tf_echo`

The tf_echo tool reports the transformation between any two frames broadcast over ROS

```bash
rosrun tf tf_echo [reference_frame] [target_frame]
```

For example

```bash
$ rosrun tf tf_echo turtle1 turtle2
At time 1568050753.324
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.344, 0.939]
            in RPY (radian) [0.000, -0.000, 0.703]
            in RPY (degree) [0.000, -0.000, 40.275]
```

#### Usign RViz

_RViz_ is a graphical 3D visualization tool that is useful for viewing the association between TF frames within the ROS system:

![RViz example]({{ 'assets/images/lab2/turtle_rviz.png' | absolute_url}})

You can run _RViz_ just running

```bash
rviz
```

One it is started, you need to press the button _Add_ to add the _TF_ in the visualization and set the correct _Fixed Frame_  (i.e. `world`).

## Additional resources

Of course there are many details we haven't discussed here, below you can find resources we find interesting

- [Official ROS tutorial](http://wiki.ros.org/ROS/Tutorials)
- [A Gentle Introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/)
- [Free videos from Robot Ignite Academy](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q/playlists)
- [Programming Robots with ROS](https://www.amazon.com/Programming-Robots-ROS-Practical-Introduction/dp/1449323898)
