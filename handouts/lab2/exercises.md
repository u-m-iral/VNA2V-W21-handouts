---
layout: default
title: Exercises
nav_order: 5
permalink: /lab2/exercises
has_toc: true
has_math: true
parent: Lab 2
---

# Exercises
{: .no_toc .text-delta .fs-9 }

1. TOC
{:toc}

## Submission

To submit your solutions create a folder called `lab2` and push one or more file to your repository with your answers (it can be plain text, markdown, pdf or whatever other format is reasonably easy to read)

- for **math-related** deliverables, LaTeX is preferred but handwritten is accepted too
- for **code-based** deliverables, push the source code of the entire package

**Deadline:** the VNA2V staff will clone your repository on **Wed, Feb 3, 11:59 EST**.

### Find a team

Next lab will have a team section, you should start thinking about your team!

<div class="alert alert-info">
  <div class="alert-content">
    <h2 class="alert-title">
      Find a team.
    </h2>
    <div class="alert-body">
      <p> Please find your team members, fill out this <a href="https://docs.google.com/spreadsheets/d/1G1pSrMQH8uDWfnm-mwNYIVEa89GTFB-f1PuaXlbjJbQ/edit?usp=sharing">spreadsheet</a>.</p>
    </div>
  </div>
</div>

Please fill it by **Wed, Feb 3, 11:59 EST**.

## Setup workspace

### Create the catkin workspace

To get started, we must create a ROS workspace for the VNA2V class.
Choose your favorite working directory, we will assume you are creating one in your home directory (i.e `~/`).
In a terminal, run:

```bash
$ mkdir -p ~/vna2v_ws/src
$ cd ~/vna2v_ws/
$ catkin init
Initializing catkin workspace in `/home/vtzoumas/vna2v_ws`.
--------------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/melodic
Workspace:                   /home/vtzoumas/vna2v_ws
--------------------------------------------------------------
Build Space:       [missing] /home/vtzoumas/vna2v_ws/build
Devel Space:       [missing] /home/vtzoumas/vna2v_ws/devel
Install Space:      [unused] /home/vtzoumas/vna2v_ws/install
Log Space:         [missing] /home/vtzoumas/vna2v_ws/logs
Source Space:       [exists] /home/vtzoumas/vna2v_ws/src
DESTDIR:            [unused] None
--------------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
--------------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
--------------------------------------------------------------
Whitelisted Packages:        None
Blacklisted Packages:        None
--------------------------------------------------------------
Workspace configuration appears valid.
--------------------------------------------------------------
```

### Getting the Lab code

Go the folder where you cloned the Labs codebase and run `git pull`. This command will update the folder with the latest code.
Let's suppose we have the codebase in `~/labs`. In `~/labs/lab2` you now have the `two_drones_pkg` folder, which is a ROS package.
Copy this folder in your VNA2V workspace and build the workspace as follows:

```bash
cp -r ~/labs/lab2/two_drones_pkg ~/vna2v_ws/src
```

### Building the code

Building the code is as easy as running:

```bash
catkin build
```

Now that you built the code you see that catkin added a bunch of new folders.
In order to use our workspace, we need to make ROS aware of all the components by sourcing the corresponding environment.
This is done by running the following in **every single terminal** where you intend to use the workspace:

```bash
source devel/setup.bash
```

For the rest of the assignment, we assume that you are performing this operation whenever necessary.

### A two-drone scenario

In this part, we are going to work with 3D Rigid Transformations and with tf, a basic tool provided by ROS to keep track of multiple coordinate frames over time.

#### The static scenario and rViz

To get started, let’s go back to the VNA2V workspace and bring up the two-drones static scenario.
In this environment, we have two aerial vehicles, AV1 [blue] and AV2 [red] that are not moving, but it serves as a good starting point!
With the VNA2V workspace sourced in a terminal, run:

```bash
roslaunch two_drones_pkg two_drones.launch static:=True
```

You should see the following window, which shows the initial positions of the two AVs.

![rViz]({{ 'assets/images/lab2/rViz.png' | relative_url}}){: .mx-auto .d-block}

This window is [rViz](http://wiki.ros.org/rviz), the mighty ROS visualizer!
Just like most processes in the ROS ecosystem, rViz is a ROS node. Like all other nodes, rViz can subscribe to ROS topics. However, its specialty is converting them into graphics on your screen to display the state of your robotic system.

As a first experience with rViz, let us:

- Add the visualization of `\tf`. In the Displays panel, click Add, select the By display type Tab in the pop-up and finally select “TF” and confirm with Ok. You should see all the reference frames, with names and their axes represented in red (x), green (y) and blue (z).  
- Save the configuration file. So that we don’t have to repeat the step above every time we launch it! Hit <kbd>CTRL</kbd> + <kbd>s</kbd> or select _File_ > _Save Config_.

Other published topics can be added to the visualizer in a similar way.

## Problem formulation

We consider the scenario illustrated in the picture below, where two aerial vehicles, AV1 [blue] and AV2 [red] are following different trajectories: a **circle** and an **arc of parabola**, respectively.
In the scene, we have highlighted the following elements:

- The world frame $(x_w, y_w, z_w)$
- The AV1 body frame, centered in $O_1$ with axes $(x_1,y_1,z_1)$
- The origin of AV2, denoted with $O_2$

![Two drones reference systems]({{ '/assets/images/lab2/two-drones-refs.png' | relative_url}}){: .mx-auto .d-block .img-size-60 }

#### Positions

In the world frame, AV1 and AV2's origins are given by: 

* $o_1^w = [\text{cos}(t), \text{sin}(t), 0]^T$, and
* $o_2^w = [\text{sin}(t), 0, \text{cos}(2t)]^T$,

<!-- $$xyz = btw$$

$$
o_1^w = \begin{bmatrix}
 cos(t)\\
 sin(t)\\
 0
\end{bmatrix},\quad
o_2^w = \begin{bmatrix}
 sin(t) \\
 0 \\
 cos(2t)  
\end{bmatrix}
$$ -->

where $t$ denotes time.

#### Orientations

We will make the following simplifying assumptions:

- AV1’s reference frame is such that $y_1$ stays tangent to AV1’s trajectory for all $t$ and $z_1$ is parallel to $z_w$ for all $t$ (i.e., equivalently, roll = pitch = 0, yaw = $t$)
- AV2’s reference frame moves with pure translation and we can assume that its axes are parallel to the world axes for all times $t$

> **Notes**
> - Given the dynamics of a quadrotor, these motions are dynamically infeasible. However, for the purpose of this lab, we disregard this fact and focus on the study of rigid transformations
> - To make the math of this problem more interesting, we chose the $y_1$ axis to point in the direction of motion of the drone. However, do not forget that the standard convention is that $x_1$ should point forward!

In the sequel, we reproduce the above scenario in ROS and study the trajectory of AV2 relative to AV1’s coordinate frame.

## Basic ROS commands

### 📨 Deliverable 1 - Nodes, topics, launch files (10 pts)

With the `roslaunch` command above we have spawned a number of ROS nodes at the same time.
Using your knowledge of ROS, answer the following questions:

1. List the nodes running in the two-drone static scenario.
  - {: .hint} Hint: you can directly inspect the launch file or get some help from `rqt_graph`. You will notice that rViz is initially not shown in it but you can uncheck the _Debug_ option for a full picture, feel free to ignore all `/rosout` nodes and topics and `/rqt_gui_py_*` that may appear
2. How could you run the two-drone static scenario without using the roslaunch command? List the commands that you would have to execute, in separate terminals, to achieve the same result.
  - {: .hint} Hint: `rosrun [...]`, try things out before finalizing your answer!
3. List the topics that each node publishes / subscribes to. What nodes are responsible for publishing the av1, av2, frames? Which topic causes rViz to plot the drone meshes?
  - {: .hint} Hint: uncheck items on the left pane in rViz until the meshes disappear, then check what node is publishing the corresponding topic
4. What changes if we omit `static:=True`? Why?
  - {: .hint} Hint: check out and briefly explain the if and unless keywords in the launch file

## Let's make things move! Publishing the transforms using tf

After exploring the static scenario, it’s time to implement the motions described in the [problem formulation](#problem-formulation) section and visualize them in rViz. With the editor of your choice, open `frames_publisher_node.cpp` in the `src` folder of `two_drones_pkg`. In this file, we provide a basic structure of a ROS node.

### Some Context

Please take your time to familiarize with this code before modifying it. Although not mandatory, the pattern found in it is a very common way to implement ROS nodes:

- The node's source code is encapsulated into a class, `FramesPublisherNode` (line 6), which keeps a nodeHandle as a private member.
- In the constructor of the class (lines 14 to 19), one performs operations that need to be executed only once upon startup (e.g. typically, initializing subscribers and publishers),
- Using a Timer (lines 10 and 17), the `onPublish()` method is called periodically - at a 50Hz - and all the operations within it are performed ad libitum,
- In the body of `main()` (towards the end of the file):
  - the node is registered with `ros::init(...)`
  - an instance of the node class is created
  - a blocking call to `ros::spin()` is issued, which starts ROS's main loop.

### 📨 Deliverable 2 - Publishing transforms (30 pts)

In `frames_publisher_node.cpp`, follow the instructions in the comments and fill in the missing code. Your objective is to populate the provided `AV1World` and `AV2World` variables to match the motions described in the problem formulation.
These objects are instances of the `tf::transform` class, which is ROS jargon for a homogeneous transformation matrix.

<div class="alert alert-warning">
  <div class="alert-content">
    <h2 class="alert-title">
      Keep in mind.
    </h2>
    <div class="alert-body">
      <p> Ensure that the orientation of the AV1 frame is in accordance with the assumptions made in the problem formulation, as this is of crucial importance for the final result!</p>
    </div>
  </div>
</div>

#### How to test

Once you are ready to compile your code, run:

```bash
catkin build
```

from the workspace folder `~/vna2v_ws`.

To try out your code, launch the two-drone scenario in non-static mode, i.e. run:

```bash
roslaunch two_drones_pkg two_drones.launch
```

#### What to expect

You should finally see the drones moving! Check that the trajectories reflect those illustrated in the figure in the problem formulation.

![Two drones example]( {{ 'assets/images/lab2/two_drones_rviz.gif' | absolute_url }}){: .mx-auto .d-block }

### Changing the rViz fixed reference frame. 

As mentioned, we are interested in the motion of AV2 relative to AV1’s reference frame. In the Displays panel (left, by default), under the Global Options section, rViz offers the possibility to change the Fixed Frame to any frame published in tf. Try it out yourself and change “world” into “av1” by typing in the corresponding field.
From this perspective, AV1 appears static, the world frame spins around its $z$ axis and AV2 seems to be following a closed-curve trajectory.

### 📨 Deliverable 3 - Looking up a transform (30 pts)

In `plots_publisher_node.cpp`, follow the instructions in the comments and fill in the missing code. Your objective is to populate the provided object, transform, with the relative transform between two given frames with names `ref_frame` and `dest_frame`.

Compile your code and try it out as previously explained.

#### What to expect

You should eventually see three trajectories, namely:

- AV1’s trajectory [blue, solid] in the world frame (circle on the x-y plane)
- AV2’s trajectory [red, solid] in the world frame (parabola on the z-x plane)
- The trajectory of AV2 in AV1’s frame [red, dashed]. You should now have a strong hunch that this curve is a ellipse on a “slanted” plane!

**Note**: if the results you are observing do not seem to make sense, try swapping `ref_frame` and `dest_frame` when interrogating `\tf`.

## Let’s do some math

So far, we have used ROS and tf to get a visual understanding of the motion of AV2 relative to AV1’s body frame.
In this section, you are asked to use your knowledge about homogeneous transformations and study the relative trajectory explicitly.

The visualization we have built should provide you with great guidance while working out the following questions. Since this exercise is designed for you to familiarize with the math of 3D transformations, we require that you explicitly write down all the homogeneous transformation matrices used in the process and precisely outline the logic and algebraic steps taken.

### 📨 Deliverable 4 - Mathematical derivations (25 pts)

1. In the [problem formulation](#problem-formulation), we mentioned that AV2’s trajectory is an arc of parabola in the $x$-$z$ plane of the world frame. Can you prove this statement?
  - {: .hint} Hint: $cos(2t)$ can be written as...
2. Compute $o_2^1(t)$, i.e., the position of AV2 relative to AV1’s body frame as a function of $t$.
  - {: .hint} Hint: write down the homogeneous transformations and compose them accordingly...
3. Show that $o_2^1(t)$ describes a planar curve and find the equation of its plane $\Pi$.
  - {: .hint} Hint: find a linear relation between $z_2^1$ and $y_2^1$
4. Rewrite the above trajectory explicitly using a 2D frame of reference $(x_p, y_p)$ on the  plane found before. Try to ensure that the curve is centered at the origin of this 2D frame and that $x_p$, $y_p$ are axes of symmetry for the curve.
  - {: .hint} Hints: 
    1. {: .hint} &emsp;i) center the new 2D frame in $p^1= (-1,-1/2,0)$, these coordinates are in AV1’s frame
    2. {: .hint} &emsp;ii) start with a 3D reference frame centered in p with axes $(x_p, y_p, z_p)$, compute $o_2^p(t)$
    3. {: .hint} &emsp;iii) make sure that the $z$ component vanishes after the change of coordinates
5. Using the expression of $o_2^p(t)$, prove that the trajectory of AV2 relative to AV1 is an ellipse and compute the lengths of its semi-axes.
  - {: .hint} Hint: what is the general form of the equation of an axis-aligned ellipse centered in the origin?

### 📨 Deliverable 5 - More properties of quaternions (5 pts)
In the lecture notes, we have defined two linear maps $\Omega_1: \mathbb{R}^4 \rightarrow \mathbb{R}^{4\times 4}$, and $\Omega_2: \mathbb{R}^4 \rightarrow \mathbb{R}^{4\times 4}$, such that for any $q \in \mathbb{R}^4$, we have:
\\[ 
\Omega_1(q) = \begin{bmatrix}
q_4 & -q_3 & q_2 & q_1 \\\
q_3 & q_4 & -q_1 & q_2 \\\
-q_2 & q_1 & q_4 & q_3 \\\
-q_1 & -q_2 & -q_3 & q_4
\end{bmatrix}, 
\Omega_2(q) = \begin{bmatrix}
q_4 & q_3 & -q_2 & q_1 \\\
-q_3 & q_4 & q_1 & q_2 \\\
q_2 & -q_1 & q_4 & q_3 \\\
-q_1 & -q_2 & -q_3 & q_4
\end{bmatrix}.
\\]
The product between any two unit quaternions can then be explicitly computed as:
\\[ 
q_a \otimes q_b = \Omega_1(q_a) q_b = \Omega_2(q_b)q_a.
\\]
In fact, the two linear maps $\Omega_1$ and $\Omega_2$ have more interesting properties, and you are asked to prove the following equalities:

1. For any unit quaternion $q$, both $\Omega_1(q)$ and $\Omega_2(q)$ are *orthogonal* matrices, *i.e.*,
\\[
\Omega_1(q)^T\Omega_1(q) = \Omega_1(q)\Omega_1(q)^T = I_4,
\\]
\\[
\Omega_2(q)^T\Omega_2(q) = \Omega_2(q)\Omega_2(q)^T = I_4.
\\]
Intuitively, what is the reason that both $\Omega_1(q)$ and $\Omega_2(q)$ must be orthogonal? 
  - {: .hint} Hint: what does $q_a \otimes q_b$ still being a unit quaternion imply?

2. For any unit quaternion $q$, both $\Omega_1(q)$ and $\Omega_2(q)$ convert $q$ to be the unit quaternion that corresponds to the 3D identity rotation, *i.e.*,
\\[
\Omega_1(q)^Tq = \Omega_2(q)^Tq = [0,0,0,1]^T.
\\]

3. For any two vectors $x,y \in \mathbb{R}^4$, show the two linear operators commute, *i.e.*,
\\[
\Omega_1(x) \Omega_2(y) = \Omega_2(y) \Omega_1(x),
\\]
\\[
\Omega_1(x) \Omega_2(y)^T = \Omega_2(y)^T \Omega_1(x).
\\]



### 📨 [Optional] Deliverable 6 - Intrinsic vs Extrinsic rotations (20 pts)

Consider the following sequence of rotations:

- $R_0$: 90&deg; around $x$
- $R_1$: 180&deg; around $y$
- $R_2$ -30&deg; around $x$

_A)_ Extrinsic

The sequence of rotations is applied with respect to a fixed frame of reference   (the world frame), as follows:

![Extrinsic rotations]( {{ 'assets/images/lab2/guitar-01.png' | absolute_url }})

_Note_: the body axes are unlabeled, but represented in red ($x_b$), green ($y_b$), blue ($z_b$)

_B)_ Intrinsic

The sequence of rotations is applied in _reverse order_ with respect to a frame of reference attached to the object (the body frame), as follows:

![Intrinsic rotations]( {{ 'assets/images/lab2/guitar-02.png' | absolute_url }})

**Note that the final orientation of the object is the same in both cases!**

This property is quite general: it holds regardless of the specific axes and angles of the rotations and for any number of rotations in the sequence.

Could you prove this formally?
