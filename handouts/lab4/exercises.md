---
layout: default_latex
title: Exercises
nav_order: 7
permalink: /lab4/exercises
has_toc: true
has_math: true
parent: Lab 4
---

# Exercises
{: .no_toc .text-delta .fs-9 }

1. TOC
{:toc}

# Submission
### Individual

Create a folder called `lab4` that includes your answers (for math-related questions LaTeX is preferred but handwritten is accepted too). Zip the folder and upload on Canvas.

**Each student needs to submit their own `lab4` folder to Canvas.**

### Team

Each group should create a new folder called `TEAM_<N>`, replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Please put the source code of the entire `planner_pkg` and `trajectory_generation_pkg` package in the folder `TEAM_2`. Zip the folder and submit to Canvas.

**Each team will only need to submit one `TEAM_<N>.zip` to Canvas.**

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 4` by **Thursday, Feb 18 11:59 EST**.
# 👤 Individual

## 📨 Deliverable 1 - Single-segment trajectory optimization (20 pts)

Consider the following minimum velocity ($r=1$) single-segment trajectory optimization problem:

<!-- \begin{eqnarray}
\min_{P(t)} \quad  \int_0^1 (P^{(1)}(t))^2 dt, \label{eq:minvel} \\\
s.t.   \quad P(0) = 0, \label{eq:initpos} \\\
       \quad P(1) = 1, \label{eq:finalpos} 
\end{eqnarray} -->


\begin{equation}
\min_{P(t)} \quad  \int_0^1 (P^{(1)}(t))^2 dt, \label{eq:minvel}
\end{equation}

\begin{equation}
s.t.   \quad P(0) = 0, \label{eq:initpos}
\end{equation}

\begin{equation}
\quad P(1) = 1, \label{eq:finalpos} 
\end{equation}


with $P(t) \in \mathbb{R}[t]$, i.e., $P(t)$ is a polynomial function in $t$ with real coefficients: 

\begin{equation}
P(t) = p_N t^N + p_{N-1} t^{N-1} + \dots + p_1 t + p_0.
\end{equation}

Note that because of constraint (\ref{eq:initpos}), we have $P(0)=p_0=0$, and we can parametrize $P(t)$ without a scalar part $p_0$. 

_1._ Suppose we restrict $P(t) = p_1 t$ to be a polynomial of degree 1, what is the optimal solution of problem (\ref{eq:minvel})? What is the value of the cost function at the optimal solution?

<!-- **Solution**: From $P(1) = 1$, we have $p_1 = 1$. Therefore, $P(t) = t$, $P^{(1)}(t) = 1$, and the value of the cost function is $\int_0^1 1^2 \cdot dt = 1$.     -->

_2._ Suppose now we allow $P(t)$ to have degree 2, i.e., $P(t) = p_2t^2 + p_1 t$.

_(a)_ Write $\int_0^1 (P^{(1)}(t))^2 dt$, the cost function of problem (\ref{eq:minvel}), as $\boldsymbol{p}^T \boldsymbol{Q} \boldsymbol{p}$, where $\boldsymbol{p} = [p_1,p_2]^T$ and $\boldsymbol{Q} \in \mathcal{S}^2$ is a symmetric $2\times 2$ matrix.

<!-- - {: .hint} Hint: $P^{(1)}(t) = p_1 + p_2 \cdot (2t) = \boldsymbol{p}^T[1,2t]^T$, therefore:

\begin{equation}
\int_0^1 (P^{(1)}(t))^2 dt = \int_0^1 \boldsymbol{p}^T 
\left[\begin{array}{c} 1 \\\
2t \end{array}\right]
\left[\begin{array}{cc} 1 & 2t \end{array}\right] \boldsymbol{p} dt 
= \boldsymbol{p}^T \left( \int_0^1 \left[\begin{array}{cc}1 & 2t \\\ 2t & 4t^2 \end{array}\right] dt \right)\boldsymbol{p}
\end{equation} -->

<!-- **Solution**: 
\begin{equation}
\boldsymbol{Q} = \left[ \begin{array}{cc} 1 & 1 \\\ 1 & \frac{4}{3} \end{array} \right].
\end{equation} -->

_(b)_ Write $P(1) = 1$, constraint (\ref{eq:finalpos}), as $\boldsymbol{A}\boldsymbol{p} = \boldsymbol{b}$, where $\boldsymbol{A} \in \mathbb{R}^{1 \times 2}$ and $\boldsymbol{b} \in \mathbb{R}$.

<!-- **Solution**: 
\begin{equation}
\boldsymbol{A} = \left[ \begin{array}{cc} 1 & 1 \end{array} \right], \boldsymbol{b} = 1.
\end{equation} -->

_(c)_ Solve the Quadratic Program (QP):
\begin{equation}
\min_{\boldsymbol{p}} \boldsymbol{p}^T \boldsymbol{Q} \boldsymbol{p} \quad s.t. \quad \boldsymbol{A} \boldsymbol{p} = \boldsymbol{b}. \label{eq:QPtrajOpt}
\end{equation}
You can solve it by hand, or you can solve it using numerical QP solvers (e.g., you can easily use the [`quadprog`](https://www.mathworks.com/help/optim/ug/quadprog.html) function in Matlab). What is the optimal solution you get for $P(t)$, and what is the value of the cost function at the optimal solution? Are you able to get a lower cost by allowing $P(t)$ to have degree 2?

<!-- **Solution**: $P^\star(t) = t$, same result as in 1. -->

_3._ Now suppose we allow $P(t) = p_3t^3 + p_2 t^2 + p_1 t$:

_(a)_ Let $\boldsymbol{p} = [p_1,p_2,p_3]^T$, write down $\boldsymbol{Q} \in \mathcal{S}^3, \boldsymbol{A} \in \mathbb{R}^{1\times 3}, \boldsymbol{b} \in \mathbb{R}$ for the QP (\ref{eq:QPtrajOpt}).

**Solution**: $P^{(1)}(t) = p_1 + p_2 \cdot (2t) + p_3 \cdot (3t^2)$. Therefore, we have:
\begin{equation}
\boldsymbol{Q} = \int_0^1 \left[\begin{array}{c} 1 \\\ 2t \\\ 3t^2 \end{array}\right]
\left[\begin{array}{ccc} 1 & 2t & 3t^2 \end{array}\right] dt =
\int_0^1 \left[\begin{array}{ccc} 1 & 2t & 3t^2 \\\ 2t & 4t^2 & 6t^3 \\\ 3t^2 & 6t^3 & 9t^4 \end{array}\right] dt = 
\left[\begin{array}{ccc} 1 & 1 & 1 \\\ 1 & \frac{4}{3} & \frac{3}{2} \\\ 1 & \frac{3}{2} & \frac{9}{5} \end{array}\right].
\end{equation}
In addition, we have:
\begin{equation}
\boldsymbol{A} = [1,1,1], \boldsymbol{b} = 1.
\end{equation}

_(b)_ Solve the QP, what optimal solution do you get? Do this example agree with the result we learned from Euler-Lagrange equation in class?

<!-- **Solution**: $P^\star(t) = t$, agrees with Euler-Lagrange equation. -->

_4._ Now suppose we are interested in adding one more constraint to problem (\ref{eq:minvel}):

\begin{eqnarray}
\min_{P(t)} \quad  \int_0^1 (P^{(1)}(t))^2 dt, \label{eq:minveladd} \\\
s.t.   \quad P(0) = 0, \\\
       \quad P(1) = 1, \\\
       \quad P^{(1)}(1) = -2.
\end{eqnarray}
Using the QP method above, find the optimal solution and optimal cost of problem (\ref{eq:minveladd}) in the case of:


_(a)_ $P(t) = p_2t^2 + p_1  t$, and 

_(b)_ $P(t) = p_3t^3 + p_2 t^2 + p_1t$.

<!-- **Solution**: 

(a): $P^\star(t) = -3t^2 + 4t$, optimal cost 3.

(b): $P^\star(t) = -3.75t^3 + 4.5t^2 + 0.25t$, optimal cost 2.125. -->


## 📨 Deliverable 2 - Multi-segment trajectory optimization (15 pts)

_1._ Assume our goal is to compute the minimum snap trajectory ($r= 4$) over $k$ segments. How many and which type of constraints (at the intermediate points and at the start and end of the trajectory) do we need in order to solve this problem? Specify the number of waypoint constraints, free derivative constraints and fixed derivative constraints.

- {: .hint} Hint: According to Euler-Lagrange method, what is the degree of the polynomial of each segment?
- {: .hint} Hint: How many unknown parameters do we need to solve?
- {: .hint} Hint: How many constraints does each waypoint/free derivative/fixed derivative constraint provide?
- {: .hint} Hint: See figure for $k=3$ as described in the lecture notes.

![Minimum snap trajectory]( {{ 'assets/images/lab4/trajOpt.png' | absolute_url }}){: .mx-auto .d-block .img-size-90}

_2._ Can you extend the previous question to the case in which the cost functional minimizes the $r$-th derivative and we have $k$ segments?

# 👥 Team

## 📨 Deliverable 3 - Drone Racing (65 pts)

For this lab we will be racing our simulated quadcopters in a drone racing course we prepared in our Unity simulator! 
- Implement all the missing parts in the code (labeled as Part 0, Part 1.1, Part 1.2, and Part 1.3, see below)
- A video showing the quadrotor completing the race course. Please upload the video onto either Google drive or Dropbox, generate a publicly viewable link, and put the link in a text file called `rviz_drone_race.txt` in your repo.
- A rosbag of your complete and fastest run. Upload the rosbag onto either Google drive or Dropbox, generate a publicly viewable link, and put the link also in `rviz_drone_race.txt`. To record the rosbag, 
```bash
rosbag record /current_state /desired_state
```

### Getting the codebase

First, let us install some prerequisites. In a terminal, run:

Next, let's set up our workspace: 

```bash
# change the folder name according to your setup
cd ~/labs
git pull
```

In `~/labs/lab4` we now have the `planner_pkg`, `trajectory_generation_pkg`, and `dependencies` folders, the first two are ROS packages that you 
will be modifying for this lab, and `dependencies` should contain all the dependencies you need for this lab.
```bash
cp -r ~/labs/lab4/. ~/vna2v_ws/src
cd ~/vna2v_ws
```

In your `src` folder, you should see the folders: `catkin_simple`, `controller_pkg`, `tesse-ros-bridge`, `planner_pkg`, `trajectory_generation_pkg`, and `dependencies`. Now, compile the new code.

```bash
catkin build
source devel/setup.bash
```

After doing so, please proceed [here](https://drive.google.com/file/d/1-1oUnTLP7wDmiaWCku0KNuRABe6pe1K5/view?usp=sharing) to download the new  binary executable for the simulator we are going to use for lab 4. 
Unzip the file, put the folder in `~/vna2v-builds`, and run the following commands:

```bash
cd ~/vna2v-builds/vna2v-2021-lab4/
chmod +x vna2v-2021-lab4.x86_64
```
Try launch the simulator, the simulator should look like this
![Unity example]({{'assets/images/lab4/unity_race_course.png' | absolute_url}}){: .mx-auto .d-block .img-size-90}

Before you start coding, keep in mind that the system we are aiming for is as follows. 
Pieces are coming together and we are getting closer and closer to the full system that we have seen so often in class!
![System diagram]({{'assets/images/lab4/lab4_diagram.jpg' | absolute_url}}){: .mx-auto .d-block .img-size-120}

### Journey of a thousand miles starts with a single step

As a warm up exercise, let's just fly and hover at the first gate! Follow the instructions for Part 0 inside `planner_pkg/src/simple_traj_planner.cpp`

Now fire up the simulator and test. In one terminal window, run
```bash
cd ~/vna2v-builds/vna2v-2021-lab4/
./vna2v-2021-lab4.x86_64
```
In another terminal, launch `tesse_ros_bridge` to connect ROS to the simulator, 
```bash
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch
```
Finally launch the test to hover at the first gate
```bash
roslaunch planner_pkg static_point_test.launch
```
- {: .hint} Hint: you can press `r` to respawn your quadcopter
- {: .hint} Hint: review the handout from lab 3 if you have trouble running the simulator. 

### Waypoint publishing 

We have already written this node for you in `planner_pkg/src/traj_vertices_publisher.cpp`. 
What we are doing here is reading the position and orientation of the gates in the racing course 
and publishing them as waypoints for trajectory optimization. 
The topic to note here is `/desired_traj_vertices`, which should contain a `geometry_msgs::PoseArray` type
storing the position and orientation of the gates on the racing course. 

### Trajectory generation and following 

Follow the instructions for Part 1 in `trajectory_generation_pkg/src/trajectory_generation_node.cpp` to get your quadcopter ready for drone racing! 
This node will subscribe to the waypoints published and use them for trajectory optimization 
using the [mav_trajectory_generation library](https://github.com/ethz-asl/mav_trajectory_generation), and then based on the trajectory, publish 
the desired state at time t to your controller.

- {: .hint} Hint: use `vertex.addConstraint(POSITION, position)` where position is of type `Eigen::Vector3d` to enforce a waypoint position.
- {: .hint} Hint: use `vertex.addConstraint(ORIENTATION, yaw)` where yaw is a double to enforce a waypoint yaw.
- {: .hint} Hint: remember angle wraps around 2$\pi$. Be careful!
- {: .hint} Hint: for the ending waypoint for position use `end_vertex.makeStartOrEnd` as seen with the starting waypoint instead of `vertex.addConstraint` as you would do for the other waypoints.

### Ready, go! 

Now we are ready to race! Fire up the virtual race course. 
```bash
cd ~/vna2v-builds/vna2v-2021-lab4/
./vna2v-2021-lab4.x86_64
```
In another terminal, launch `tesse_ros_bridge` to connect ROS to the simulator, 
```bash
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch
```
Launch the trajectory follower,
```bash
roslaunch trajectory_generation traj_following.launch
```
And finally, launch the waypoint publisher 
```bash
roslaunch planner_pkg traj_gen.launch
```

If everything is working well, your drone should be gracefully going through each gate like this 

<img data-src="{{ 'assets/images/lab4/drone_aero_race.gif' | absolute_url}}" class="lazyload mx-auto d-block" >

**Note**: If the quadcopter flips over after launching the trajectory follower, press `r` in the Unity window to respwan, the quadcopter should just go back to the starting position.

### [Optional] Faster, faster (Extra credit: 15 pts)

How can you make the drone go faster? We will award extra credit to the fastest 3 teams!