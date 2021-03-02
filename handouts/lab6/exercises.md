---
layout: default
title: Exercises
nav_order: 12
permalink: /lab6/exercises
has_toc: true
has_math: true
parent: Lab 6
---

# Exercises
{: .no_toc .text-delta .fs-9 }

1. TOC
{:toc}

# Submission

In this Lab, there are 5 deliverables throughout the handout. Deliverables 1 and
2 will require pen and paper and are considered individual tasks, while
Deliverable 3-5 are a team task which requires coding in the lab6 directory that
we will provide.

### Individual

Create a folder called `lab6` that includes your answers (for math-related questions LaTeX is preferred but handwritten is accepted too). Zip the folder and upload on Canvas.

**Each student needs to submit their own `lab6` folder to Canvas.**

### Team

Each group should create a new folder called `TEAM_<N>`, replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Please put the source code of the entire `lab6` folder in the folder `TEAM_2`. For the non-code deliverables (plots, comments), please include a PDF in the `TEAM_2` folder. Zip the folder and submit to Canvas.

**Each team will only need to submit one `TEAM_<N>.zip` to Canvas.**


### Deadline

**Deadline:** To submit your solution, please upload the corresponding files under `Assignment > Lab 6` by **Tuesday, Mar 9, 11:59 EST**.

# 👤 Individual

## 📨 Deliverable 1 - Nister's 5-point Algorithm [20 pts]

### Read the paper and answer the questions below

Read the following paper.

[1] Nistér, David. “An efficient solution to the five-point relative pose
problem.” 2003 IEEE Computer Society Conference on Computer Vision and Pattern
Recognition, 2003. Vol. 2. 2003. [link here](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.86.8769&rep=rep1&type=pdf).

Questions:
1. Outline the main computational steps required to get the relative pose
   estimate (up to scale) in Nister’s 5-point algorithm.
2. Does the 5-point algorithm exhibit any degeneracy? (degeneracy = special
   arrangements of the 3D points or the camera poses under which the algorithm
   fails)
3. When used within RANSAC, what is the expected number of iterations the
   5-point algorithm requires to find an outlier-free set?
   - {: .hint } Hint: take same assumptions of the lecture notes

## 📨 Deliverable 2 - Designing a Minimal Solver [15 pts]

**Can you do better than Nister?** Nister’s method is a minimal solver since it
uses 5 point correspondences to compute the 5 degrees of freedom that define the
relative pose (up to scale) between the two cameras (recall: each point induces
a scalar equation). In the presence of external information (e.g., data from other
sensors), we may be able use less point correspondences to compute the relative
pose.

Consider a drone flying in an unknown environment, and equipped with a camera and an Inertial Measurement Unit (IMU). We want to use the feature correspondences extracted in the images captured at two consecutive time instants $t_1$ and $t_2$ to estimate the  elative pose (up to scale) between the pose at time $t_1$ and the pose at time $t_2$.
Besides the camera, we can use the IMU (and in particular the gyroscopes in the IMU) to estimate the relative rotation between the pose of the camera at time $t_1$ and $t_2$.

You are required to solve the following problems:
1. Assume the relative camera rotation between time and is known from the IMU.
   Design a minimal solver that computes the remaining degrees of freedom of the
   relative pose.
   - {: .hint } Hint: we only want to compute the pose up to scale
2. **OPTIONAL (5 bonus pts)**: Describe the pseudo-code of a RANSAC algorithm using the minimal solver
   developed in point a) to compute the relative pose in presence of outliers
   (wrong correspondences).

# 👥 Team

In this section, we will estimate the motion of a (simulated) flying drone in real time
and compare the performances of different algorithms.

For the algorithms,
we will be using the implementations provided in the 
[OpenGV](https://laurentkneip.github.io/opengv/page_how_to_use.html) library
(note: Open**G**V). 

For the datasets, we will use pre-recorded `rosbag` files of our simulated drone flying in an indoor environment.

Additionally, for motion estimation:
   - We will only focus on two-view (vs multi-camera) pose estimation. In
     OpenGV, we refer to two-view problems as “Central” (vs “Non-Central”)
     relative pose problems.
   - We will focus only on the calibrated case, where the intrinsics matrix K is given, and
     we assume that the images are rectified (distortion removed) using the
     parameters that you estimated previously.

## Getting started: code base and datasets
- **Prerequisites**: Lab 6 will use the feature matching algorithms developed in Lab 5 (in particular, we use SIFT matching), so
make sure you have a working version of Lab 5 already in the VNA2V workspace.

- **Prepare the code base**: Use `git pull` to update the gitlab repo used to distribute lab codes, and you should see a new folder named `lab6`. Copy the entire `lab6` folder to the `src` folder of your vna2v workspace (e.g., `~/vna2v_ws/src`). Now we are ready to install OpenGV by doing:
```shell
cd ~/vna2v_ws/src (path to src of vna2v workspace)
wstool init
wstool merge lab6/install/lab_6.rosinstall -y
wstool update -j8
```
The above scripts will download OpenGV into your workspace (you will see a folder `opengv` under `src`).
Now run:
```shell
catkin build lab_6
```
to build the `lab_6` package (which should build OpenGV first and then build `lab_6` itself).

- **Download the datasets**: We will use the following dataset for this lab:
  1. `office.bag` and you can download it [here](https://drive.google.com/file/d/15eJ-pNjwWB728gMHoAhaCGwiQtWwDlAT/view?usp=sharing).

After downloading the dataset, we suggest you to put them in the `~/data/vna2v` folder.

The rosbag files include the following topics of the drone:
  - Ground-truth pose estimate of the drone's body frame: `/tesse/odom`
  - RGB image from the left-front camera of the drone: `/tesse/left_cam/rgb/image_raw`
  - Depth image: `/tesse/depth_cam/mono/image_raw`

You can play these 
datasets by running (after using `roscore` to start ROS master first):
```shell
rosbag play ~/data/vna2v/office.bag
```
while in parallel open RVIZ by:
```shell
rviz -d ~/vna2v_ws/src/lab6/rviz/office.rviz
```
You should see on the left the RGB Image and the Depth image.

## Let's perform motion estimation!

We will use two methods to estimate the motion of the drone:
- Motion estimation from 2D-2D correspondences (Deliverable 4)
- Motion estimation from 3D-3D correspondences (Deliverable 5)

In Deliverable 4, we will perform motion estimation **only** using 2D RGB images taken
from the drone's camera, while in Deliverable 5, we will additionally use the depth 
measurements to get the sense of 3D.

**NOTE:** 
- All your main implementations of the motion estimation algorithms should be in
  the `pose_estimation.cpp` file. In the file, we have also provided many comments to help
  your implementation, so please go through the comments in details.
- For this lab, we
  provide a number of useful utility functions in `lab6_utils.h`. You do not need to
  use these functions to complete the assignment, but they might help save you
  some time and frustration.

## 📨 Deliverable 3 - Initial Setup [5 pts]

Before we go to motion estimation, an important task is to calibrate the camera of the drone,
i.e., to obtain the camera intrinsics and distortion coefficients. Normally you would 
need to calibrate the camera yourself offline to obtain the parameters. 

However, in this lab the camera that the drone is equipped with has been
calibrated already, and calibration information is provided to you!
(If you are curious about how to calibrate a camera, feel free to check this [ROS package](http://wiki.ros.org/camera_calibration))

As part of the starter code, we provide a function `calibrateKeypoints` to calibrate and undistort the keypoints.
Make sure you use this function to calibrate the keypoints before passing them to RANSAC.
   

## 📨 Deliverable 4 - 2D-2D Correspondences [45 pts]

Given a set of keypoint correspondences in a pair of images (2D - 2D image
correspondences), as computed in the previous lab 5, we can use 2-view
(geometric verification) algorithms to estimate the relative pose (up to scale)
from one viewpoint to another.

To do so, we will be using three different algorithms and comparing their
performance.

We will first start with the 5-point algorithm of Nister. Then we will test the 8-point method we have seen in class. Finally, we will test the 2-point method you developed in Deliverable 2. For all techniques, we use the feature matching code we developed in Lab 5. In particular, we use SIFT for feature matching in the remaining of this problem set.

We provide you with a skeleton code in `lab6` folder where we have set-up ROS
callbacks to receive the necessary information.

We ask you to complete the code inside the following functions:

### 1. `cameraCallback`: this is the main function for this lab. 

Inside, you will have to use three different algorithms to estimate the relative pose
from frame to frame:
* OpenGV's the 5-point algorithm with RANSAC [(see OpenGV
 API)](https://laurentkneip.github.io/opengv/classopengv_1_1sac__problems_1_1relative__pose_1_1CentralRelativePoseSacProblem.html)
* OpenGV's [8-point algorithm by Longuet-Higgins with
  RANSAC](https://laurentkneip.github.io/opengv/classopengv_1_1sac__problems_1_1relative__pose_1_1CentralRelativePoseSacProblem.html)
* OpenGV's [2-point algorithm with RANSAC](https://laurentkneip.github.io/opengv/classopengv_1_1sac__problems_1_1relative__pose_1_1TranslationOnlySacProblem.html). This algorithm requires you to provide the
relative rotation between pairs of frames. This is usually done by integrating
the IMU’s gyroscope measurements. Nevertheless, for this lab, we will ask you to
compute the relative rotation using the ground-truth pose of the drone between
both frames.

For each part, follow the comments written in the source code for further
details.

**We strongly recommend you to take a look at how to use OpenGV functions [here](https://laurentkneip.github.io/opengv/page_how_to_use.html).**

**OPTIONAL (5 bonus pts)**: if you are curious about how important is to reject outliers via RANSAC, try to use the 5-point method [without RANSAC (see OpenGV
 API)](https://laurentkneip.github.io/opengv/namespaceopengv_1_1relative__pose.html#af269f7393720263895fb9b746e4cec4a),
 and add the results to the performance evaluation below.

###  2. `evaluateRPE`: evaluating the relative pose estimates

After implementing the relative pose estimation methods, you are required to evaluate their accuracy and plot their errors over time. Since you also have the ground-truth pose of the drone, it is
possible to compute the Relative Pose Error (RPE) between your estimated relative pose
from frame to frame and the actual ground-truth movement. Follow the equations
below and compute the translation and rotation relative errors on the rosbag we provided.

<!-- Also, take into account that the
ground-truth pose is the one for the body frame of the drone (typically located
at the center of the IMU), but you are actually estimating the pose of the
camera. Usually, this transformation is calculated beforehand, using a package
such as Kalibr. -->

***The relative pose error is a metric for investigating the local consistency of a trajectory***

RPE compares the relative poses along the estimated and the reference trajectory. Given the ground truth pose $T^W_{ref,t}$ at time $t$ (with respect to the world frame $W$), we can compute the ground truth relative pose between time $t-1$ and $t$  as:

\\[ T_{ref,t}^{ref,t-1} = \left(T^W_{ref,t-1}\right)^{-1}  T^W_{ref,t} \in \SE{3} \\]

Similarly, the 2-view geometry algorithms we test in this lab will provide an estimate for the relative pose between the frame at time $t-1$ and $t$:

\\[ T^{est,t-1}_{est,t} \in \SE{3} \\]

Therefore, we can compute the mismatch between the ground truth and the estimated relative poses using one of the distances we discussed during lecture. 

***When using 2D-2D correspondences, the translation is only computed up to scale (and is conventionally returned as a vector with unit norm). so we recommend scaling the corresponding ground truth translation to have unit norm before computing the errors we describe below.***

**Relative translation error:** This is simply the Euclidean distance between the ground truth and the estimated relative translation:

\\[ RPE_{t-1,t}^{tran} = \left\Vert \mathrm{trans}\left(T_{ref,t}^{ref,t-1}\right) - \mathrm{trans}\left(T^{est,t-1}_{est,t}\right) \right\Vert_2  \\]

where $\mathrm{trans}(\cdot)$ denotes the translation part of a pose. 

**Relative rotation error:** This is the chordal distance between the ground truth and the estimated relative rotation:

\\[ RPE_{i,j}^{rot} = \left\Vert \mathrm{rot}\left(T_{ref,t}^{ref,t-1}\right) - \mathrm{rot}\left(T_{est,t}^{est,t-1}\right) \right\Vert_{F}  \\]

where $\mathrm{rot}(\cdot)$ denotes the rotation part of a pose. 

You will need to implement these error metrics, compute them for **consecutive frames in the rosbag**, and plot them as discussed above.

As a deliverable, **provide 2 plots showing the rotation error and the translation error
over time** for each of the tested techniques (2 plots with 3 lines for the
algorithms using RANSAC). You can write the data to a file and do the plotting with Python if you prefer (upload as well the python script if necessary).

### 3. Publish your relative pose estimate 

In order to visualize your relative pose estimate between time $t-1$ and $t$, we postmultiply your estimated relative pose between time $t-1$ and $t$ by the ground truth pose at time $t-1$. This will give you a pose estimate at time $t$ that you can visualize in Rviz.  **To do so, we use the ground-truth pose of the previous frame (obtained from ROS messages), "plus" the relative pose between current frame and previous frame (obtained from your algorithms, and then scale the translation using ground-truth),
to compute the estimated (absolute) pose of the current frame, and then publish it.**

To run your code, use:

```bash
roslaunch lab_6 video_tracking.launch
```
but be sure to modify the dataset path and parameters to run the correct method! For example,
the `pose_estimator` parameter determines which algorithm to be used for the motion estimation.

**Note that we are cheating in this visualization since we use the ground truth from the previous time stamp.
In practice, we cannot concatenate multiple estimates from 2-view geometry since they are up to scale (so for visualization, we use groundtruth to recover the scale).**

In the next deliverable we will see that 3D-3D correspondences allow us to reconstruct the correct scale for the translation.**


<div class="alert alert-warning"> <div class="alert-content"> <h2
  class="alert-title"> ATTENTION. </h2> <div class="alert-body"> <p><b>NOTE:</b> You need to have <b>xterm</b> installed on your Linux system in order to properly start the launch file. You can install it using the following command in a terminal window.
<code>
sudo apt install xterm
</code>
</p></div> </div> </div>

## 📨 Deliverable 5 - 3D-3D Correspondences [20 pts]

The rosbag we provide you also contains depth values registered with the RGB
camera, this means that each pixel location in the RGB camera has an associated
depth value in the Depth image.

In this part, we have provided code to scale to bearing vectors to 3D point clouds,
and what you need to do is to 
use Arun’s algorithm (with RANSAC) to compute the drone’s relative
pose from frame to frame.

### 1. `cameraCallback`: Implement Arun’s algorithm

Implement [Arun's algorithm](http://laurentkneip.github.io/opengv/namespaceopengv_1_1point__cloud.html#a047c3c5a395a740e7f3f2b8573289211)
in this function. Use the evaluateRPE function you used previously to **plot the
rotation error and the translation error over time** as well. Mind that, in this
case, there is no scale ambiguity, therefore we cannot really compare the translation error of this
approach against the previous ones. Implement Arun's algorithm _with_ RANSAC using OpenGV.

To run your code, use:

```bash
roslaunch lab_6 video_tracking.launch
```
with the `pose_estimator` parameter set to `3` so that Arun's method is used.

**Note that while we can now reconstruct the trajectory by concatenating the relative poses, such a trajectory estimate will quickly diverge due to error accumulation. In future lectures, we will study Visual-Odometry and Loop closure detection as two ways to mitigate the error accumulation.**

## Performance Expectations
What levels of rotation and translation errors should one expect from using these different algorithms? To set the correct expection, we think the following
errors are satisfactory:
- Using 5-point or 8-pt with RANSAC, for most of the frames, you can get rotation error below 1 degree and translation error below 0.5 (note that the translation error is between 0 and 2 since
both ground-truth translation and estimated translation have unit norm), with 5-pt algorithm slightly outperforming 8-pt algorithm.
- Using 2-point with RANSAC, for most of the frames, you can get the translation error below 0.1 (note that the translation error is between 0 and 2).
- Using 3-point with RANSAC (3D-3D), for most of the frames, you can get rotation error below 0.1 degree, and translation error below 0.1 (if you normalize the translations), and even smaller if you don't normalize the translations since the frame rate is very high.

## Summary of Team Deliverables
For the given dataset, we require you to run **all algorithms** on it and compare their performances.
Therefore, as a summary for Team Deliverables:
1. Plots of translation and rotation error for each of the methods (5pt, 8pt, 2pt, Arun 3 pt) using the given rosbag (using RANSAC is required, while without RANSAC is optional).
2. **OPTIONAL (10 bonus pts)**: repeat the tests using a rosbag you collect during drone racing (your Lab 4 solution). The rosbag must contain stereo RGB images, depth information, and odometry, which are not published by tesse_ros_bridge by default. So, in order to collect a suitable rosbag, you must follow these steps:
    1. Download the [updated simulator](https://drive.google.com/file/d/1loAS87Rw9-fVB-DK2q-E2ve4SCNz0uWQ/view?usp=sharing) and run it with these command line arguments: `-screen-width 720 -screen-height 480`
    2. Launch the tesse ros bridge with these arguments: `roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_stereo_rgb:=true publish_depth:=true publish_odom:=true`
    3. Collect the rosbag of your trajectory as before (e.g. by launching the appropriate nodes and running `rosbag record -a` in another window)
    4. If you experience any errors while following the above steps, please check Piazza and make a new post if your question is not already answered.
    5. {: .hint } **Important:** If you run into an error that you are unable to fix yourself after checking Piazza, you can still receive full bonus points through submitting a **complete and detailed** bug report on Piazza which clearly explains the commands you used and any error messages or unexpected behaviour that occurred, as well as describing attempts you've made to understand/fix the bug. Please copy and paste this report into your team writeup to ensure you get the bonus.

