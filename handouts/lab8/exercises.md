---
layout: default
title: Exercises
nav_order: 15
permalink: /lab8/exercises
has_toc: true
has_math: true
parent: Lab 8
---

# Exercises
{: .no_toc .text-delta .fs-9 }

1. TOC
{:toc}

# Submission
### Individual

Create a folder called `lab8` that includes your answers (for math-related questions LaTeX is preferred but handwritten is accepted too). Zip the folder and upload on Canvas.

**Each student needs to submit their own `lab8` folder to Canvas.**

### Team

Each group should create a new folder called `TEAM_<N>`, replacing `<N>` with your team number. For example team 2 will name their folder `TEAM_2`. Please put the source code of the entire `lab8` folder in the folder `TEAM_2`. For the non-code deliverables (plots, comments), please include a PDF in the `TEAM_2` folder. Zip the folder and submit to Canvas.

**Each team will only need to submit one `TEAM_<N>.zip` to Canvas.**

### Deadline

**Deadline:** To submit your solution, please upload the corresponding files under `Assignments > Lab 8` by **Thursday, Mar 25th, 11:59 EDT**.

# 👤 Individual

## 📨 Deliverable 1 - Bags of Visual Words [20 pts]

Please answer the following questions; the complete writeup should be between 1/2 to 1 page.

1. Explain which components in a basic BoW-based place recognition system determine the robustness of the system to illumination and 3D viewpoint changes. Why? Aim for 75-125 words, and try to give specific examples.
 - {: .hint } Hint: You may find it enlightening to read the [DBoW paper](http://doriangalvez.com/papers/GalvezTRO12.pdf) on the subject, though you should be able to answer based on this week's lectures.
2. Explain the purpose of Inverse Document Frequency (IDF) term in tf-idf. What would happen without this term and why? Aim for 75-125 words.
 - {: .hint} Hint: Consider the case where a few words are very common across almost all documents/images. Also, you can check for resources about IDF online (such as [this one](https://www.analyticsvidhya.com/blog/2020/02/quick-introduction-bag-of-words-bow-tf-idf/)) if you would like to build your intuition.
3. How does the vocabulary size in BoW-based systems affect the performance of the system, particularly in terms of computational cost and precision/recall? Aim for 75-125 words.
- {: .hint } Hint: For precision, how would adding words to the vocabulary make it easier/harder to recognize when 2 documents/images are very similar or different? Likewise for recall?


# 👥 Team

## Using Neural Networks for Object Detection

YOLO is a Convolutional Neural Network that detects objects of multiple classes.
It is based on the paper [“You Only Look Once: Unified, Real-Time Object
Detection”](https://pjreddie.com/media/files/papers/yolo.pdf). Every detected
object is marked by a bounding box. The level of confidence for each detection
is given as a probability metric (more details can be found in [YOLOv3
page](https://pjreddie.com/darknet/yolo/)). Since we are using ROS for most of
our software, we will use the repository in
[darknet_ros](https://github.com/leggedrobotics/darknet_ros).

### Installation

First, ensure that you have OpenCV 3 installed in your system by running:
```
pkg-config --modversion opencv
```
You should see output that looks like `3.4.0` or similar (`3.X.Y`) since we have used opencv in previous labs. Otherwise, the quickest
way to install all relevant dependencies is to run
```
sudo apt install ros-melodic-desktop
```
Alternatively, you can use `sudo apt-get install libopencv-*` (you need everything except libopencv-apps*).

Concerning the installation of the `darknet_ros` package, we ask you to follow the
[installation procedure](https://github.com/leggedrobotics/darknet_ros#building)
in the Readme of the repo. You can use the automatically downloaded weights that are acquired from
building the package.

Make sure the installation is correct:

```bash
catkin build darknet_ros --no-deps --verbose --catkin-make-args run_tests
```
You should see an image with two bounding boxes indicating that there is a
person (albeit incorrectly).

### Usage

Make sure you read the Readme in the repo, in particular the [Nodes
section](https://github.com/leggedrobotics/darknet_ros#nodes) which introduces
the parameters used by YOLO and the ROS topics where the output is published.

Now, download the following rosbags (Each of them can be around 1-3 GB):

1. [RGB-D TUM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download), download from the links below:
    1. [Sequence freiburg3_teddy](https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_teddy.bag).
2. [Euroc dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), download from the links below:
    2. [MH_01_easy.bag](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag) and also [dataset](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip).
    3. [V1_01_easy.bag](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag) and also [dataset](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip).

Now, change `~/vna2v_ws/src/darknet_ros/darknet_ros/config/ros.yaml` with the corresponding rgb topic in each
dataset. For example, for sequence `freiburg3_teddy`, change
`ros.yaml` as:


```yaml
subscribers:
  camera_reading:
    topic: /camera/rgb/image_color
    queue_size: 1
```

Now, open two terminals. In one, run YOLO:

```bash
roslaunch darknet_ros darknet_ros.launch
```

While in the other terminal, you should play the actual rosbag (try with freiburg3_teddy rosbag):

```bash
rosbag play PATH/TO/ROSBAG/DOWNLOADED
```

Great! Now you should be seeing YOLO detecting objects in the scene!

## 📨 Deliverable 2 - Object Localization [25 pts]

Our goal for this exercise is to localize the teddy bear that is at the center
of the scene in the _freiburg3_teddy_ dataset. To do so, we will use YOLO
detections to know where the teddy bear is. With the bounding box of the teddy
bear, we can calculate a crude approximation of the _bear’s 3D position_ by
using the center pixel of the bounding box. If we accumulate enough 2D
measurements, we can formulate a least-squares problem in GTSAM to triangulate
the 3D position of the teddy bear.

For that, we will need to perform the following steps:

1. The `freiburg3_teddy` rosbag provides ground-truth transformation of the
   camera with respect to the world. Subscribe to the `tf` topic in ROS that
   gives the transform of the camera with respect to the world.
2. In parallel, you should be able to get the results from `darknet_ros` (YOLO)
   by either making the node subscribe to the stream of images, or using the
   [Action message that the package
   offers](https://github.com/leggedrobotics/darknet_ros#actions).
3. Use YOLO to detect the bounding box around the teddy bear.
4. Extract the center pixel of the bounding box.
5. While this is a rough approximation, formulate a GTSAM problem where we are
   trying to estimate the 3D position of the center pixel in the bounding box.
   You will need to use multiple `GenericProjectionFactors` in order
   to fully constrain the 3D position of the teddy bear. Try to use the
   `GenericProjectionFactor` to estimate the 3D position of the teddy
   bear. Recall the GTSAM exercise where you performed a toy-example of Bundle
   Adjustment problem and use the same factors to build the problem. Note that
   now, the poses of the camera are given to you as ground-truth information.
   Therefore, you might want to use priors on the poses as given by the
   ground-truth poses given by the `tf` topic.
6. Solve the problem in GTSAM. You can re-use previous code from lab_7.
7. Plot the 3D position of the teddy bear in Rviz.
8. Plot also the trajectory of the camera. You can re-use previous code from lab_7

We have provided a template in `deliverable_2.cpp` and `yolo_localization.cpp`
and a corresponding header file `yolo_localization.h` to help you structure your
code. Some helper functions and hints are also provided to you in `helper_functions.hpp`.
You can choose not to use the provided `yolo_localization.cpp` template and design your
own architecture and you'll get up to 5 bonus points for doing so (refer to the "Design
Your Own Architecture" section).

### [Optional] Design Your Own Architecture [+5 pts]

Since there are many ways to solve this problem, and since we have reached a point
where you should be comfortable designing your own ROS callbacks and general code
architecture, we encourage you to implement your solution in your own style and
make reasonable assumptions and considerations. If you choose to design your own
solution architecture and not use the provided template code, we will assign
**up to 5 bonus points** depending on your design (whether your design will lead to
a better performance, the principles you apply, and the readability and efficiency
of your architecture). To get the bonus points, we ask you to write a small summary
of the assumptions, design choices and considerations that you have taken in order to
solve this problem. Aim for around 250 words, or half a page, and include it in your
deliverables.

### Performance Expectations

Your final RVIZ figure should look something like the following image. In particular,
try to show both the trajectory of the camera (green), the camera poses for which you
got a good detection of the teddy bear (red arrows), and a geometry_msgs::PointStamped for the
teddy bear's estimated location (purple sphere). Note that the size of the sphere does
not matter as long as it is visible, although you are welcome to compute the covariance
of your estimate and draw a PoseWithCovariance if you would like the size to represent the covariance.

<img data-src="{{ 'assets/images/lab8/deliverable_2.png' | absolute_url}}" class="lazyload mx-auto d-block" >

## 📨 [Optional] Deliverable 3 - Object Reconstruction [+12 pts]

Since we are given the bounding boxes of the object to detect, it would be
possible to match keypoints inside the bounding box for as many frames as
possible and triangulate a point cloud around the teddy bear. Doing this
repeatedly for different viewpoints, you could perform a sparse 3D
reconstruction of the teddy bear.

No starter code is provided for this deliverable, although it will be have much
overlap with your deliverable 2 code. You can reuse functions but please try to
delineate a boundary between your deliverables (for grading purposes).

## 📨 Deliverable 4 - Evaluating BoW Place Recognition using RANSAC [25 pts]

[DBoW2](https://github.com/dorian3d/DBoW2) is a state-of-the-art algorithm for
place recognition (loop closure). It is based in a Bag of Words technique
(details in their [paper](http://doriangalvez.com/papers/GalvezTRO12.pdf)).

Place recognition is a common module in a SLAM pipeline and it is often used as
a parallel process to the actual Visual Odometry pipeline. Whenever a place is
recognized as having been visited previously, this module computes the relative
pose between the camera that took the first image of the scene and the current
camera. Then, the SLAM system fuses this result with the visual odometry (typically
adding a new factor to the factor graph). Note that the module might fail at 
recognizing a scene, which might result in a lack of loop closures, or - what
is worst - provide wrong matches.

For this exercise, we ask you to assess the quality of the loop closures
extracted by DBoW2.

To do this, we will be using the modified version of DBoW2 from
[ORB-SLAM](https://github.com/raulmur/ORB_SLAM2), a state-of-the-art Visual
Odometry SLAM pipeline.

### Installation

Follow the Readme in our github fork of [ORB-SLAM](https://github.com/ToniRV/ORB_SLAM2) (which modifies ORB-SLAM in order to publish loop closures in ROS).

<div class="alert alert-warning"> <div class="alert-content"> <h2
  class="alert-title"> ATTENTION. </h2> <div class="alert-body"> <p>Please run the following commands somewhere <b>OUTSIDE</b> of your catkin workspace (i.e., <b>not</b> in `vna2v_ws`). It may still work for you if you run it inside, but some students have reported that ORB-SLAM will mistakenly link against other packages in your catkin workspace from previous labs and crash with a segmentation fault.</p></div> </div> </div>

For most systems that have `ros-melodic-desktop` installed, the following steps should work:
```
cd ~ # or some other folder OUTSIDE your catkin workspace
sudo apt install -y libglew-dev autoconf
cd PATH/TO/VNA2V_WS
cd src/

# Install Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git pangolin
mkdir pangolin/build
pushd pangolin/build
cmake .. && make -j4
sudo make install
popd

# Install ORB-SLAM2
git clone https://github.com/ToniRV/ORB_SLAM2.git orb-slam2
pushd orb-slam2
./build.sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$PWD/Examples/ROS
./build_ros.sh

# OPTIONAL: Run this so that you can  do `rosrun ORB_SLAM2 ...` from anywhere 
echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:'$PWD/Examples/ROS >> ~/.bashrc
```

Make sure you follow all the installation steps, including [building ORB-SLAM in ROS](https://github.com/raulmur/ORB_SLAM2#building-the-nodes-for-mono-monoar-stereo-and-rgb-d).

### Usage

Once installed, you should be able to run the following from the `orb-slam2` directory:

```bash
rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
```

In another terminal, you should run the corresponding Euroc rosbag:

```bash
rosbag play --pause ~/datasets/EuRoC/MH_01_easy/MH_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
```

Where we remap different topics to the ones that ORB-SLAM listens to. We also
start the bag in `--pause` mode, in order to make it play you should press
‘space’ in the terminal where you executed the command.

We encourage you to try as well with the other Euroc rosbag, `V1_01_easy`, but
this is not required. Alternatively, you can also see the output when using the
RGB-D data of the TUM dataset for the teddy bear.

All starter code for this deliverable is provided in `deliverable_4.cpp`. 
**Note that we only ask you to use the Euroc dataset.** 

1. We have added a subscriber to the `loop_closure` topic advertised by ORB-SLAM
   (in **Stereo** mode). Whenever you receive a message with the indices of the
   frames that are supposedly a loop closure, upload the images from the dataset
   of images by using the function `getFilenameForImageIdx`. You will need to
   change the global variable `PATH_TO_DATASET_data_csv` to point to the folder
   where the file data.csv is (This is only for Euroc dataset!).
2. Compute the quality of the loop closure by re-using the code with RANSAC to
   estimate the number of inlier keypoint matches. Rank each loop closure with
   respect to the number of inliers you found (or in other words, the quality of
   the loop closure).
3. Visualize pairs of images for the loop closures that were retrieved by
   ORB-SLAM (implicitly by DBoW2), including inlier keypoint matches. Try to look for both the good and (in case there are) the bad
   ones.
4. In your writeup, describe at least one good loop closure and one bad. For the "good", mention whether it looked "easy" or "hard".
   For the "bad", use the inlier matches to try to guess why it made a mistake.

**_<span style="text-decoration:underline;">Tips</span>_**:

- To run the RGB-D example, use as `PATH_TO_SETTINGS_FILE `the file that there is inside `Examples/ROS/ORB-SLAM2/` named `Asus.yaml`, such as:

```bash
rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Asus.yaml 
```

Make sure that when you play the corresponding rosbag, the topics are mapped correctly.

You can either remap the topics in the rosbag by typing:


```bash
rosbag play PATH/TO/THE/ROSBAG original_topic_name:=new_topic_name
```

Or by creating a launch file which uses the tag
<[remap](http://wiki.ros.org/roslaunch/XML/remap)> for the ORB-SLAM node (such
as how we have done it in previous labs).


**_<span style="text-decoration:underline;">FAQ:</span>_**

*   _ORB-SLAM tracking failure?_ It is possible that ORB-SLAM has spurious
    tracking failures and just breaks, it should not happen very often though.
*   _ORB-SLAM does not detect any loop closure?_ ORB-SLAM has built-in checks to
    ensure that any loop closure that is accepted is most probably correct.
    Therefore, unless the scene is clearly doing a loop-closure (and even in
    such occasions) it might not accept a potential loop closure.
*   _How can I increase the number of loop closures that ORB-SLAM returns?_ You
    could have a look at the LoopClosing.h and .cc files, where the actual loop
    closure is computed. You might notice that there are many hardcoded
    parameters. Our fork of ORB-SLAM should have these values small enough to
    generate more loop-closures than usual while being reasonably correct. If you
    are curious, feel free to modify the parameters therein to increase the number
    of loop closures detected. You could also re-run the rosbag or
    alternatively, play the bag in a loop (only if the trajectory ends at the
    same spot where it started, and note that the frame ids returned will not be
    aligned with the actual name of the frames).


## Summary of Team Deliverables

 1. Report the final position estimate of the teddy bear in the world reference frame and analyze your performance. [Optional] A 1/2 page summary of the implementation and assumptions made by your Object Localization code.
 2. An image showing the trajectory of the robot and the final estimated location of the teddy bear in RVIZ.
 3. [Optional] A screenshot of your sparse 3D reconstruction of the teddy bear.
 4. For one feature extractor (e.g. choose 1 of SIFT, SURF, ORB, FAST), show at least two loop closures (pairs of images) along with the number and ratio of inliers in each. Preferably one good loop closure + one bad loop closure, if possible.
   - If you only get one loop closure with the descriptor you chose, you can include it and rerun with a different descriptor to get a 2nd loop closure. 

