---
layout: default
title: Exercises
nav_order: 10
permalink: /lab5/exercises
has_toc: true
has_math: true
parent: Lab 5
---

# Exercises
{: .no_toc .text-delta .fs-9 }

1. TOC
{:toc}

# Submission

To submit your solutions create a folder called `lab5` and push one or more file
to your repository with your answers.

### Individual

Please push the deliverables into your personal repository, for math-related questions only typeset PDF files are allowed (e.g., using Latex, Word, Markdown).

### Team

Please push the source code for the entire package to the folder `lab5` of the
team repository. For the tables and discussion questions, please push a PDF to
the `lab5` folder of your team repository.

### Deadline

**Deadline:** the VNA2V staff will clone your repository on **October 7th** at 11:59 PM EDT.

# 👤 Individual

## 📨 Deliverable 1 - Practice with Perspective Projection [10 pts]

Consider a sphere with radius $r$ centered at $[0\ 0\ d]$ with respect to the
camera coordinate frame (centered at the optical center and with axis oriented
as discussed in class). Assume $d > r + 1$ and assume that the camera has
principal point at $(0,0)$, focal length equal to 1, pixel sizes $s_x = s_y = 1$
and zero skew $s_\theta = 0$ (see lecture notes for notation)
the following exercises:

1. **Derive** the equation describing the projection of the sphere onto the image plane.
- {: .hint} Hint: Think about what shape you expect the projection on the image plane to be, and then derive a characterisitic equation for that shape
in the image plane coordinates $u,v$ along with $r$ and $d$.
2. **Discuss** what the projection becomes when the center of the sphere is at an arbitrary location, not necessarily along the optical axis. What is the shape of the projection?

## 📨 Deliverable 2 - Vanishing Points [10 pts]

Consider two 3D lines that are parallel to each other. As we have seen in the
lectures, lines that are parallel in 3D may project to intersecting lines on the
image plane. The pixel at which two 3D parallel lines intersect in the image
plane is called a vanishing point. Assume a camera with principal point at
(0,0), focal length equal to 1, pixel sizes $s_x = s_y = 1$ and zero skew
$s_\theta = 0$ (see lecture notes for notation). 
Complete the following exercises:

1. **Derive** the generic expression of the vanishing point corresponding to two parallel 3D lines.
2. **Find (and prove mathematically)** a condition under which 3D parallel lines remain parallel in the image plane.

- {: .hint} Hint: For both 1. and 2. you may use two different approaches:
- {: .hint} **Algebraic approach**: a 3D line can be written as a set of points $p(\lambda) = p_0 +
\lambda u$ where $p_0 \in \mathbb{R}^3$ is a point on the line, $u \in
\mathbb{R}^3$ is a unit vector along the direction of the line, and $\lambda \in
\mathbb{R}$.
- {: .hint} **Geometric approach**: the projection of a 3D line can be understood as the intersection beetween two planes.

# 👥 Team

# Update the lab codebase

Assuming you have already cloned our `Labs` repository, you simply need to pull the latest changes!

```bash
cd ~/Labs
git pull origin master
```

Copy the contents of the `lab5` directory into the `src` folder of your catkin workspace:

```bash
cp -r ~/Labs/lab5 ~/vna2v_ws/src
cd ~/vna2v_ws/src
```

Make sure to keep the `lab3` and `lab4` folders in your catkin workspace, too! Some of their dependencies (specifically `glog_catkin` and `catkin_simple`) are also required by `lab5`. Finally, build the code:

```bash
cd ~/vna2v_ws
catkin build -j$(nproc) lab_5
```

**NOTE:** Building the `opencv3_catkin` package may take some time. As a point
of comparison, running with `catkin build -j8` on my machine, it took 5.5
minutes. If you run into an error with 'gflags' when trying to build `opencv3_catkin`, run `catkin clean` to clean your workspace, then build `opencv3_catkin` _before_ building the rest of the code, i.e.:
```bash
catkin build -j$(nproc) opencv3_catkin
catkin build -j$(nproc) lab_5
```

<div class="alert alert-warning"> <div class="alert-content"> <h2
  class="alert-title"> ATTENTION. </h2> <div class="alert-body"> <p><b>NOTE:</b> There was a bug in <code>lab5/feature_tracking/launch/two_frames_tracking.launch</code> where "/ >" should have been replaced with " />". Please pull the updated code or make the change yourself.</p></div> </div> </div>

# Feature Tracking and Matching

Feature tracking between images is a fundamental module in many computer vision
applications, as it allows us both to triangulate points in the scene and, at
the same time, estimate the position and orientation of the camera from frame to
frame (you will learn how to do so in subsequent lectures).

## Descriptor-based Feature Matching

### Feature Detection (SIFT)

Feature detection consists in extracting keypoints (pixel locations) in an
image. In order to track features reliably from frame to frame, we need to
specify what is a good feature to detect.

The ideal features in an image are the ones that have the following properties:

- Repeatability: the same feature can be found in several images despite geometric (i.e. rotations, scale changes, etc) and photometric transformations (i.e. changes in brightness).
- Saliency: each feature has a distinctive description. Otherwise, matching between features is difficult if all features look the same.
- Compactness and efficiency: fewer features than image pixels.
- Locality: a feature occupies a relatively small area of the image, making it
  robust to clutter and occlusion.

As you have seen in the lecture, one very successful feature detector is SIFT
(Scale Invariant Feature Transform), which is not only invariant to image
rotations, but also to scale changes of the image.

Let us now use SIFT to detect keypoints in an image to see what is the actual output.

We want you to get familiar with state-of-the-art feature detectors, so we will avoid having you re-implement the detectors themselves and focus instead on their comparison. Refer to the tutorials in OpenCV for the different algorithms we will be using for more details. For example, SIFT is detailed [here](https://docs.opencv.org/3.4.3/da/df5/tutorial_py_sift_intro.html) and its OpenCV API is defined [here](https://docs.opencv.org/3.4.11/d7/d60/classcv_1_1SIFT.html). You will need to search through the OpenCV documentation for details on the other descriptors and methods mentioned in this handout.

**NOTE**: Check your OpenCV version.

## 📨 Deliverable 3 - Feature Descriptors (SIFT) [20 pts]

We provide you with skeleton code for the base class FeatureTracker that
provides an abstraction layer for all feature trackers. Furthermore, we give you
two empty structures for the SIFT and SURF methods that derive from the class
FeatureTracker.

Inside the `lab5` folder, we provide you with two images ‘box.png’ and
‘box_in_scene.png’ (inside the `images` folder).

We will first ask you to extract keypoints from both images using SIFT. For
that, we refer you to the skeleton code in the `src` folder named
`track_features.cpp`. Follow the instructions written in the comments; specifically,
you will need to complete:
- The stub <code>SiftFeatureTracker::detectKeypoints()</code> and <code>SiftFeatureTracker::describeKeypoints()</code> in <code>lab5/feature_tracking/src/sift_feature_tracker.cpp</code>
- The first part of <code>FeatureTracker::trackFeatures()</code> in <code>lab5/feature_tracking/src/feature_tracker.cpp</code>

Once you have implemented SIFT, you can test it by running:

```bash
roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT # note you can change the descriptor later
```

Your code should be able to plot a figure like the one below (keypoints you
detected should not necessarily coincide with the ones in the figure):

![Detected keypoints]( {{ 'assets/images/lab5/keypoints.png' | absolute_url }}){: .mx-auto .d-block }

Now that we have detected keypoints in each image, we need to find a way to
uniquely identify these to subsequently match features from frame to frame.
Feature descriptors in the literature are multiple, and while we will not review
all of them, they all rely on a similar principle: using the pixel intensities
around the detected keypoint to describe the feature.

Descriptors are multidimensional and are typically represented by an array.

Follow the skeleton code in `src` to compute the descriptors for all the
extracted keypoints.

- {: .hint} Hint: In OpenCV, SIFT and the other descriptors we will look at are children
of the "Feature2D" class. We provide you with a SIFT <code>detector</code> object,
so look at the <a href="https://docs.opencv.org/3.4/d0/d13/classcv_1_1Feature2D.html">
Feature2D "Public Member Functions"</a> documentation to determine which command you need to detect keypoints and get their descriptors. 

## 📨 Deliverable 4 - Descriptor-based Feature Matching [10 pts]

With the pairs of keypoint detections and their respective descriptors, we are
ready to start matching keypoints between two images. It is possible to match
keypoints just by using a brute force approach. Nevertheless, since descriptors
can have high-dimensionality, it is advised to use faster techniques.

In this exercise, we will ask you to use the FLANN (Fast Approximate Nearest
Neighbor Search Library), which provides instead a fast implementation for
finding the nearest neighbor. This will be useful for the rest of the problem
set, when we will use the code in video sequences.

1. What is the dimension of the SIFT descriptors you computed?
2. Compute and plot the matches that you found from the _box.png_ image to the _box\_in\_scene.png_.
3. You might notice that naively matching features results in a significant
amount of false positives (outliers). There are different techniques to minimize
this issue. The one proposed by the authors of SIFT was to calculate the best
two matches for a given descriptor and calculate the ratio between their
distances: `Match1.distance < 0.8 * Match2.distance` This ensures that we do not
consider descriptors that have been ambiguously matched to multiple descriptors
in the target image. Here we used the threshold value that the SIFT authors
proposed (0.8).
4. Compute and plot the matches that you found from the _box.png_ image to the
_box\_in\_scene.png_ after applying the filter that we just described. You should
notice a significant reduction of outliers.

Specifically, you will need to complete:
- The stub <code>SiftFeatureTracker::matchDescriptors()</code> in <code>feature_tracking/src/sift_feature_tracker.cpp</code>
- The second part of <code>FeatureTracker::trackFeatures()</code> in <code>feature_tracking/src/feature_tracker.cpp</code>

- {: .hint} Hint: Note that the <code>matches</code> object is a pointer type, so to use it
you will usually have to type <code>*matches</code>. Once you've used the FLANN matcher
to get the matches, if you want to iterate over all of them you could use a loop like:<br>
<code>for (auto& match : *matches) {<br>
  // check match.size(), match[0].distance, match[1].distance, etc.<br>
}</code> <br>
Likewise, <code>good_matches</code> is a pointer so to add to it you will need <code>good_matches->push_back(...)</code>.

## 📨 Deliverable 5 - Keypoint Matching Quality [10 pts]

Excellent! Now, that we have the matches between keypoints in both images, we
can apply many cool algorithms that you will see in subsequent lectures.

For now, let us just use a blackbox function which, given the keypoints
correspondences from image to image, is capable of deciding whether some matches
are considered outliers.

1. Using the function we gave you, compute and plot the inlier and outlier
   matches, such as in the following figure:
![Keypoint matches]( {{ 'assets/images/lab5/kp_matches.png' | absolute_url }}){: .mx-auto .d-block }
- {: .hint} Hint: Note that <code>FeatureTracker::inlierMaskComputation</code> computes an inlier mask of type <code>std::vector&lt;uchar&gt;</code>,
but for <code>cv::drawMatches</code> you will need a <code>std::vector&lt;char&gt;</code>. You can go from one to the other by using:<br>
<code>std::vector&lt;char&gt; char_inlier_mask{inlier_mask.begin(), inlier_mask.end()};</code>
- {: .hint} Hint: You will need to call the <code>cv::drawMatches</code> function twice, first to plot the everything in red (using <code>cv::Scalar(0,0,255)</code> as the color), and then again to plot inliers in green (using the inlier mask and <code>cv::Scalar(0,255,0)</code>). The second time, you will need to use the <code>DrawMatchesFlags::DRAW_OVER_OUTIMG</code> flag to draw on top of the first output. To combine flags for the <code>cv::drawMatches</code> function, use the bitwise-or operator: <code>DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | DrawMatchesFlags::DRAW_OVER_OUTIMG</code>

2. Now, we can calculate useful statistics to compare feature tracking
   algorithms. First, let's compute statistics for SIFT. Submit a table similar
   to the following for SIFT (you might not get the same results, but they
   should be fairly similar):

<table>
  <tr>
    <th style="border-bottom: 0px; text-align: center;"><b>Statistics</b></th>
    <th style="text-align: center;" colspan="2"><b>Approach</b></th>
  </tr>
  <tr>
    <td></td>
    <td style="text-align: center;">SIFT</td>
    <td style="text-align: center;">...</td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Keypoints in Img 1</td>
    <td style="text-align: center;">603</td>
    <td></td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Keypoints in Img 2</td>
    <td style="text-align: center;">969</td>
    <td></td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Matches</td>
    <td style="text-align: center;">603</td>
    <td></td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Good Matches</td>
    <td style="text-align: center;">93</td>
    <td></td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Inliers</td>
    <td style="text-align: center;">78</td>
    <td></td>
  </tr>
  <tr>
    <td style="text-align: center;">Inlier Ratio</td>
    <td style="text-align: center;">83.9%</td>
    <td></td>
  </tr>
</table>

## 📨 Deliverable 6 - Comparing Feature Matching Algorithms on Real Data [20 pts]

The most common algorithms for feature matching use different detection, description, and matching techniques. We'll now try different techniques and see how they compare against one another:

### 6.a. Pair of frames
Fill the previous table with results for other feature tracking algorithms. We ask that you complete the table using the following additional algorithms:

1. [SURF](https://docs.opencv.org/3.4.2/df/dd2/tutorial_py_surf_intro.html) (a faster version of SIFT)
2. [ORB](https://docs.opencv.org/3.4.2/d1/d89/tutorial_py_orb.html) (we will use it for SLAM later!)
3. [FAST](https://docs.opencv.org/3.4.2/df/d0c/tutorial_py_fast.html)
   (detector) +
   [BRIEF](https://docs.opencv.org/3.4.2/dc/d7d/tutorial_py_brief.html)
   (descriptor)

- {: .hint} Hint: The SURF functions can be implemented <i>exactly</i> the same as SIFT, while ORB and FAST can also be implemented exactly the same way except that the FLANN matcher must be initialized with a new parameter like this: <code>FlannBasedMatcher matcher(new flann::LshIndexParams(20, 10, 2));</code>. This is not necessarily the best solution for ORB and FAST however, so we encourage you to look into other methods (e.g. the <a href="https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html#goal">Brute-Force matcher</a> instead of the FLANN matcher) if you have time.

We have provided method stubs in the corresponding header files for you to implement in the CPP files. Please refer to the OpenCV documentation, tutorials, and C++ API when filling them in. You are encouraged to modify the default parameters used by the features extractors and matchers. A complete answer to this deliverable should include a brief discussion of what you tried and what worked the best.

<div class="alert alert-warning"> <div class="alert-content"> <h2
  class="alert-title"> ATTENTION. </h2> <div class="alert-body"> <p><b>NOTE:</b> If you have the issue "Unknown interpolation method in function 'resize'" for your <b>ORB feature tracker</b>, explicitly set the number of levels in the ORB detector to 1 <a href="https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html#adc371099dc902a9674bd98936e79739c">see OpenCV API here</a>)</p></div> </div> </div>

By now, you should have four algorithms, with their pros and cons, capable of
tracking features between frames.

- {: .hint} Hint: It is normal for some descriptors to perform worse than others, especially on this pair of images -- in fact, some may do very poorly, so don't worry if you observe this.

### 6.b. Real Datasets

Let us now use an actual video sequence to track features from frame to frame and push these algorithms to their limit!

We have provided you with a set of datasets in rosbag format [here](https://github.mit.edu/VNA2V-W21/lab-data/tree/master/lab5). Please download the following datasets, which are the two "easiest" ones:

- `30fps_424x240_2018-10-01-18-35-06.bag`
- `vna2v-lab5-smooth-trajectory.bag`

Testing other datasets may help you to identify the relative strengths and weaknesses of the descriptors, but this is not required.

We also provide you with a roslaunch file that executes two ROS nodes:

```bash
roslaunch lab_5 video_tracking.launch path_to_dataset:=/home/$USER/Downloads/<NAME_OF_DOWNLOADED_FILE>.bag
```

- One node plays the rosbag for the dataset
- The other node subscribes to the image stream and is meant to compute the statistics to fill the table below

You will need to first specify in the launch file the path to your downloaded dataset. Once you're done, you should get something like this:

 - {: .hint} Hint: You may need to change your plotting code in <code>FeatureTracker::trackFeatures</code> to call <code>cv::waitKey(10)</code> instead of <code>cv::waitKey(0)</code> after imshow in order to get the video to play continuously instead of frame-by-frame on every keypress.

<img data-src="{{ 'assets/images/lab5/sift.gif' | absolute_url}}" class="lazyload mx-auto d-block" >

Finally, we ask you to summarize your results in one table for each dataset and asnwer some questions:

- Compute the **average** (over the images in each dataset) of the statistics on the table below for the different datasets and approaches. You are free to use whatever parameters you find result in the largest number of inliers.
<table>
  <tr>
    <th style="border-bottom: 0px; text-align: center;"><b>Statistics</b></th>
    <th style="text-align: center;" colspan="4"><b>Approach for Dataset X</b></th>
  </tr>
  <tr>
    <td></td>
    <td style="text-align: center;">SIFT</td>
    <td style="text-align: center;">SURF</td>
    <td style="text-align: center;">ORB</td>
    <td style="text-align: center;">FAST+BRIEF</td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Keypoints in Img 1</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Keypoints in Img 2</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Matches</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Good Matches</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
  </tr>
  <tr>
    <td style="text-align: center;"># of Inliers</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
    <td style="text-align: center;">...</td>
  </tr>
  <tr>
    <td style="text-align: center;">Inlier Ratio</td>
    <td style="text-align: center;">...%</td>
    <td style="text-align: center;">...%</td>
    <td style="text-align: center;">...%</td>
    <td style="text-align: center;">...%</td>
  </tr>
</table>
- What conclusions can you draw about the capabilities of the different approaches? Please make reference to what you have tested and observed.

- {: .hint} Hint: <b>We don't expect long answers, there aren't specific answers we are looking for, and you don't need to answer every suggested question below!</b> We are just looking for a few sentences that point out the main differences you noticed and that are supported by your table/plots/observations.<br>Some example questions to consider:
 - {: .hint} Which descriptors result in more/fewer keypoints?
 - {: .hint} How do they the descriptors differ in ratios of good matches and inliers?
 - {: .hint} Are some feature extractors or matchers [faster](https://stackoverflow.com/a/22387757) than others?
 - {: .hint} What applications are they each best suited for? (e.g. when does speed vs quality matter)

## 📨 Deliverable 7 - Feature Tracking: Lucas Kanade Tracker [20 pts]

So far we have worked with descriptor-based matching approaches. As you have
seen, these approaches match features by simply comparing their descriptors.
Alternatively, feature tracking methods use the fact that, when recording a
video sequence, a feature will not move much from frame to frame. We will now
use the most well-known differential feature tracker, also known as Lucas-Kanade
(LK) Tracker.

1. Using [OpenCV's documentation](https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_lucas_kanade.html) and the [C++ API for the LK tracker](https://docs.opencv.org/3.3.1/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323), track features for the video sequences we provided you by using the [Harris corner detector](https://docs.opencv.org/3.4.2/dc/d0d/tutorial_py_features_harris.html) (like [here](https://www.dropbox.com/s/zodssejrdl9vqdb/jpl_cave.mp4?dl%3D0)). Show the feature tracks at a given frame extracted when using the Harris corners, such as this:
![LK Tracker Output]( {{ 'assets/images/lab5/lk_tracker.png' | absolute_url }}){: .mx-auto .d-block }
2. Add an extra entry to the table used in **Deliverable 6.** using the Harris + LK tracker that you implemented.
3. What assumption about the features does the LK tracker rely on?
4. Comment on the different results you observe between the table in this section and the one you computed in the other sections.

- {: .hint} Hint: You will need to convert the image to grayscale with <code>cv::cvtColor</code> and will want to look into the documentation for <code>cv::goodFeaturesToTrack</code> and <code>cv::calcOpticalFlowPyrLK</code>. The rest of the trackFeatures() function should be mostly familiar feature matching and inlier mask computation similar to the previous sections. Also note that the `status` vector from calcOpticalFlowPyrLK indicates the matches.

- {: .hint} Hint: For the show() method, you will just need to create a copy of the input frame and then make a loop that calls <code>cv::line</code> and <code>cv::circle</code> with correct arguments before calling <code>imshow</code>.

## 📨 [Optional] Deliverable 8 - Optical Flow [+20 pts]

LK tracker estimates the optical flow for sparse points in the image. Alternatively, dense approaches try to estimate the optical flow for the whole image. Try to calculate your [own optical flow](https://www.dropbox.com/s/37u2b5xax6puf5j/own_flow.mp4?dl=0), or the flow of a video of your choice, using [Farneback's algorithm](https://docs.opencv.org/3.3.1/dc/d6b/group__video__track.html#ga5d10ebbd59fe09c5f650289ec0ece5af).

- {: .hint} Hint: Take a look at this <a href="https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html">tutorial</a>, specifically the section on dense optical flow. Please post on piazza if you run into any issues or get stuck anywhere.

<img data-src="{{ 'assets/images/lab5/farneback-arm.gif' | absolute_url}}" class="lazyload mx-auto d-block" >
