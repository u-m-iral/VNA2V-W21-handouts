---
layout: default
title: Installing ROS
nav_order: 2
permalink: /lab2/ros
has_toc: true
parent: Lab 2
---

![ROS](https://www.generationrobots.com/blog/wp-content/uploads/2016/03/Logo-ROS-Robot-Operating-System1.jpg)

# Installing ROS
{: .no_toc .text-delta .fs-9 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Getting started with ROS

The Robot Operating System (ROS) is a crucial middleware (a.k.a. collection of software packages) that enables roboticists all over the world to implement their algorithms in a clean and modular fashion and share their work effectively with the community.

In addition, it is at the very core of our class, so we'd better start playing with it!

## Installing ROS

By now, you should have a working (preferably fresh) install of Ubuntu 18.04 and have become accustomed with the basics of Linux, Git and C++.
The most efficient way to install ros is through the Debian (binary) packages.

To install [ROS Melodic](http://wiki.ros.org/melodic){:target="_blank"} on it, we will follow the [official guide](http://wiki.ros.org/melodic/Installation/Ubuntu) to install the **Desktop-Full Install option**.

### Setup repositories

Let's add the `packages.ros.org` repository to our system

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

and setup the keys

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Installation

Before installing ROS we need to update the apt index

```bash
sudo apt update
```

Now let's install ROS 🤖

```bash
sudo apt install ros-melodic-desktop-full
```

Note: if you encounter the following problem when running the command line above:
```
Reading package lists... Done
Building dependency tree       
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 ros-melodic-desktop-full : Depends: ros-melodic-perception but it is not going to be installed
E: Unable to correct problems, you have held broken packages.
```
Then a solution is to instead of running `sudo apt install ros-melodic-desktop-full`, perform the following:
```bash
sudo apt install aptitude
sudo aptitude install ros-melodic-desktop-full
```

### Environment setup

It's convenient if the ROS environment variables are automatically loaded as soon a new shell is launched, let's edit `~/.bashrc` to do so

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools rosbash
```

### Initialization

Before using ROS, we need to initialize `rosdep`.

```bash
sudo rosdep init
rosdep update
```

