---
layout: default
title: Install Ubuntu 18.04
nav_order: 1
permalink: /lab1/ubuntu
has_toc: true
parent: Lab 1
---

# Install Ubuntu 18.04
{: .no_toc .text-delta .fs-9 }

For this and the following labs, you need a (preferably clean) Ubuntu 18.04 LTS (Bionic Beaver) installation (see below). There are plenty of installation guides and tutorials on the web (and, in particular, on YouTube).

## Steps

1. Download the ISO image from [ubuntu.com](http://releases.ubuntu.com/18.04/ubuntu-18.04.3-desktop-amd64.iso){:target="_blank"}
2. Create a bootable USB stick
   - [How to create a bootable USB stick on **Windows**](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows){:target="_blank"}
   - [How to create a bootable USB stick on **Mac OS**](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-macos){:target="_blank"}
   - [How to create a bootable USB stick on **Ubuntu**](https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu){:target="_blank"}
3. Boot from USB stick and install
  - [Install Ubuntu desktop (full erase)](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop){:target="_blank"}
  - [Install Ubuntu alongside Windows (dual boot)](https://www.itzgeek.com/how-tos/linux/ubuntu-how-tos/how-to-install-ubuntu-18-04-alongside-with-windows-10-or-8-in-dual-boot.html){:target="_blank"}

<div class="alert alert-warning">
  <div class="alert-content">
    <h2 class="alert-title">
      Warning.
    </h2>
    <div class="alert-body">
      <p> Partitioning can be tricky if you are installing Linux for the first time. There are plenty of guides for “dual-boot Ubuntu installation” alongside both Windows and OS X. In most cases, you would first need to shrink one of your partitions (e.g., in Windows) and create an “unallocated space” which will be used during the Ubuntu installation process; see, e.g., this guide. </p>
      <br />
      <p><em>Ask for help if you are unsure.</em></p>
    </div>
  </div>
</div>

## Ubuntu Setup

Once Linux is installed we need to update all the packages, to do so open a terminal (<kbd>CTRL</kbd>+<kbd>Alt</kbd>+<kbd>T</kbd>) and type

```sh
sudo apt update
sudo apt upgrade
sudo apt install build-essential cmake
```
