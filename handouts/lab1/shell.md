---
layout: default
title: Shell basics
nav_order: 2
permalink: /lab1/shell
has_toc: true
parent: Lab 1
---

# Shell basics
{: .no_toc .text-delta .fs-9 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Exploring the Filesystem

### `pwd`

Modern filesystems are organized in folders, being able to navigate the filesystem is fundamental.
Everytime we work with the shell we are within one folder, to know where we are we can use the command `pwd` (_Print Working Directory_):

```bash
$ pwd
/home/username/vna2v2021/lab1
```

where `username` is the login username you set.

### `ls`

To list the contents of the current directory (files and/or child directories, etc.) we use `ls` (LiSt)

```bash
$ ls
ex0.cpp   ex1.cpp   ex2.cpp   final.cpp
```

#### File permissions and ownership

The concept of permissions and ownership is crucial in anu unix system. Every to file and directory is assigned 3 types of **owner**:

- _User_: is the owner of the file, by default, the person who created a file
- _Group_: user-group can contain multiple users, all users belonging to a group will have the same access permissions to the file
- _Other_: Any other user who has access to a file

At the same time to every file and directory is assigned a type of permission
- _Read_
- _Write_
- _Execute_

We get all this information using `ls -l`, for example:
```sh
$ ls -l
total 1112
-rw-r--r--  1 username  staff  557042 Aug 24 21:57 dante.txt
-rwxr-xr-x  1 username  staff      40 Aug 23 18:36 hello.sh
-rw-r--r--  1 username  staff     171 Aug 23 18:28 hello_vna2v2021.tar.gz
-rw-r--r--  1 username  staff      49 Aug 24 22:55 numbers.txt
```
The permissions are specified by the 1st field, the ownership is specified by the 3rd and 4th fields. Fo example, the file `hello.sh` is owned by me (`username`) and the group is `staff`. THe permission string is `-rwxr-xr-x` meaning that:
- The owner can read (r), write (w) and execute (x) the file
- The group can read and execute
- Other can read and execute

### `cd`
To change the current folder we can use `cd` (Change Directory). For example `cd /` moves to the file system root or

```bash
cd /home
```

To move to the parent of the current folder we use `cd ..`, it can also be concatenated like `cd ../..` to move two (or more) levels up. To move back to your home folder we use `cd ~` (or simply `cd`).

### `find`

Image you have a folder containing many files and you want to locate a file called `findme.txt`. To accomplish it you can use
```bash
find . -name "findme.txt"
```
Let's analyze the command. The `.` represent the current folder, so we are saying to `find` to look in the current folder recursively (you can change it with relative or absolute paths) for a file called `findme.txt`. Find is a powerful tool, you can have complex expression to match files, have a look at `find --help`.

## Edit Filesystem

### `mkdir`
`mkdir` (make directory) is used to create new, empty directories: let's create a new dir named `newdir`

```bash
$ mkdir newdir
$ ls 
newdir
$ cd newdir
```

### `touch`

`touch` was created to modify file timestamps, but it can also be used to quickly create an empty file. You can easily create a `newfile.txt` with

```bash
$ touch newfile.txt
$ ls
newfile.txt
```

### `rm`

You can remove any file with `rm` -- **be careful**, this is non-recoverable! I suggest to add the flag `-i` to prompt a confirmation message

```bash
rm -i newfile.txt
rm: remove regular empty file 'newfile.txt'? y
```

You can also remove directories with `rm`, the only catch is that it returns an error when the folder is not-empty. The common practice, but pretty prone to non-recoverable errors, is to run `rm -rf foldername`. The command will remove the folder with all its content (`r` - recursive) forcing the operation (`f` - force). This operation will not ask for confirmation. You can of course add the flag `i` (i.e. `rm -rfi foldername`) but will ask confirmation for **every file**, this is pretty annoying if the folder contains many files.

### `cp`

Copying file is as simple as running `cp` (CoPy). If we want to duplicate the file `numbers.txt` we can run

```bash
$ cp numbers.txt numbers_copy.txt
$ls
numbers.txt  numbers_copy.txt
```

### `mv`

If we want to rename `numbers_copy.txt` to `new_numbers.txt` we can run

```bash
$ mv numbers_copy.txt new_numbers.txt
$ ls
new_numbers.txt  numbers.txt
```

With the same command we can also move the file to another location, for example if we want to move `numbers.txt` to a newly create folder `dataset` we execute

```bash
$ mkdir dataset
$ mv numbers.txt dataset/numbers.txt
$ ls dataset
numbers.txt
```

## Viewing and Editing Files

### `cat`
`cat` concatenates a list of files and sends them to the standard output stream and is often used to quickly view the content of a file. For example we can inspect the content of the file `numbers.txt`.

```bash
$ cat numbers.txt
One
Two
Three
Four
Five
Six
Seven
Eight
Nine
Ten
```

### `nano` and `vim`

`nano` is a minimalistic command-line text editor. It's a great editor for beginners. More demanding user pefer `vim`. It's a powerful and highly customizable text editor (I love it!). I strongly suggest to learn how to use vim, one of the best way to learn vim is to simply run `vimtutor` in your terminal but if you prefer games try [Vim Adventures](https://vim-adventures.com)!

## Download, uncompress and execute a script

In this section we will download a compressed file, extract the content, inspect and run a script.

### Download

Imagine you have to download (you have to, actually) `http://www.umich.edu/~username/hello_vna2v21.tar.gz` you can use `wget`.

```bash
wget http://www.umich.edu/~username/hello_vna2v21.tar.gz
```

### Uncompress

As you see it is a compressed file, to uncompress it we can use `tar`

```bash
tar -xvf hello_vna2v2021.tar.gz
```

The flags `xvf` are respectively extract, verbose file.
Now we would like to run the script. We should first inspect the file (never run a script without inspection), let's use `cat`

```bash
$ cat hello.sh
#!/usr/bin/env bash
echo "Hello world!"
```

The file is not dangerous, it only print something to the terminal. 

### Run

Before running the script we should verify that we can actually run the script so let's see its permissions

```bash
$ ls -l hello.sh
-rw-r--r--  1 username  staff  40 Aug 23 18:36 hello.sh
```

Ops! This time we have no right to run the script, we have to add it:

```bash
chmod +x hello.sh
```

Let's check again:

```bash
ls -l hello.sh
-rwxr-xr-x  1 username  staff  40 Aug 23 18:36 hello.sh
```

Ok, now we can execute the script. To execute the script it's enough to add `./` before the name of the file to e

```bash
$ ./hello.sh
Hello world!
```

<div class="alert alert-warning">
  <div class="alert-content">
    <h2 class="alert-title">
      Keep in mind.
    </h2>
    <div class="alert-body">
      <p> When you use <code>./</code> the bash shell is creating a <strong>new</strong> shell, child of the current one and executing the code there. This is usually fine, sometimes you need to run a script like it was prompted directly in the current bash, in that case you should use the <code>source</code> command, e.g. <code>source hello.sh</code></p>
    </div>
  </div>
</div>

## Pipe

The Pipe is a command in Linux that lets you use two or more commands such that output of one command serves as input to the next. In short, the output of each process directly as input to the next one like a pipeline. The symbol `|` denotes a pipe.

For example, consider the following file:

```bash
$ cat numbers.txt
One
Two
Three
Four
Five
Six
Seven
Eight
Nine
Ten
```

We can sort the lines piping `cat` with `sort`

```bash
$ cat numbers.txt | sort
Eight
Five
Four
Nine
One
Seven
Six
Ten
Three
Two
```

## Output redirect

We redirect the output of a command to a file. This is useful when we want to save the output of a program without writing specific code.

The common commands that we use and their results are

- `command > output.txt`

The standard output stream will be redirected to the file only, it will not be visible in the terminal. If the file already exists, it gets overwritten.

- `command &> output.txt`

Both the standard output and standard error stream will be redirected to the file only, nothing will be visible in the terminal. If the file already exists, it gets overwritten.

- `command | tee output.txt`

The standard output stream will be copied to the file, it will still be visible in the terminal. If the file already exists, it gets overwritten.

- `command |& tee output.txt`

Both the standard output and standard error streams will be copied to the file while still being visible in the terminal. If the file already exists, it gets overwritten.

<div class="alert alert-info">
  <div class="alert-content">
    <h2 class="alert-title">
      Moreover.
    </h2>
    <div class="alert-body">
      <p> If you want to append instead of overwrite you can use the double angle brackets <code>>></code>.
      With <code>tee</code> instead add the flag <code>-a</code> (e.g. <code>tee -a output.txt</code>).</p>
    </div>
  </div>
</div>

## Superuser

Working with the terminal you wil, sooner or later, get a "Permission denied" error. This occur because you do not have the right permission to run the command.

For example if you try install vim you might get something like

```bash
$ apt install sl
E: Could not open lock file /var/lib/dpkg/lock-frontend - open (13: Permission denied)
E: Unable to acquire the dpkg frontend lock (/var/lib/dpkg/lock-frontend), are you root?
```

The superuser (usually "root") is the only person who can install software, to install vim we must elevate ourself to system administrator . The command we need to use is `sudo`

```bash
$ sudo apt install sl
[sudo] password for username:
Reading package lists... Done
Building dependency tree       
Reading state information... Done
The following NEW packages will be installed:
  sl
0 upgraded, 1 newly installed, 0 to remove and 2 not upgraded.
Need to get 26.4 kB of archives.
After this operation, 98.3 kB of additional disk space will be used.
Get:1 http://us.archive.ubuntu.com/ubuntu bionic/universe amd64 sl amd64 3.03-17build2 [26.4 kB]
Fetched 26.4 kB in 0s (250 kB/s)
Selecting previously unselected package sl.
(Reading database ... 162980 files and directories currently installed.)
Preparing to unpack .../sl_3.03-17build2_amd64.deb ...
Unpacking sl (3.03-17build2) ...
Setting up sl (3.03-17build2) ...
Processing triggers for man-db (2.8.3-2ubuntu0.1) ...
```

## Install packages

Ubuntu, like any other Linux distribution based on Debian, use the [dpkg packaging system](https://wiki.debian.org/DebianPackageManagement).
A packaging system is a way to provide programs and applications for installation. This way, we don’t have to build every program from the source.

APT (Advanced Package Tool) is the command line tool to interact with the packaging system. Installing a package that is available on one of the repository known by the system is as easy as running

```bash
sudo apt install <package_1> <package_2> <package_3>
```

For example if we want to install the package `sl` we would simply run

```bash
sudo apt install sl
```

> Try to run `sl` now!

## Get help

It's hard remember all commands with all their flags, `man` command in Linux is used to display the user manual of any command that we can run on the terminal.

Moreover many commands offer an help (for example try to run `ls --help`). The common ways to summon the help is via the flags `-h` or `--help`.

Last but not least, Google is your friend!
