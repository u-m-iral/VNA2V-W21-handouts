---
layout: default
title: Git
nav_order: 3
permalink: /lab1/git
has_toc: true
parent: Lab 1
---

# Git
{: .no_toc .text-delta .fs-9 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Intro

The following are selected chapters from [Pro Git](https://git-scm.com/book/en/v2){:target="_blank"}, if you are new to Git please read carefully these chapters as they pose the foundation of git.
1. [Getting Started](https://git-scm.com/book/en/v2/Getting-Started-Getting-Help){:target="_blank"}
2. [Getting a Git Repository](https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository){:target="_blank"}
3. [Recording Changes to the Repository](https://git-scm.com/book/en/v2/Git-Basics-Recording-Changes-to-the-Repository){:target="_blank"}
4. [Viewing the Commit History](https://git-scm.com/book/en/v2/Git-Basics-Viewing-the-Commit-History){:target="_blank"}
5. [Undoing Things](https://git-scm.com/book/en/v2/Git-Basics-Undoing-Things){:target="_blank"}
6. [Working with Remotes](https://git-scm.com/book/en/v2/Git-Basics-Working-with-Remotes){:target="_blank"}
7. [Branches in a Nutshell](https://git-scm.com/book/en/v2/Git-Branching-Branches-in-a-Nutshell){:target="_blank"}
8. [Basic Branching and Merging](https://git-scm.com/book/en/v2/Git-Branching-Basic-Branching-and-Merging){:target="_blank"}
9. [Branch Management](https://git-scm.com/book/en/v2/Git-Branching-Branch-Management){:target="_blank"}
10. [Branching Workflows](https://git-scm.com/book/en/v2/Git-Branching-Branching-Workflows){:target="_blank"}
11. [Remote Branches](https://git-scm.com/book/en/v2/Git-Branching-Remote-Branches){:target="_blank"}

Also visit [Git command reference](https://git-scm.com/docs){:target="_blank"} to get help with commands and command syntax.

---

The following exercises are designed to help you to experiment and learn how these commands are used in practice.

## Getting started

1. Install Git with (requires internet connection). For this you need to update APT cache and then install the git-core package
2. Add your name/email to your Git configuration (system-wide)

```bash
git config --global user.name YOUR_NAME
git config --global user.email YOUR_UMICH_EMAIL_ADDRESS
```

3. [Generate](https://docs.gitlab.com/ee/ssh/#generating-a-new-ssh-key-pair){:target="_blank"} SSH keys (do not forget the passphrase if you choose to set one)
4. [Add](https://docs.gitlab.com/ee/ssh/#adding-an-ssh-key-to-your-gitlab-account){:target="_blank"} SSH keys to your [gitlab.umich.edu account](https://gitlab.umich.edu/profile/keys){:target="_blank"}
5. [Create a new repository](https://docs.gitlab.com/ee/user/project/repository/#create-a-repository){:target="_blank"} on [https://gitlab.umich.edu](https://gitlab.umich.edu/){:target="_blank"}
6. Open a terminal and create a new directory using `mkdir` named `vna2v21` in your HOME directory
7. Clone your (empty) Git repo (earn street cred by calling by using "repo" instead of "repository")

```bash
git clone git@gitlab.umich.edu:USERNAME/REPO.git
```

## Merge Conflict with an Imaginary Collaborator

Now we simulate a common situation that arises when two or more people use the same repo.

1. Navigate to your repo and create new `me.txt` with **your name and UMICH email**, e.g.
```bash
$ cat me.txt
Jon Snow
lordsnow@umich.edu
```
2. Check the status with `git status`
3. Add (stage), check the status, commit and push your changes – commit message can be "Added my email"
```bash
add me.txt
git status
git commit -m "Added my email"
git push
```
4. Inspect the log  with `git log` 

Now you can go to your repo’s page on Github and inspect the commit history and contents of your repo.

Let's continue editing the files
1. Let's create a new branch
```bash
git checkout -b new_branch_to_merge_later
```
2. Edit the file `me.txt` with completely different content, e.g.
```bash
$ cat me.txt
Arya Stark
astark@umich.edu
```
3. Add (stage), check the status, and commit your changes (you can push too if you want) – commit message can be “Somebody added another email”
4. Now switch branch to master with `git checkout master`
5. Inspect the output of `git log --graph --oneline --all`
6. Append your course number to the file
```bash
$ echo "Course 16" >> me.txt
$ cat me.txt
Jon Snow
lordsnow@umich.edu
Course 16
```
6. Add (stage), check the status, and commit your changes – commit message can be “Added my course number”
7. Merge the two branches
```bash
$ git merge new_branch_to_merge_later
Auto-merging me.txt
CONFLICT (content): Merge conflict in me.txt
Automatic merge failed; fix conflicts and then commit the result.
```

BOOM 💥. A conflict appears. Thanks, Git for letting us know about this!
Let's resolve the conflict

1. Inspect the file `me.txt`, you should see something like - Git helps us by marking the conflict region with special characters: `HEAD` refers to your current branch/commit and below the `=======` the other commit
```bash
$ cat me.txt
<<<<<<< HEAD
Jon Snow
lordsnow@umich.edu
Course 16
=======
Arya Stark
astark@umich.edu
>>>>>>> new_branch_to_merge_later
```
2.  In this case, we would like to have Jon name so we simply remove everything else (including `<<<<<<< HEAD`) from the file
3.  After resolving the conflict, it is time to stage our file and create our merge commit - inspect the log, see the diff, and check the status
```bash
git add me.txt
git commit -m "Merge commit"
git push
```
4. Inspect the output of `git log --graph --oneline --all`
5. Inspect the output of `git diff HEAD~2` - what does this command do?
