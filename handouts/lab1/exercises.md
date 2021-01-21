---
layout: default
title: Exercises
nav_order: 5
permalink: /lab1/exercises
has_toc: true
parent: Lab 1
---

# Exercises
{: .fs-9 }

## Submission

To submit your solution, you will be creating a `.zip` file following the exercises below and upload it on Canvas under **Assignment > Lab 1: Linux, Git, C++ - handout** by **[January 27 at midnight (11:59 EST)](https://www.timeanddate.com/worldclock/fixedtime.html?msg=VNA2V+Lab+Deadline&iso=20210127T235959&p1=784)**.


<!-- ~~To submit your solutions you are required to create a repository in the _VNA2V-W21-Submissions_ group in gitlab.umich.edu (this will be your first exercise).~~

~~VNA2V staff will clone your repository from gitlab.umich.edu on **[January 27 at midnight (11:59 EST)](https://www.timeanddate.com/worldclock/fixedtime.html?msg=VNA2V+Lab+Deadline&iso=20210127T235959&p1=784)**.
This will be considered as your submission and will be graded accordingly.~~ -->

<div class="alert alert-warning">
  <div class="alert-content">
    <h2 class="alert-title">
      Late Submission.
    </h2>
    <div class="alert-body">
      <p>Please email us if you want to submit later than the deadline. Otherwise late penalty will be applied.</p>
    </div>
  </div>
</div>

## Exercises 

### Git (5 pts)

In this exercise you are required to set a git repository, for example inside your own GitLab namespace. You will be downloading the zip file of this repository and submit it to Canvas.

1. Create a repository for your personal submissions
  - Go to [https://gitlab.umich.edu/your_namespace](https://gitlab.umich.edu/){:target="_blank"} and click on "New Project" to create a new repository
  - Create a new **Private** repository and call it as your UMICH unique-name (let's call it "YOUR_UNIQUENAME"), e.g. if your UMICH email is _astark@umich.edu_, call it _astark_
  - Clone the repository to `~/vna2v-personal` (you will have a team submission later) running `git clone git@gitlab.umich.edu:your_namespace/YOUR_UNIQUENAME.git ~/vna2v-personal` (replace `YOUR_UNIQUENAME` with the name of the repo you just created)
  - Create a folder called `lab1`
2. Clone [https://gitlab.umich.edu/VNA2V-W21/labs](https://gitlab.umich.edu/VNA2V-W21/labs){:target="_blank"} in a folder of your choice

You are required to put your solutions in the repository you created in the first Git exercise.

<!-- ~~In this exercise you are required to set a git repository inside the _VNA2V-W21-Submission_ group.
This is require for the correct submission of all the exercises of the class.~~

1. ~~Create a repository for your personal submissions~~
   - ~~Go to [https://gitlab.umich.edu/VNA2V-W21-submissions](https://gitlab.umich.edu/VNA2V-W21-submissions){:target="_blank"} and click on "New Project" to create a new repository~~
   - ~~Create a new **Private** repository and call it as your Kerberos username, e.g. if your UMICH email is _astark@umich.edu_, call it _astark_~~
   - ~~Clone the repository to `~/vna2v-personal` (you will have a team submission later) running `git clone git@gitlab.umich.edu:VNA2V-W21-submissions/YOUR_USERNAME.git ~/vna2v-personal` (replace `YOUR_USERNAME` with the name of the repo you just created)~~
   - ~~Create a folder called `lab1`~~
2. ~~Clone [https://gitlab.umich.edu/VNA2V-W21/labs](https://gitlab.umich.edu/VNA2V-W21/labs){:target="_blank"} in a folder of your choice~~

~~You are required to put your solutions in the repository you created in the first Git exercise.~~

<div class="alert alert-warning">
  <div class="alert-content">
    <h2 class="alert-title">
     Warning. 
    </h2>
    <div class="alert-body">
      <p>If you created the repository in your personal account instead of VNA2V-W21-submissions you need to transfer the ownership in order to complete your submission. Scroll to the bottom of the page for instructions.</p>
    </div>
  </div>
</div> --> 

### Shell (35 pts)

1. Exercise 1 - Answer to the following questions
   - Download `https://raw.githubusercontent.com/dlang/druntime/master/benchmark/extra-files/dante.txt` (try using `wget`)
   - Create a file called `exercise1.txt` in `~/vna2v-personal/lab1` and answer to the following questions
     1. How many lines does it contains?
     2. How many words does it contains?
     3. How many lines are not blank?
   - Push the file to git
2. Exercise 2 - Output redirecting
  - Install `fortune-mod` using `apt`
  - After installation, type `fortune` in your terminal to see a (hopefully) interesting proverb/quote
  - Run `fortune` 5 more times and each time redirect the output to a file called `fortunes.txt` in `~/vna2v-personal/lab1` (Hint: do not recreate the file 5 times - each time a new proverb should be added to the end of `fortunes.txt`)
  - Push the file to git

> **Hint**: For the first exercise you might want to use the command `wc` (Word Count).

### C++: Warm-up Exercises (20 pts)

Feel free to refer to [this](https://en.cppreference.com/w/) when answering the following questions.
Some of the questions below are based on [C++ Primer](https://www.oreilly.com/library/view/c-primer-fifth/9780133053043/), which is also an excellent resource for C++ programming.
Put all answers into a text file called `cpp-warmup.txt` and push it to git.

*Operators*
1. What are the values of `i` and `j` after running the following code?
```cpp
int i = 0, j;
j = ++i;
j = i++;
```
2. What does the following code print?
```cpp 
int i = 42;
std::string output = (i < 42) ? "a" : "b";
std::cout << output << std::endl;
```

*References and Pointers*
1. What does the following code print?
```cpp
int i;
int& ri = i;
i = 5;
ri = 10;
std::cout << i << " " << ri << std::endl;
```
2. What does the following code print?
```cpp
int i = 42;
int* j = &i;
*j = *j**j;
std::cout << *j << std::endl;
```
3. What does the following code print?
```cpp
int i[4] = {42,24,42,24};
*(i+2) = *(i+1)-i[3];
std::cout << *(i+2) << std::endl;
```
4. What does the following code print?

```cpp
void reset(int &i) {
    i = 0;
}

int j = 42;
reset(j);
std::cout << j << std::endl;
```

*Numbers*
1. What are the differences between `int`, `long`, `long long`, and `short`? 
2. What are the differences between a `float` and `double`? What is the value of `i` after running the following code snippet?
```cpp
int i;
i = 3.14;
```
3. What are the differences between an unsigned and signed type? What is the value of `c` in the following code snippet assuming `chars` are 8-bit?
```cpp
unsigned char c = -1;
```
4. What will the value of `i` be after running the following code snippet?
```cpp
int i = 42;
if (i) {
    i = 0;
} else {
    i = 43;
}
```


### C++: RandomVector (40 pts)

In this exercise we will implement the class `RandomVector`.
Inside `~/vna2v-personal/lab1` create a folder called `RandomVector` and copy the content from [https://gitlab.umich.edu/VNA2V-W21/Labs/tree/master/lab1](https://gitlab.umich.edu/VNA2V-W21/labs/tree/master/lab1){:target="_blank"}.

The class `RandomVector` defined in the header file `random_vector.h` abstract a vector of doubles.
You are required to implement the following methods:

- `RandomVector(int size, double max_val = 1)` (constructor): initialize a vector of doubles of size `size` with random values between 0 and `max_val` (default value 1)
- `double mean()` returns the mean of the values in random vector
- `double max()` returns the max of the values in random vector
- `double min()` returns the min of the values in random vector
- `void print()` prints all the values in the random vector
- `void printHistogram(int bins)` computes the histogram of the values using `bins` number of bins between `min()` and `max()` and print the histogram itself (see the example below).

To to so complete all the `TODO`s in the file `random_vector.cpp`. When you are done compile the application by running

```bash
g++ -std=c++11 -Wall -pedantic -o random_vector main.cpp random_vector.cpp
```

**Note:** we expect you to not use the function from the `<algorithm>` header.

If you complete correctly the exercise you should see something like

```bash
$ ./random_vector
0.458724 0.779985 0.212415 0.0667949 0.622538 0.999018 0.489585 0.460587 0.0795612 0.185496 0.629162 0.328032 0.242169 0.139671 0.453804 0.083038 0.619352 0.454482 0.477426 0.0904966
Mean: 0.393617
Min: 0.0667949
Max: 0.999018
Histogram:
***     ***
***     ***
***     ***
***     ***
***     ***
***     ***
***     *** ***
*** *** *** *** ***
```

> **Optional (10 pts)**: Try to implement the methods **with and without** the functions available in the header `<algorithm>`.


<!-- 
## Transfer ownership of Git repository

It is possible to transfer ownership of your projects on GitLab.

~~If you created the repository in your personal account instead of _VNA2V-W21-submissions_ you might want to transfer the ownership in order to complete your submission.~~

1. On GitHub, navigate to the main page of the repository.
2. On the side bar menu, click **Settings -> General** and Expand on **Advanced**.  
![Step 1]({{ '/assets/images/lab1/TransferOwnershipStep1.png' | relative_url}})
1. Scroll down until your reach **transfer project**, select a suitable repository from the drop-down list, and click on **Transfer Project**.
![Step 1]({{ '/assets/images/lab1/TransferOwnershipStep2.png' | relative_url}})
1. Type the name of your repository, then click **Confirm**.  
![Step 1]({{ '/assets/images/lab1/TransferOwnershipStep3.png' | relative_url}})
5. Done! -->
