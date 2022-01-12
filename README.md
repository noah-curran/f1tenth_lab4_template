# Lab 4: Follow the Gap

## I. Learning Goals

- Reactive methods for obstacle avoidance

## II. Overview

In this lab, you will implement a reactive algorithm for obstacle avoidance. While the base starter code defines an implementation of the F1TENTH Follow the Gap Algorithm, you are allowed to submit in C++, and encouraged to try different reactive algorithms or a combination of several. In total, the python code for the algorithm is only about 120 lines.

## III. Review of F1TENTH Follow the Gap

The lecture slides on F1TENTH Follow the gap is the best visual resource for understanding every step of the algorithm. However, the steps are outlined over here:

1. Obtain laser scans and preprocess them.
2. Find the closest point in the LiDAR ranges array.
3. Draw a safety bubble around this closest point and set all points inside this bubble to 0. All other non-zero points are now considered “gaps” or “free space”.
4. Find the max length “gap”, in other words, the largest number of consecutive non-zero elements in your ranges array.
5. Find the best goal point in this gap. Naively, this could be the furthest point away in your gap, but you can probably go faster if you follow the “Better Idea” method as described in lecture.
6. Actuate the car to move towards this goal point by publishing an `AckermannDriveStamped` to the /drive topic.

### IV. Implementation

Implement a reactive algorithm to make the car drive autonomously around the Levine Hall map. You are free to implement any reactive algorithm you want, but the skeleton code is for the F1TENTH follow the gap algorithm in lecture. You can implement this node in either C++ or Python. You can download the skeleton from Github classroom. There is also a test map `levine_blocked.pgm` for you to evaluate on.

### V. Deliverables and Submission

Please follow the submission.md in the Github repo.

### VI. Grading Rubric

We will test your code by accelerating the car down a straight towards a wall in the Levine map,
and your safety node should stop the car before collision.

- Compilation: **10** Points
- Implemented Find-Max Gap: **40** Points
- Implemented Find best point: **30** Points
- Levine Video: **10** Points
- Custom Map Video: **10** Points

### VII. Extra Resources

UNC Follow the Gap Video: https://youtu.be/ctTJHueaTcY
