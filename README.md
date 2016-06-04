# usma_roboteq
## ROS Roboteq Driver
This package is a python script used to drive the roboteq motor controller. Originally, this code was created in support of the IGVC competition at Oakland University, Michigan, but can easily be repurposed for other chassis in other projects.

## Package Use
1. ```git clone``` this package
2. No catkin_make is necesary. There are no compiled languages in this package.
3. ```roslaunch roboteq roboteq_driver.launch```

## TODO List
[x] Create udev rules that map the roboteq driver to ```/dev/roboteq```
[x] Allow the driver to publish to the arduino light controller for autonomous navigation
[ ] Fix the driver post navigation issue

