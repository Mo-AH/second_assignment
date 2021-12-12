# Second Assignment of Research Track 1 - Robotics Engineering - UniGE

This project makes use of [ROS](http://wiki.ros.org) (Robot-Operating-System), a set of software libraries and tools that allows us to build robot applications.
The goal of the project is to make a robot run autonomously in a circuit, that is a reproduction of the Monza F1 circuit, giving the user an interface to control the robot, like increasing velocity or to reset to initial position.

To do this, two nodes are created:
* `robot_node` = controls the movements of the robot, making him drive in counter-clockwise sense and avoiding to touch the walls of the circuit.
* `input_node` = takes specific buttons from user, sending them through a service to the robot_node

-----------------------

## Install and running

To run this simulator, [__ROS Noetic__](http://wiki.ros.org/noetic/Installation) is required and it should be installed in the machine, following the instructions of the linked page, and a ROS workspace should be created.

Done that, you can simply follow these steps:
1. clone this repository in the `src` folder of your ROS workspace.
2. launch `$catkin_make` command from the main directory of the workspace to build the package
3. launch `$roslaunch second_assignment run.launch` command to run all the nodes required
4. start the standard robot movement by pressing the `X` button in the console and modify speed/position parameters by using given commands from the input_node

-----------------------

# NODES

To do this assignment, we need to make nodes communicate with each other in the following way:

![image](https://user-images.githubusercontent.com/91679281/145716269-da555e81-0180-427a-a826-96e8b6ebd387.png)


## Stageros

`stageros` node is the one given by the professor, that simulates a world as defined in 
`my_world.world` file in the folder world. This file tells us everything about the environment: `.png` image which represents the circuit (Monza F1), the robot (a simple cube) is defined here and an initial posiion is set.

Stageros is subscribed to the topic `cmd_vel` from `geometry_msgs` package which provides a `Twist` type message: we can control the movement of the robot by publishing in this topic new velocities, both angular and linear.

Stageros publishes on the `base_scan` topic from `sensor_msgs` package which provides a `LaserScan` type message: we can get informations about walls's distances by subscribing to this topic. 

Finally, we can use a service called `reset_position` from the `std_srvs` package, which uses a type of message called `Empty`: there is no actual data inside it, but we can use it as a request to the service to reset the robot to its initial position.

## Robot_node

This is the main node of the assignment. It has to control the robot movements by publishing angular/linear velocities on the topic `cmd_vel` and it provides a service called `Speed_service` that is called by `input_node` to control the two speeds of the robot and its structure is:

```
char input
---
float32 linear
float32 angular
```

When it's called by `input_node`, with request being only a char, it performs some operation to change speeds or to reset position, and returns as a response the values of linear and angular speed.

That's his pseudocode:

```

Subscribe to `base_scan` topic from `stageros` node to get distances [1]
Provide a service `Speed_service` to get requests from `input_node` [2]
Create a publisher to `cmd_vel` topic to set velocities [3]


[1] get distances from stageros node (control callback)
    choose direction
    [3] move in the direction found (publish velocity)

[2] if received command (setSpeed callback)
    change speeds/position 

```


## Input_node

This is the node that allows the user to control the robot. It gives us information about the current speeds parameters and it provides a list of buttons that the user can input through the keyboard:

| Commands | Description|
|:--------:|:----------:|
|__[X]__   |START = Standard moving|
|__[G]__   |Decrease linear speed|
|__[H]__   |Increase linear speed|
|__[J]__   |Decrease angular speed|
|__[K]__   |Increase angular speed|
|__[S]__   |Stop motion|
|__[R]__   |Reset position and stop|

When a valid button is pressed from the user, the robot trasmits it to the robot_node, which responds by sending back the current speeds of the robot, that are printed. This is done through the service `Speed_service` from the robot_node.

### Pseudocode

```

Get the service `Speed_service` (as a client) to send requests to `robot_node` [1]

loop
    take command from user (getCommand function)
        controls if it's valid
        send request/get response [1]
        print new speeds

```
