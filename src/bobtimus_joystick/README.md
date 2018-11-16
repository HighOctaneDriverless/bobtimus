# Bobtimus Joystick
Node to allow controlling the racecar with a common gaming controller.
It takes the input from the joy node and performs actions based on the pressed buttons.

### How to use it 
##### Step 1) 

Install the Joy node for Ros Melodic

```
# On Ubuntu
sudo apt install ros-melodic-joy
```

##### Step 2)
 
Find the controller input

```
ls /dev/input/ | grep js*
```
If the command does not return a controller use the following commands:
```
# On Ubuntu
sudo apt-add-repository -y ppa:rael-gc/ubuntu-xboxdrv
sudo apt-get update
sudo apt-get install ubuntu-xboxdrv
```
##### Step 3)

Adapt permissions for the controller and setup joy node parameter

```
sudo chmod a+rw /dev/input/[your controller]
rosparam set joy_node/dev "/dev/input/[your controller]"
```

##### Step 4)

Start the node

```
rosrun joy joy_node
```

