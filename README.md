### 1. installation and build the file


install setting program
```
sudo apt-get -y update
sudo apt-get -y install --no-install-recommends \
                    libncurses5-dev \
                    libudev-dev \
                    libeigen3-dev \
                    git \
                    tmux \
                    curl \
                    wget \
                    htop \
                    python\
                    python-pip \
                    python-tk \
                    python-setuptools \
                    

                    
pip install -U numpy \
                    argparse \
                    pyyaml \
                    opencv-python \
                    rospkg \
                    catkin-pkg \
                    scipy \
                    matplotlib \
                    gym \
                    torch
                    torchvision \
                    
****(Optional)-if you want to work on python3

sudo apt-get -y install --no-install-recommends \
                    python3\
                    python3-pip \
                    python3-tk \
                    python3-setuptools \
                    
                    
pip3 install -U numpy \
                    argparse \
                    pyyaml \
                    opencv-python \
                    rospkg \
                    catkin-pkg \
                    scipy \
                    matplotlib \
                    gym \
                    torch
                    torchvision \
```


install dependent packages
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-ros-control ros-melodic-ros-controllers

```

install gazebo_ros_control
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
down load source file
```
git clone https://github.com/kaistcapstone/capstone1_ROS_gazebo
```
build the code
```
cd ~/capstone1_ROS_gazebo
catkin_make
```


# Install object detection library
```
cd ~/capstone2_ROS_gazebo
sudo pip install -r requirements.txt
```

(if you want to work in python3)
```
cd ~/capstone2_ROS_gazebo
sudo pip3 install -r requirements.txt
```

# Install Hector SLAM

- In terminal:

```
sudo apt install ros-melodic-hector-slam
roscd hector_mapping
cd launch
gedit mapping_default.launch
```

- In mapping_default.launch file:

change 
```
<arg name="odom_frame" default="nav"/>
```
to
```
<arg name="odom_frame" default="base_footprint"/>
```
add
```
<param name="tf_map_scanmatch_transform_frame_name" value="$(arg tf_map_scanmatch_transform_frame_name)" />
```
# build the code
```
cd ~/capstone2_ROS_gazebo
catkin_make
```

### 2. running the code

map open
```
roslaunch map_generate import_world.launch
```
swapn the robot
```
roslaunch turtlebot3_description spawn_turtlebot3.launch
```
launch the controller_manager
```
roslaunch junny_control junny_control.launch
```
steer the robot
```
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

```
->              q     w     e
                a     s     d
                      x
```
```
w: linear velocity increase
a: angular velocity increase to clockwise
s: stop
d: angular velocity increase to counter-clockwise
x: linear velocity decrease
```
detect and publish ball position from camera image
```
rosrun ball_detection ball_detect_node
```
draw the lidar data and ball position
```
rosrun data_integrate data_show_node
```

### Change object detection test image in map(gazebo)

- In terminal
'''
roscd map_generate or move to src/map_generate directory
cd worlds
gedit map_final_ball.world
'''

- In map_final_ball.world

change line124
<name>test1</name> 
to
<name>test3</name>

10 test image is provided. You can change test1 to test10.

Image source:
T.-Y. Lin, M. Maire, S. Belongie, J. Hays, P. Perona, D. Ra-manan, P. Doll ́ar, and C. L. Zitnick. Microsoft COCO: Com-mon objects in context. InECCV, 2014.


# Changing Animal Picture 

In terminal:

```
rosrun capstone_scoring animal_topic.py
```

- After putting a ball into the goal. 
Publish to /current_ball topic, 1 or 2 depending on the number of goal-ins you have.

- For different order of animal picutre, choose case value in the animal_topic.py
cases : 1 or 2 or 3

- Randomly Relocating an object in the entrance zone

---

# Randomly Locating Stop Sign

In terminal:

```
rosrun capstone_scoring stop.py
```
- Change the values in line 14 of stop.py

```python
goal_y =  np.random.uniform([lower bound],[upper bound])
```
# Cantact

"Do Hakgi" <kevindo@kaist.ac.kr>,
"Bae Jaewoong" <jwoong.bae@kaist.ac.kr>,

