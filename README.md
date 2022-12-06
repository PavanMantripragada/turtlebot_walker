# turtlebot_walker
A ROS2 package for generating roomba like behaviour for turtlebot3

## Demo
![](results/output.gif)

## Dependencies
- ROS Humble Installed on PC 
- rclcpp
- geometry_msgs
- sensor_msgs
- turtlebot3

## Creating a ros workspace
```
mkdir ros2_ws/src
```

## Cloning the package
```
cd <path to ros workspace>/ros2_ws/src
git clone https://github.com/PavanMantripragada/turtlebot_walker.git
```

## Building the package
```
cd <path to ros workspace>/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot_walker
```

## For Running Code
Source the workspace by running `source install/setup.bash` in all terminals.

### To run with recording bag data
```
export TURTLEBOT3_MODEL=burger
cd <path to repository>/turtlebot_walker/launch
ros2 launch turtlebot_walker everything_launch.xml bag_record:=1
```
### To run without recording bag data
```
export TURTLEBOT3_MODEL=burger
cd <path to repository>/turtlebot_walker/launch
ros2 launch turtlebot_walker everything_launch.xml bag_record:=0
```
To terminate press `Ctrl+C` on the terminal.

### To see info of recorded bag file
```
cd <path to repository>/turtlebot_walker/results
ros2 bag info ./all_topics
```
### To play the recorded bag file
```
cd <path to repository>/turtlebot_walker/results
ros2 bag play ./all_topics
```

## Auxilary Information for Developers

## Formating the code
```
cd <path to repository>/turtlebot_walker
clang-format -style=Google -i ./src/walker.cpp  ./src/main.cpp ./include/turtlebot_walker/walker.hpp
```

## Running cppcheck
```
cd <path to repository>/turtlebot_walker
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name \*.hpp -or -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/")
```

## Running cpplint
```
cd <path to repository>/turtlebot_walker
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```