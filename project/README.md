## Self Balancing Robot Simulation using ROS and Gazebo

### Start
- Run simulation
```
roslaunch self_balancing_robot main.launch
```
- Run reinforcement learning
```
roscd self_balancing_robot/src
python robot_environment.py
```

### Issues
Remember to source workspace
```
source devel/setup.bash
```