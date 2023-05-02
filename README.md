# README


# ARIAC 2023 : ENPM663

## Group Members: ##

### Vineet Singh (vsingh03@gmail.com)
### Krishna Hundekari (krishnah@umd.edu) 
### Ishan Tamarkar (itamraka@umd.edu)
### Pranav Shinde (pshinde@umd.edu)


## Steps to run the code: 
1. Place the package **group7** in ARIAC Workspace
2. In Terminal, move to root directory of ARIAC and build the workspace using colcon build

        colcon build

    OR only build the package using 

        colcon build --packages-select group7

3. Source the workspace after build is complete and launch the AIRAC 

        ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3 competitor_pkg:=group7 sensor_config:=group7_sensors

4. In a new terminal source ARIAC workspace and launch MoveIt

        ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

4. In a new terminal source ARIAC workspace and start the competition using **group7** package launch file

        ros2 launch group7 group7.launch.py

Output Video for RWA3 : https://youtu.be/J83ADXhgkKc

Output Video for RWA4 : https://youtu.be/dmnHMJKD2i4



```
group7
├─ .vscode
│  ├─ c_cpp_properties.json
│  └─ settings.json
├─ CMakeLists.txt
├─ README.md
├─ config
│  └─ group7_sensors.yaml
├─ etc
│  ├─ RWA2 Group7 Control Diagram.jpg
│  └─ instructions.txt
├─ include
│  └─ group7
│     ├─ competition_state_subscriber.hpp
│     └─ order_class.hpp
├─ launch
│  └─ group7.launch.py
├─ package.xml
├─ rviz
│  └─ group7.rviz
└─ src
   ├─ .vscode
   │  ├─ c_cpp_properties.json
   │  └─ settings.json
   └─ competition_state_subscriber.cpp

```