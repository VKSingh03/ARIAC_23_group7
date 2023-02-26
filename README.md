# README


# ARIAC 2023 : ENPM663 RWA1

## Group Members: ##

### Vineet Singh (vsingh03@gmail.com)
### Ishan Tamarkar (itamraka@umd.edu)
### Krishna Hundekari (krishnah@umd.edu) 
### Pranav Shinde (pshinde@umd.edu)


## Steps to run the code: 
1. Place the package **group7** in ARIAC Workspace
2. In Terminal, move to root directory of ARIAC and build the package using colcon build

        colcon build --packages-select group7

3. Source the workspace build is complete and launch the AIRAC 

        ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa1

4. Launch the **group7** package using 

        ros2 run group7 group7_start_competition




