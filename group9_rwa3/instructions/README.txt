This file will describe how to run the room_recorder.py file so that the propper .yaml file will be generated in the 'records' folder

1) For this group the turtlebot3_house.world file was used instead of small_house.world. If you have already switched worlds in the 'mapping' package skip to step 4
2) In the 'mapping' package, open start_navigation.launch
3) Edit the following lines with what has been provided and save the file:
  <arg name="world_name" default="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world" />
  <arg name="model" default="waffle" />
  <arg name="x" default="-.5" />
  <arg name="y" default=".5" />
  <arg name="yaw" default="0" />
  <arg name="map_file" default="$(find mapping)/maps/my_map.yaml" />
4) Move the my_map.pgm and my_map.yaml files to the 'maps' folder in the 'mapping' package. These files are found in the 'map' folder
5) Before starting location recording read the new_room_breakdown file in the 'map' folder
6) Open the room_recorder.py file in gedit or pycharm
7) In the service_callback() function edit the file vairable as follows:
  file = open('/home/<user>/<your catkin worksapce>/src/group9_rwa3/records/location_recorder.yaml', 'a')
8) Navigate to your catkin workspace
9) Run $ roslaunch mapping start_navigation.launch. This will open the gazebo enviorment and rviz.
10) In a new terminal run $ rosrun group9_rwa3 room_recorder.py
11) Use the 2D nav tool in rviz to move the bot to the desired location
12) In a new terminal run $ rosservice call room_service <room_name>. This will save the position and orientation of the bot with the given room name in the location_recorder.yaml file in the 'records' folder
13) Repeat steps 9 and 10 until all rooms have been recorded
