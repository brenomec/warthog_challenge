# warthog_challenge

Clone the repository into a catkin workspace and run:

```console
rosdep install --from-paths src --ignore-src -r -y
```

After instaling all the dependencies run:

```console
catkin_make
source your_workspace/devel/setup.bash
```

With a builded repo you can launch the simulation and start the mission:

```console
roslaunch warthog_desafio aruco_mission.launch
rosrun warthog_desafio aruco_go.py
```
