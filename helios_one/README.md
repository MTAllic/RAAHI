## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

ros2 launch articubot_one launch_sim.launch.py world:=./src/articubot_one/worlds/office_small.world

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true

rviz2 -d main.rviz

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map_save.yaml -p use_sim_time:=true
ros2 run nav2_util lifecycle_bringup map_server




NAV2:

ros2 run twist_mux twist_mux  --ros-args --params-file ./src/articubot_one/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped

 ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true


AMCL_Nav2:
ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml use_sim_time:=true
 ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml src/articubot_one/config/
cp /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py src/articubot_one/launch/
cp /opt/ros/humble/share/nav2_bringup/launch/localization_launch.py src/articubot_one/launch/


YOLO:
ros2 launch helios_recognition launch_yolov8.launch.py


Nav test commands:
ros2 topic echo /goal_pose

ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -2.1577, y: -0.5017, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7906, w: 0.6123}}}"


ros2 launch yolo_to_llm launch_yolo_to_llm.launch.py





