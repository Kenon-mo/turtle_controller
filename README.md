`ros2 run ur_robot_driver start_ursim.sh -m ur3 ` # Remember to set external control
`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 launch_rviz:=true`
`ros2 run ur3e_aruco_controller camera_read `
`ros2 run ur3e_aruco_controller robot_mover `
