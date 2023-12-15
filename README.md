Projekt polega na sterowaniu robotem ur3e za pomocą prostego subscribera i publishera. Do przetestowania działania potrzebna jest kamerka oraz kod AR np. wydrukowany.
Po skonfigurowaniu środowiska należy uruchomić komendy w terminalu (w folderze ze środowiskiem ROS):

`ros2 run ur_robot_driver start_ursim.sh -m ur3 ` - załącza sterownik do Universal Robots

Po włączeniu panelu ur3e należy uruchomić program z funkcją  External Control (ze Structure -> URCaps)

`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.56.101 launch_rviz:=true` - należy uruchomić komendę w osobnym oknie terminala, wyświetla robota w `Rviz2`

`ros2 run ur3e_aruco_controller camera_read ` - uruchamia camera_node z utworzonej paczki 

`ros2 run ur3e_aruco_controller robot_mover ` - uruchamia robot_node z utworzonej paczki.

W camera_node znajduje się publisher, który na stworzonym temacie `/ArUcoCenterAboveHalf` publikuje wiadomość w formacie `std_msgs/Bool`, wartość wiadomości
zależy od pozycji środka kodu ArUco - jeśli jest powyżej połowy ekranu to publikowana wiadomość przyjmuje wartość `true`, w przeciwnym wypadku `false`.

Robot_node subskrybuje temat `/ArUcoCenterAboveHalf` i reaguje na zmianę wartości wiadomości - jeśli zmienia się ona z `false` na `true` to ramię
robota wysuwa się do przodu, jeśli z `true` na `false` to ramię odjeżdża w tył. Node zawiera też publisher, który publikuje wiadomość typu `trajectory_msgs/msg/JointTrajectory`
na temacie `/scaled_joint_trajectory_controller/joint_trajectory` zawierającą pozycję robota (przód lub tył).
