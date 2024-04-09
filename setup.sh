#!/bin/bash
echo 'Make sure IP addys have been added to hosts'
read -p "Enter your Raspi IP Addy: " raspi_ip
read -p "Enter your laptop IP Addy: " laptop_ip
echo $raspi_ip $laptop_ip
Port=5001

source ~/cg2111a/devel/setup.bash
export ROS_MASTER_URI=http://$laptop_ip:11311
export ROS_IP=$laptop_ip

roscore &
 
sleep 5
gnome-terminal -- bash -c "ssh -t pi@$raspi_ip './rplidar-setup.sh $laptop_ip $raspi_ip; bash'"
sleep 5
gnome-terminal -- bash -c "source ~/cg2111a/devel/setup.bash;roslaunch rplidar_ros view_slam.launch;  exec bash"
sleep 5
gnome-terminal -- bash -c "ssh -t pi@$raspi_ip 'cd Alex-main/TLS_server/;./tls-alex-server4; bash'"
sleep 15
gnome-terminal -- bash -c "cd TLS-programming;./tls-alex-client4 $raspi_ip $Port;exec bash"
wait





: '

source ~/cg2111a/devel/setup.bash
export ROS_MASTER_URI=http://$laptop_ip:11311
export ROS_IP=$laptop_ip

roscore &
 
sleep 3
gnome-terminal -- bash -c "ssh -t pi@$raspi_ip 'echo run ./rplidar-setup.launch $laptop_ip $raspi_ip;bash'"
gnome-terminal -- bash -c "source ~/cg2111a/devel/setup.bash;echo 'run: roslaunch rplidar_ros view_slam.launch';  exec bash"
gnome-terminal -- bash -c "ssh -t pi@$raspi_ip 'cd Alex-main/TLS_server/; echo run ./tls-alex-server[edition number] ; bash'"
gnome-terminal -- bash -c "cd TLS-programming; echo run ./tls-alex-client[ver no];exec  bash"
wait

'



