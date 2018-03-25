#!/bin/bash

# ROS installation and catkin initialization script

# Keys
echo "Updating keys and updating apt-get"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update

# ROS installation

PS3='Do you want to install ROS?: '
options=("Yes" "No")
select opt in "${options[@]}"
do
case $opt in
	"Yes")
		sudo apt-get -y install ros-kinetic-desktop-full
		sudo rosdep init
		rosdep update
		echo "Installing Smach Viewer"
		sudo apt-get -y install ros-kinetic-smach-viewer
		echo "Installing Map Server"
		sudo apt-get -y install ros-kinetic-map-server
		PS3='Would you like to add ROS to your bashrc?: '
		options=("Yes" "No")
		select opt in "${options[@]}"
		do
		case $opt in
			"Yes")
				echo "Adding ROS to .bashrc. A backup file is created as .bashrc_before_ros"
				cp ~/.bashrc ~/.bashrc_before_ros
				echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
				echo "Sourcing .bashrc"
				source ~/.bashrc
				break
				;;
			"No")
				echo "Sourcing ROS"
				source /opt/ros/kinetic/setup.bash
				break
				;;
			esac
		done

		PS3='Would you like to apply the ROS bugfix?: '
		options=("Yes" "No")
		select opt in "${options[@]}"
		do
		case $opt in
			"Yes")
				echo "Fixing stage.hh in opt/ros/kinetic/include/Stage-4.1/"
				sudo cp bugfix/stage.hh /opt/ros/kinetic/include/Stage-4.1/stage.hh
				echo "Fixing joint.hpp in opt/ros/kinetic/include/kdl/"
				sudo cp bugfix/joint.hpp /opt/ros/kinetic/include/kdl/joint.hpp
				echo "Applied bugfix."
				break
				;;
			"No")
				break
				;;
			esac
		done
		break
		;;
	"No")
		break
		;;
	esac
done

# Catkin workspace

PS3='Do you want to create your catkin workspace?: '
options=("Yes" "No")
select opt in "${options[@]}"
do
case $opt in
	"Yes")
		echo "Initializing catkin workspace"
		cd $HOME
		echo "Creating directories"
		mkdir catkin_ws
		cd catkin_ws
		mkdir src
		cd src
		catkin_init_workspace
		cd ..
		catkin_make
		echo "Sourcing the workspace"
		source devel/setup.bash
		PS3='Would you like to add catkin to your bashrc?: '
		options=("Yes" "No")
		select opt in "${options[@]}"
		do
		case $opt in
			"Yes")
				echo "Adding catkin to .bashrc. A backup file is created as .bashrc_before_catkin"
				cp ~/.bashrc ~/.bashrc_before_catkin
				echo "Adding catkin workspace to bashrc"
				echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
				echo "Sourcing .bashrc"
				source ~/.bashrc
				break
				;;
			"No")
				echo "Sourcing Catkin"
				source devel/setup.bash
				break
				;;
			esac
		done
		break
		;;
	"No")
		break
		;;
	esac
done

echo "Goodbye!"


