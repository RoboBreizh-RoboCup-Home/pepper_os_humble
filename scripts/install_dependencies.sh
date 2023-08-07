# Unsure we are on the prefix
/tmp/gentoo/executeonprefix

cd ~ && git clone https://github.com/RoboBreizh-RoboCup-Home/robobreizh_pepper_ws.git -b vscode-noetic
cd robobreizh_pepper_ws && cd src

# PetriNet hard link
ln -s /home/nao/.local/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . 
ln -s /home/nao/.local/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs .

# Compile everything
cd .. && catkin_make
vcs import < src/robobreizh_pepper_ws/robobreizh.repos src
catkin_make

source ~/robobreizh_pepper_ws/devel/setup.bash
