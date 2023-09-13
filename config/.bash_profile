# Check if the link exists in /tmp/gentoo
if [ ! -L /tmp/gentoo ]; then
  echo "Softlink to this Gentoo Prefix in /tmp/gentoo does not exist, creating it..."
  cd /tmp
  ln -s /home/nao/gentoo gentoo
fi

alias tree="tree -L 2"
alias ..="cd .."
alias ll="ls -l"
alias cdws="cd ~/robobreizh_pepper_ws/"

splan(){
    rostopic pub /pnp/planToExec std_msgs/String \"data:\'$1\'\" -1
}

case $- in
    *i*) ;;
      *) return;;
esac

if grep -q /tmp/gentoo/bin/bash /proc/$$/cmdline ; then
    :
else
    EPREFIX=/tmp/gentoo
    SHELL=/tmp/gentoo/bin/bash
    echo "Entering ROS Pepperfix ${EPREFIX}"
    RETAIN="HOME=$HOME TERM=$TERM USER=$USER SHELL=$SHELL XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
    [[ -n ${PROFILEREAD} ]] && RETAIN+=" PROFILEREAD=$PROFILEREAD"
    [[ -n ${SSH_AUTH_SOCK} ]] && RETAIN+=" SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
    [[ -n ${DISPLAY} ]] && RETAIN+=" DISPLAY=$DISPLAY"
    env -i $RETAIN $SHELL -l
fi

alias nao_driver="ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=$ROS_IP network_interface:=$MACHINE_INTERFACE"

#Source ROS2
function humble(){
	. ~/ros2_humble/install/local_setup.bash
    echo "Sourcing ROS2..."
	# . ~/catkin_ros2/install/local_setup.bash #naoqi_driver
}

humble
