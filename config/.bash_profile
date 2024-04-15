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

export ROS_LANG_DISABLE=genlisp:geneus

MACHINE_IP=$(ifconfig eth0 | grep 'inet ' | cut -d: -f3 | awk '{ print $2}')
if [ -z "$MACHINE_IP" ]
then
    MACHINE_IP=$(ifconfig wlan0 | grep 'inet ' | cut -d: -f3 | awk '{ print $2}')
    MACHINE_INTERFACE="wlan0"
else
    MACHINE_INTERFACE="eth0"
fi
export ROS_IP=$MACHINE_IP

export ROS_NETWORK_INTERFACE=$MACHINE_INTERFACE


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
alias pip='pip3'

# commands Pepper
alias wake="qicli call ALMotion.wakeUp"
alias rest="qicli call ALMotion.rest"
alias straight="sh ~/.local/share/scripts/set_my_pepper_straight.sh"
alias vision_services="sh ~/.local/share/scripts/vision_services.sh"
alias say="sh ~/.local/share/scripts/ALsay.sh"

export PNP_LIBRARY=/home/nao/.local/bin/usr/local/lib/

#Source ROS2
function humble(){
	. ~/ros2_humble/install/local_setup.bash
	. ~/catkin_ros2/install/local_setup.bash #naoqi_driver
}

humble

# there is a confusion between libqi.so egenrated for libqi-release package of ROS2 
# and the libqi.so generated for the libqi-python module, see https://github.com/aldebaran/libqi-python/issues/12
export LD_LIBRARY_PATH=/tmp/gentoo/usr/lib/python3.11/site-packages/qi:$LD_LIBRARY_PATH
