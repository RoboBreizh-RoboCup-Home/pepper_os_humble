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

export RTABMap_DIR=~/.local/rtabmap/build
export PATH=~/.local/bin:$PATH
export PYTHONPATH=/home/nao/.local/bin/libqi_python/lib/python3.8/site-packages:$PYTHONPATH

export ROS_LANG_DISABLE=genlisp:geneus
if [[ $(ps aux | grep roscore_ip | grep -v grep) ]]; then
    export ROS_MASTER_URI=`/tmp/gentoo/usr/bin/python3.8 -c "import rosgraph; print(rosgraph.Master('ip_checker').getUri())"`
    export ROS_IP=`/tmp/gentoo/usr/bin/python3.8 -c "import rosgraph; print(rosgraph.Master('ip_checker').getUri().replace('http://','').replace(':11311/',''))"`
    export ROS_NETWORK_INTERFACE=`ifconfig | grep $ROS_IP -1 | awk 'NR==1{print $1}' | sed 's/://g'`
else
    MACHINE_IP=$(ifconfig eth0 | grep 'inet ' | cut -d: -f3 | awk '{ print $2}')
    if [ -z "$MACHINE_IP" ]
    then
        MACHINE_IP=$(ifconfig wlan0 | grep 'inet ' | cut -d: -f3 | awk '{ print $2}')
        MACHINE_INTERFACE="wlan0"
    else
        MACHINE_INTERFACE="eth0"
    fi
    export ROS_IP=$MACHINE_IP

    echo "No roscore found, setting ROS_MASTER_URI and ROS_IP to machine ip."
    export ROS_MASTER_URI=http://$MACHINE_IP:11311
    export ROS_NETWORK_INTERFACE=$MACHINE_INTERFACE
 fi

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

export PNP_LIBRARY=/home/nao/.local/bin/usr/local/lib/
export ROS_PACKAGE_PATH=/home/nao/.local/rtabmap/build/bin/:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=/home/nao/.local/bin/libqi/lib:/home/nao/.local/bin/usr/local/lib/

#Source ROS2
function humble(){
	. ~/ros2_humble/install/local_setup.bash
	# . ~/catkin_ros2/install/local_setup.bash #naoqi_driver
}

humble
