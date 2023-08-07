FROM awesomebytes/pepper_2.5.5.5 AS base-gentoo

USER nao
WORKDIR /home/nao

#Download and install the custom made pepper_os.
RUN cat /proc/cpuinfo; cat /proc/meminfo; df -h


#Loading the pepper_os v11 archive.
#RUN wget --no-check-certificate -O pepper_os_11.0.4.tar.lzma "https://mycore.core-cloud.net/index.php/s/Rh4EbGqxc05W3ap/download?path=%2F&files=pepper_os_11.0.4.tar.lzma"
COPY ./pepper_os_11.0.4.tar.lzma ./pepper_os_11.0.4.tar.lzma
RUN tar --lzma -xvf pepper_os_11.0.4.tar.lzma &&\
    rm pepper_os_11.0.4.tar.lzma

# Fix permissions of tmp
USER root
RUN chmod a=rwx,o+t /tmp
USER nao

# Prepare environment to run everything in the prefixed shell
RUN cd /tmp && ln -s /home/nao/gentoo gentoo &&\
    cp /etc/group /tmp/gentoo/etc/group || true &&\
    cp /etc/passwd /tmp/gentoo/etc/passwd || true
# To make sure everything builds and reports i686 we do this trick
RUN sed -i 's/env -i/linux32 env -i/' /tmp/gentoo/executeonprefix
# To allow the use of the $EPREFIX variable
RUN sed -i 's/SHELL=$SHELL"/SHELL=$SHELL EPREFIX=$EPREFIX"/' /tmp/gentoo/executeonprefix

# And now switch the shell so every RUN command is executed in it
SHELL ["/tmp/gentoo/executeonprefix"]

# THIS IS VERY UNSAFE FOR THE STABILITY OF THE OS
RUN emaint sync -a
FROM base-gentoo AS interactive
WORKDIR /home/nao/

# update pip
RUN python3.8 -m pip install --upgrade pip

#Add configuration files.
#COPY --chown=nao:nao config/.gitconfig /home/nao/
#COPY --chown=nao:nao config/.my-credentials /home/nao/
#COPY --chown=nao:nao config/.git-credentials /home/nao/
COPY --chown=nao:nao config/.bash_profile /home/nao/
COPY --chown=nao:nao config/.vimrc /home/nao/
#ADD  --chown=nao:nao config/.vim_runtime /home/nao/.vim_runtime
COPY --chown=nao:nao config/.bash_profile /home/nao/.bash_profile

# Remove obsolete packages
RUN cd /home/nao/.local &&\
    rm -rf vosk-api rtabmap qibuild opencv_contrib opencv-4.5.5 kaldi &&\
    cd /tmp/gentoo/var/cache &&\
    rm -rf *

#Select python3.8 as the default version and hiding python3.10 to prevent colcon from building ROS2 using a mix of python3.8 and python3.10.
RUN eselect python set python3.8
RUN mv /tmp/gentoo/usr/bin/python3.10 /tmp/gentoo/usr/bin/python3.10_temp

#Install ROS2 python dependencies
RUN python3.8 -m ensurepip --upgrade
RUN python3.8 -m pip install opencv-python colcon-common-extensions coverage flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mock mypy pep8 pydocstyle pytest pytest-mock vcstool empy lark

#Install asio (Required by ROS2)
RUN cd ~/ &&\
 wget https://sourceforge.net/projects/asio/files/asio/1.28.0%20%28Stable%29/asio-1.28.0.tar.gz &&\
 tar -xzf asio-1.28.0.tar.gz &&\
 cd asio-1.28.0/ &&\
 autoreconf -i &&\
 ./configure &&\
 make &&\
 make prefix=/tmp/gentoo/usr libdir=/tmp/gentoo/lib install 

#Download ROS2
RUN mkdir -p ~/ros2_humble/src &&\
    cd ~/ros2_humble &&\
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src &&\
    #Remove unecessary packages (Unusable display and DDS implementations that do not work on 32 bits OS)
    rm -rf src/eclipse-cyclonedds src/eclipse-iceoryx src/ros-visualization src/ros2/rviz src/ros/ros_tutorials/turtlesim &&\
    #Build ROS2
    colcon build --symlink-install &&\
    #Building will fail due to the mimick package. While compiling mimick downloads and builds mimick_vendor. The CMakeList.txt of mimick_vendor doesn't take into account 32 bits architecture like Pepper and has to be adjusted accordingly.   
    sed -i '61s/.*/  set (_ARCH "x86")/' ~/ros2_humble/build/mimick_vendor/mimick-de11f8377eb95f932a03707b583bf3d4ce5bd3e7-prefix/src/mimick-de11f8377eb95f932a03707b583bf3d4ce5bd3e7/CMakeLists.txt &&\
    colcon build --symlink-install

#Download naoqi_driver
RUN . ~/ros2_humble/install/local_setup.bash &&\
    mkdir -p ~/catkin_ros2/src &&\
    cd ~/catkin_ros2/src/ &&\
    git clone https://github.com/ros-naoqi/naoqi_driver2 &&\
    git clone --branch humble https://github.com/ros-perception/vision_opencv &&\  
    git clone --branch debian/galactic/naoqi_libqi https://github.com/ros-naoqi/libqi-release &&\
    git clone --branch debian/galactic/naoqi_libqicore https://github.com/ros-naoqi/libqicore-release &&\             
    git clone https://github.com/ros-naoqi/naoqi_bridge_msgs2 &&\
    git clone https://github.com/ros/diagnostics

#Adjust several naoqi_driver files to be compatible with ROS2 Humble 
#As of Humble, Support for rclcpp::Duration(int64 nanoseconds) has been deprecated and removed. This require fixing several bits of code. 
RUN sed -i '122s/.*/    rclcpp::Duration d(static_cast<std::chrono::nanoseconds>((helpers::Time::now() - msg.header.stamp).nanoseconds()));/' /data/home/nao/catkin_ros2/src/naoqi_driver2/src/recorder/basic_event.hpp

RUN sed -i '132s/.*/    rclcpp::Duration d(static_cast<std::chrono::nanoseconds>((time - msg.header.stamp).nanoseconds()));/' /data/home/nao/catkin_ros2/src/naoqi_driver2/src/recorder/basic_event.hpp

RUN sed -i '294s/.*/            rclcpp::Duration d(static_cast<std::chrono::nanoseconds>((schedule - this->now()).nanoseconds()));/' /data/home/nao/catkin_ros2/src/naoqi_driver2/src/naoqi_driver.cpp

#The declare_parameters(char* parameter_name) function doesn't exist and requires at least a second parameter. 
RUN sed -i '42s#.*#  rclcpp::ParameterValue value = node->declare_parameter(parameter_name, rclcpp::PARAMETER_STRING);#' /data/home/nao/catkin_ros2/src/naoqi_driver2/src/publishers/info.cpp

#Humble no longer supports node_executable and node_name, those have to be changed to executable and name respectively. 
RUN sed -i '35s#.*#            executable="naoqi_driver_node",#' /data/home/nao/catkin_ros2/src/naoqi_driver2/launch/naoqi_driver.launch.py

RUN sed -i '36s#.*#            name=[launch.substitutions.LaunchConfiguration("namespace")],#' /data/home/nao/catkin_ros2/src/naoqi_driver2/launch/naoqi_driver.launch.py

#Compile naoqi_driver
RUN cd ~/catkin_ros2 &&\
    . ~/ros2_humble/install/local_setup.bash &&\
    colcon build --symlink-install

#Rename python3.10 back to its original name 
RUN mv /tmp/gentoo/usr/bin/python3.10_temp /tmp/gentoo/usr/bin/python3.10

#Deleting older version of ROS
RUN rm -rf /home/nao/gentoo/opt/ros/noetic

#Compress archive to lzma format.
#WARNING: This operation takes about an hour and a half.
#cd /home/nao; tar -c --lzma -f /tmp/pepper_os.tar.lzma -C /home/nao gentoo -C  /home/nao asio-1.28.0 -C /home/nao ros2_humble -C /home/nao .local -C /home/nao .bash_profile -C /home/nao naoqi -C /home/nao catkin_ros2 || true

SHELL ["/tmp/gentoo/executeonprefix"]
CMD ["/home/nao/gentoo/startprefix"]
