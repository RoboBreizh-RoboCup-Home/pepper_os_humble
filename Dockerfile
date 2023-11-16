FROM awesomebytes/pepper_2.5.5.5

USER nao
WORKDIR /home/nao

COPY gentoo_on_tmp.tar.lzma /home/nao/gentoo_on_tmp.tar.lzma
RUN tar --lzma -xvf gentoo_on_tmp.tar.lzma &&\
    rm gentoo_on_tmp.tar.lzma

# Fix permissions of tmp
USER root
RUN chmod a=rwx,o+t /tmp
USER nao

ENV CHOST i686-pc-linux-gnu
ENV CFLAGS -march=i686 -O2 -pipe

# Prepare environment to run everything in the prefixed shell
RUN cd /tmp && ln -s /home/nao/gentoo gentoo &&\
    cp /etc/group /tmp/gentoo/etc/group || true &&\
    cp /etc/passwd /tmp/gentoo/etc/passwd || true
# # To make sure everything builds and reports i686 we do this trick
# RUN sed -i 's/env -i/linux32 env -i/' /tmp/gentoo/executeonprefix
# To allow the use of the $EPREFIX variable
RUN sed -i 's/SHELL=$SHELL"/SHELL=$SHELL EPREFIX=$EPREFIX"/' /tmp/gentoo/executeonprefix

# And now switch the shell so every RUN command is executed in it
SHELL ["/tmp/gentoo/executeonprefix"]

# So we dont get complains
RUN echo "en_US.UTF-8 UTF-8" >> $EPREFIX/etc/locale.gen
RUN $PREFIXED locale-gen

# Let's make the compilations faster when possible
# Substitute the default -j2 with -j<NUM_CORES/2>
RUN rm -rf $EPREFIX/etc/portage/make.conf
RUN touch $EPREFIX/etc/portage/make.conf
RUN sed -i -e 's/j2/j'"$((`grep -c \^processor \/proc\/cpuinfo` / 2))"'/g' $EPREFIX/etc/portage/make.conf

# Add extra jobs if we have enough CPUs
RUN echo "EMERGE_DEFAULT_OPTS=\"--jobs $((`grep -c \^processor \/proc\/cpuinfo` / 2)) --load-average `grep -c \^processor \/proc\/cpuinfo`\"" >> $EPREFIX/etc/portage/make.conf
RUN cat $EPREFIX/etc/portage/make.conf

# dev-vcs/git-2.20.1 fails with (this command was already here, but executed after emerging git)
# ERROR: cannot verify www.kernel.org's certificate
# rosdep fails with
# ERROR: cannot verify pypi.org's certificate, issued by 'CN=DigiCert SHA2 Extended Validation Server CA,OU=www.digicert.com,O=DigiCert Inc,C=US':
#  Unable to locally verify the issuer's authority
# RUN $PREFIXED emerge dev-libs/openssl app-misc/ca-certificates net-misc/wget net-misc/curl
RUN update-ca-certificates
RUN echo 'ACCEPT_LICENSE="*"' >> $EPREFIX/etc/portage/make.conf

#Update portage
RUN emerge --oneshot sys-apps/portage

# Emerging git
RUN emerge dev-vcs/git

# Set up Gentoo to sync via git (way faster)
RUN rm -rf $EPREFIX/usr/portage && mkdir -p $EPREFIX/etc/portage/repos.conf && echo "[DEFAULT]\n\
main-repo = gentoo\n\
\n\
[gentoo]\n\
location = /tmp/gentoo/usr/portage\n\
sync-type = git\n\
sync-uri = https://github.com/gentoo-mirror/gentoo \n\
auto-sync = yes\n\
masters = gentoo" > $EPREFIX/etc/portage/repos.conf/gentoo.conf

# Sync everything
RUN emaint sync -a
RUN emerge --sync 
RUN emerge gentoo-sources

RUN emerge dev-python/pip
# I don't know why this is needed, but it is
RUN rm -rf /tmp/gentoo/usr/lib/python3.11/EXTERNALLY-MANAGED

# Some dependencies
# Don't ask me why openblas needs BINARY=32, but it does
RUN BINARY=32 emerge openblas
RUN NPY_DISABLE_SVML=1 python3.11 -m pip install numpy==1.22.2
RUN python3.11 -m pip install opencv-python colcon-common-extensions coverage flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mock mypy pep8 pydocstyle pytest pytest-mock vcstool empy lark catkin_pkg
RUN emerge dev-util/cmake sys-apps/dbus dev-libs/tinyxml2 dev-cpp/eigen media-libs/libglvnd dev-libs/openssl app-admin/eselect sci-libs/orocos_kdl gtest

RUN echo ">=media-libs/libglvnd-1.7.0 X" >> $EPREFIX/etc/portage/package.use
RUN FEATURES="-collision-detect -protect-owned" emerge media-libs/mesa

# build libqi
RUN emerge =dev-libs/boost-1.82.0-r1
RUN pip install qibuild scikit-build toml pybind11==2.11.1

RUN mkdir -p /home/nao/.local &&\
    cd /home/nao/.local &&\
    git clone https://github.com/victorpaleologue/libqi-python.git && \
    cd libqi-python &&\
    export BOOST_VERSION=1.82 && \
    export PYBIND11_VERSION=2.11.1 && \
    python3.11 ./setup.py bdist_wheel


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
    git clone --branch fix_humble https://github.com/victorpaleologue/naoqi_driver2.git &&\
    git clone --branch humble https://github.com/ros-perception/vision_opencv &&\  
    git clone --branch debian/galactic/naoqi_libqi https://github.com/ros-naoqi/libqi-release &&\
    git clone --branch debian/galactic/naoqi_libqicore https://github.com/ros-naoqi/libqicore-release &&\             
    git clone https://github.com/ros-naoqi/naoqi_bridge_msgs2 &&\
    git clone https://github.com/ros/diagnostics


WORKDIR /home/nao/

ENTRYPOINT ["/bin/bash"]