FROM awesomebytes/pepper_2.5.5.5 AS base-gentoo

USER nao
WORKDIR /home/nao
RUN cat /proc/cpuinfo; cat /proc/meminfo; df -h

#Loading the ros2 archive
COPY gentoo_on_tmp.tar.lzma  ./gentoo_on_tmp.tar.lzma
RUN tar --lzma -xvf gentoo_on_tmp.tar.lzma &&\
    rm gentoo_on_tmp.tar.lzma

# Fix permissions of tmp
USER root
RUN chmod a=rwx,o+t /tmp
USER nao

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

# Let's make the compilations faster when possible
# Substitute the default -j2 with -j<NUM_CORES/2>
RUN sed -i -e 's/j1/j'"$((`grep -c \^processor \/proc\/cpuinfo` / 2))"'/g' $EPREFIX/etc/portage/make.conf
# Add extra jobs if we have enough CPUs
RUN sed -i 's/EMERGE_DEFAULT_OPTS=.*//' $EPREFIX/etc/portage/make.conf &&\
    echo "EMERGE_DEFAULT_OPTS=\"--jobs $((`grep -c \^processor \/proc\/cpuinfo` / 2)) --load-average `grep -c \^processor \/proc\/cpuinfo`\"" >> $EPREFIX/etc/portage/make.conf

RUN emerge patchelf

# Some utilities and dependencies for the next steps
# + tflite and opencv for the image processing
RUN pip install numpy pybind11 srt requests cffi tqdm tflite-runtime  opencv-python colcon-common-extensions coverage flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mock mypy pep8 pydocstyle pytest pytest-mock vcstool empy lark

# TODO: if time, could be good to try to compile openvino to compare performance against onnxruntime
# RUN git clone  https://github.com/openvinotoolkit/openvino.git  && \
#     cd openvino && \
#     git submodule update --init --recursive && \
#     mkdir build && cd build && \ 
#     # build options 
#     cmake -DCMAKE_BUILD_TYPE=Release \
#     # Python3.10
#     -DENABLE_PYTHON=ON -DENABLE_WHEEL=ON -DPYTHON_EXECUTABLE=`which python3.10` \
#     # Force CPU only, no multi-device
#     -DENABLE_INTEL_CPU=ON -DENABLE_INTEL_GPU=OFF -DENABLE_INTEL_GNA=OFF -DENABLE_HETERO=OFF -DENABLE_MULTI=OFF -DENABLE_PROXY=OFF \
#     # remove unused frontend
#     -DENABLE_OV_PADDLE_FRONTEND=OFF -DENABLE_OV_TF_FRONTEND=OFF -DENABLE_OV_PYTORCH_FRONTEND=OFF -DENABLE_OV_IR_FRONTEND=OFF \
#     # other useless options
#     -DOPENVINO_EXTRA_MODULES=OFF -DENABLE_IR_V7_READER=OFF -DENABLE_SYSTEM_PROTOBUF=OFF -DENABLE_LTO=ON -DCMAKE_CXX_FLAGS="-msse2" .. && \
#     make -j12
# cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_PYTHON=ON -DENABLE_WHEEL=ON -DPYTHON_EXECUTABLE=`which python3.10` -DENABLE_INTEL_CPU=ON -DENABLE_INTEL_GPU=OFF -DENABLE_INTEL_GNA=OFF -DENABLE_HETERO=OFF -DENABLE_MULTI=OFF -DENABLE_PROXY=OFF -DENABLE_OV_PADDLE_FRONTEND=OFF -DENABLE_OV_TF_FRONTEND=OFF -DENABLE_OV_PYTORCH_FRONTEND=OFF -DENABLE_OV_IR_FRONTEND=OFF -DOPENVINO_EXTRA_MODULES=OFF -DENABLE_IR_V7_READER=OFF -DENABLE_SYSTEM_PROTOBUF=OFF -DBUILD_SHARED_LIBS=OFF .. && make -j12

# Bulding of onnxruntime from source, not necessary because we have the wheel now
# RUN  git clone --recursive https://github.com/Microsoft/onnxruntime.git && \
#     cd onnxruntime && \
#     ./build.sh --config MinSizeRel --enable_pybind --build_wheel --parallel --cmake_extra_defines CMAKE_CXX_FLAGS="-msse2"

ADD --chown=nao:nao wheels/onnxruntime-1.16.0-cp310-cp310-linux_x86_64.whl onnxruntime-1.16.0-cp310-cp310-linux_x86_64.whl
RUN pip install ~/onnxruntime-1.16.0-cp310-cp310-linux_x86_64.whl && \
    rm ~/onnxruntime-1.16.0-cp310-cp310-linux_x86_64.whl

# LibQi python from source, not necessary because we have the wheel now
# Install of libqi Python3.8 bindings:
# RUN mkdir -p /home/nao/.local &&\
#     cd /home/nao/.local &&\
#     git clone https://github.com/aldebaran/qibuild.git && \
#     cd qibuild && \
#     python3.10 -m pip install -e . 

# RUN mkdir -p /home/nao/.local &&\
#     cd /home/nao/.local &&\
#     git clone https://github.com/aldebaran/libqi.git && \
#     cd libqi && \
#     mkdir build && cd build && \
#     cmake .. -DQI_WITH_TESTS=OFF -DCMAKE_PREFIX_PATH=/home/nao/.local/qibuild/cmake/qibuild -DCMAKE_INSTALL_PREFIX=/tmp/gentoo/usr -DWITH_BOOST_LOCALE=TRUE && \
#     make -j12 && \
#     make install 

# RUN mkdir -p /home/nao/.local &&\
#     cd /home/nao/.local &&\
#     git clone https://github.com/Maelic/libqi-python.git && cd libqi-python && \
#     mkdir build && cd build && \
#     cmake .. -DQI_WITH_TESTS=OFF -Dqi_DIR=/home/nao/.local/libqi/build/sdk/cmake -Dqibuild_DIR=/home/nao/.local/qibuild/cmake/qibuild -DCMAKE_INSTALL_PREFIX=/home/nao/.local/bin/libqi_python && \
#     cmake --build . --verbose && \x
#     cmake --install .

ADD --chown=nao:nao wheels/qi-3.1.1-cp310-cp310-linux_x86_64.whl qi-3.1.1-cp310-cp310-linux_x86_64.whl
RUN pip install ~/qi-3.1.1-cp310-cp310-linux_x86_64.whl && \
    rm ~/qi-3.1.1-cp310-cp310-linux_x86_64.whl

### Cross-compilation of Kaldi and Vosk for onboard speech recognition
RUN emerge sci-libs/lapack
RUN cd /home/nao/.local && \
    git clone -b vosk --single-branch https://github.com/alphacep/kaldi \
    && cd /home/nao/.local/kaldi/tools \
    && git clone -b v0.3.13 --single-branch https://github.com/xianyi/OpenBLAS \
    && git clone -b v3.2.1  --single-branch https://github.com/alphacep/clapack \
    && make -C OpenBLAS OPENBLAS_TARGET=ATOM TARGET=ATOM ONLY_CBLAS=1 DYNAMIC_ARCH=1 USE_LOCKING=1 USE_THREAD=0 all \
    && make -C OpenBLAS PREFIX=/home/nao/.local/kaldi/tools/OpenBLAS/install install \
    && mkdir -p clapack/BUILD && cd clapack/BUILD && cmake .. && make -j 8 || true 
RUN cd /home/nao/.local/kaldi/tools/clapack/BUILD \
    && find . -name "*.a" | xargs cp -t ../../OpenBLAS/install/lib \
    && cd /home/nao/.local/kaldi/tools \
    && git clone --single-branch https://github.com/alphacep/openfst openfst \
    && cd openfst \
    && autoreconf -i \
    && CFLAGS="-g -O3" CXXFLAGS="-msse -mfpmath=sse" ./configure --prefix=/home/nao/.local/kaldi/tools/openfst --enable-static --enable-shared --enable-far --enable-ngram-fsts --enable-lookahead-fsts --with-pic --disable-bin \
    && make -j 10 && make install \
    && cd /home/nao/.local/kaldi/src \
    && ./configure --mathlib=OPENBLAS_CLAPACK --shared --use-cuda=no \
    && sed -i 's:-msse -msse2:-msse -msse2:g' kaldi.mk \
    && sed -i 's: -O1 : -O3 :g' kaldi.mk \
    && make -j $(nproc) online2 lm rnnlm \
    && find /home/nao/.local/kaldi -name "*.o" -exec rm {} \;
RUN cd /home/nao/.local && \
    git clone https://github.com/alphacep/vosk-api \
    && cd vosk-api/src \
    && KALDI_ROOT=/home/nao/.local/kaldi make -j8 \
    && cd ../python \
    && python3.10 setup.py install

#Download ROS2
RUN mkdir -p /home/nao/ros2_humble/src &&\
    cd /home/nao/ros2_humble &&\
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src &&\
    #Remove unecessary packages (Unusable display and DDS implementations that do not work on 32 bits OS)
    rm -rf src/eclipse-iceoryx src/ros-visualization src/ros2/rviz src/ros/ros_tutorials/turtlesim 
    #rm -rf src/ros2/demos/intra_process_demo &&\

# Download ROS2 variants
RUN cd /home/nao/ros2_humble &&\
    git clone https://github.com/ros2/variants.git src/ros2/variants

# Remove line 21 of file ~/ros2_humble/src/geometry2/geometry2/package.xml
# This remove the tf2_bullet dependency so we can build without bullet3
RUN sed -i '21d' /home/nao/ros2_humble/src/ros2/geometry2/geometry2/package.xml

# Build ROS2 ros_base
RUN cd /home/nao/ros2_humble && \
    colcon build --symlink-install --packages-up-to ros_base --packages-skip tf2_bullet

# Attempt to build custom ros package such as laserscan, cv_bridge, diagnostics etc...
ADD --chown=nao:nao custom.repos /home/nao/ros2_humble/custom.repos
RUN cd /home/nao/ros2_humble && \
    vcs import src < custom.repos && \
    vcs pull src

# PCL won't compile with previous version of GCC so we need to re emerge it
RUN emerge =sys-devel/gcc-12.2.1_p20230428-r1 
RUN USE="openmp" emerge sci-libs/pcl

RUN cd /home/nao/ros2_humble && \
    . install/setup.bash && \
    colcon build --symlink-install --packages-select image_geometry \
    iceoryx_hoofs \
    iceoryx_posh \
    iceoryx_binding_c \
    sensor_msgs_py \
    cv_bridge \
    cyclonedds \
    camera_calibration_parsers \
    angles \
    camera_info_manager \
    image_transport \
    example_interfaces \
    composition \
    depthimage_to_laserscan \
    image_tools \
    intra_process_demo \
    lifecycle \
    logging_demo \
    # TODO: pcl related stuff doesn't build for wathever reason
    # pcl_msgs \
    # pcl_conversions \
    # perception_pcl \
    teleop_twist_keyboard \
    tlsf \
    tlsf_cpp \
    topic_monitor \
    image_common \
    # TODO: This also doesn't build, would need to be fixed later
    # image_pipeline \
    # image_transport_plugins \  
    # laser_filters \
    laser_geometry \
    vision_opencv \
    diagnostic_updater \
    diagnostic_aggregator \
    diagnostic_common_diagnostics \
    self_test \
    diagnostics \
    demo_nodes_cpp \
    demo_nodes_py 

# We re-build gtest because we need a version that works with c++11 rather than c++14 for naoqi_libqi
RUN cd .local && \
    git clone https://github.com/google/googletest.git -b release-1.12.1 && \
    cd googletest && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_GMOCK=OFF -DCMAKE_INSTALL_PREFIX=/tmp/gentoo/usr && \
    make -j12 && \
    make install

# Building of naoqi_driver and dependencies
RUN cd ros2_humble/src && \
    git clone https://github.com/victorpaleologue/naoqi_driver2.git -b fix_humble && \
    cd /home/nao/ros2_humble && \
    vcs import src < /home/nao/ros2_humble/src/naoqi_driver2/dependencies.repos && \
    vcs pull && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --symlink-install --packages-select naoqi_bridge_msgs naoqi_libqi naoqi_libqicore naoqi_driver

# Build ncnn
RUN git clone https://github.com/Tencent/ncnn.git && \
    cd ncnn && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/tmp/gentoo/usr -DNCNN_BUILD_EXAMPLES=ON .. && \
    make -j12 && \
    make install

# Testing if ncnn works
RUN cd ncnn/build && \
    cd ../benchmark && \
    ../build/benchmark/benchncnn 10 $(nproc) 0 -1

# Clean up
RUN rm -rf /home/nao/.local/

#Add configuration files.
#COPY --chown=nao:nao config/.gitconfig /home/nao/
#COPY --chown=nao:nao config/.my-credentials /home/nao/
#COPY --chown=nao:nao config/.git-credentials /home/nao/
COPY --chown=nao:nao config/.bash_profile /home/nao/
COPY --chown=nao:nao config/.vimrc /home/nao/
#ADD  --chown=nao:nao config/.vim_runtime /home/nao/.vim_runtime
COPY --chown=nao:nao config/.bash_profile /home/nao/.bash_profile


# Enable pulseaudio if anyone manually executes startprefix
# Adding to the line 'RETAIN="HOME=$HOME TERM=$TERM USER=$USER SHELL=$SHELL"'
RUN sed 's/SHELL=$SHELL/SHELL=$SHELL XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR/g' /tmp/gentoo/startprefix_original

RUN emerge app-portage/gentoolkit 
RUN eclean distfiles

# # # cleanup space first
RUN rm -rf /tmp/gentoo/var/cache/binpkgs/* /tmp/gentoo/var/tmp/* /home/nao/.cache/* /home/nao/gentoo/var/cache/distfiles/*
# Hack to earn extra 500MB~
SHELL ["/bin/sh", "-c"]
USER root
# Remove all .git directories
RUN find /home/nao -name ".git" -type d -exec rm -rf {} \; || true
RUN rm -rf /opt
RUN du -sh /tmp/gentoo
USER nao

SHELL ["/tmp/gentoo/executeonprefix"]

RUN cd /home/nao && tar -cv --lzma -f /tmp/pepper_os_humble.tar.lzma -C /home/nao gentoo -C  /home/nao ros2_humble -C /home/nao .bash_profile -C /home/nao -C /home/nao naoqi --remove-files --checkpoint=.100  --absolute-names || true

ENTRYPOINT ["/bin/bash"]