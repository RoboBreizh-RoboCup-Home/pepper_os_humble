FROM awesomebytes/pepper_2.5.5.5

USER nao
WORKDIR /home/nao

#Loading the ros2-prepared archive
COPY gentoo_prefix_base.tar.lzma  ./gentoo_prefix_base.tar.lzma
RUN tar --lzma -xvf gentoo_prefix_base.tar.lzma &&\
    rm gentoo_prefix_base.tar.lzma

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

# remove python3.12
RUN emerge --unmerge python:3.12
RUN emerge --depclean
RUN rm -rf /tmp/gentoo/usr/bin/python3.12 /tmp/gentoo/usr/bin/python3.12-config

#install onnxruntime from wheel
ADD wheels/onnxruntime-1.15.1-cp311-cp311-linux_i686.whl /tmp/gentoo/onnxruntime-1.15.1-cp311-cp311-linux_i686.whl
RUN pip install /tmp/gentoo/onnxruntime-1.15.1-cp311-cp311-linux_i686.whl && rm /tmp/gentoo/onnxruntime-1.15.1-cp311-cp311-linux_i686.whl

RUN mkdir -p /home/nao/.local &&\
    cd /home/nao/.local &&\
    git clone https://github.com/aldebaran/qibuild.git && \
    cd qibuild && \
    python -m pip install -e . 

RUN mkdir -p /home/nao/.local &&\
    cd /home/nao/.local &&\
    git clone https://github.com/aldebaran/libqi.git -b qi-framework-v3.0.0 && \
    cd libqi && \
    mkdir build && cd build && \
    cmake .. -DQI_WITH_TESTS=OFF -DCMAKE_PREFIX_PATH=/home/nao/.local/qibuild/cmake/qibuild -DCMAKE_INSTALL_PREFIX=/tmp/gentoo/usr -DCMAKE_CXX_FLAGS='-std=c++17' && \
    make -j12 && \
    make install

# RUN mkdir -p /home/nao/.local &&\
#     cd /home/nao/.local &&\
#     git clone https://github.com/victorpaleologue/libqi-python.git && cd libqi-python && \
#     BOOST_VERSION=1.78 PYBIND11_VERSION=2.11.1 python setup.py bdist_wheel -DQI_WITH_TESTS=OFF -Dqi_DIR=/home/nao/.local/libqi/build/sdk/cmake -Dqibuild_DIR=/home/nao/.local/qibuild/cmake/qibuild && \
#     pip install dist/*.whl

#install libqi-python from wheel
ADD wheels/qi-3.1.0-cp311-cp311-linux_i686.whl /tmp/gentoo/qi-3.1.0-cp311-cp311-linux_i686.whl
RUN pip install /tmp/gentoo/qi-3.1.0-cp311-cp311-linux_i686.whl && rm /tmp/gentoo/qi-3.1.0-cp311-cp311-linux_i686.whl

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
    
# dependencies for vosk
RUN pip install tqdm cffi requests srt
RUN cd /home/nao/.local && \
    git clone https://github.com/alphacep/vosk-api \
    && cd vosk-api/src \
    && KALDI_ROOT=/home/nao/.local/kaldi make -j8 \
    && cd ../python \
    && python setup.py install

RUN emerge sci-libs/orocos_kdl

### THIS IS THE BUILD OF ONNXRUNTIME FROM SOURCE - NOT NEEDED ANYMORE
# Need to compile pybind11 from source to fight the redefinition of PyFrameObject:
# https://github.com/sirfz/tesserocr/issues/298
RUN cd ~/.local && git clone https://github.com/pybind/pybind11.git -b v2.10.0 &&  cd pybind11 && \
    mkdir build && \
    cd build && \
    cmake .. -DPYBIND11_PYTHON_VERSION=3.11 -DCMAKE_INSTALL_PREFIX=/tmp/gentoo/usr && \
    make -j12 && \
    make install

# ONNX won't install protoc correctly by default so we do it beforehand
# RUN cd ~/.local && wget https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protoc-21.12-linux-x86_32.zip && \
#     unzip -o protoc-21.12-linux-x86_32.zip -d /tmp/gentoo/usr/local/ bin/protoc && \
#     unzip -o protoc-21.12-linux-x86_32.zip -d /tmp/gentoo/usr/local/ 'include/*' && \
#     ldconfig && \
#     rm protoc-21.12-linux-x86_32.zip

# build latest onnxruntime version
# RUN cd .local && git clone --recursive https://github.com/Microsoft/onnxruntime.git -b v1.15.1 && \
#     cd onnxruntime && \ 
#     ./build.sh --config Release --parallel --build_wheel --disable_ml_ops --update --build --x86 --cmake_extra_defines CMAKE_INSTALL_PREFIX=/tmp/gentoo/usr CMAKE_CXX_FLAGS="-lstdc++ -msse -msse4.1 -mfpmath=sse" CMAKE_SYSTEM_NAME=Linux CMAKE_SYSTEM_PROCESSOR=i686 ONNX_CUSTOM_PROTOC_EXECUTABLE=/tmp/gentoo/usr/local/bin/protoc && \
#     cd build/Linux/Release/ && \
#     make install && \
#     cd dist && pip install *.whl

#Download ROS2
RUN mkdir -p ~/ros2_humble/src &&\
    cd ~/ros2_humble &&\
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src &&\
    #Remove unecessary packages (Unusable display and DDS implementations that do not work on 32 bits OS)
    rm -rf src/eclipse-cyclonedds src/eclipse-iceoryx src/ros-visualization src/ros2/rviz src/ros/ros_tutorials/turtlesim src/ros2/geometry2/tf2_bullet src/ros2/geometry2/test_tf2 &&\
    cd src && git clone https://github.com/ptrmu/ros2_shared.git && cd .. &&\
    #Build ROS2
    colcon build --symlink-install; exit 0

RUN cd ~/ros2_humble/ &&\
    #Building will fail due to the mimick package. While compiling mimick downloads and builds mimick_vendor. The CMakeList.txt of mimick_vendor doesn't take into account 32 bits architecture like Pepper and has to be adjusted accordingly.   
    sed -i '61s/.*/  set (_ARCH "x86")/' ~/ros2_humble/build/mimick_vendor/mimick-de11f8377eb95f932a03707b583bf3d4ce5bd3e7-prefix/src/mimick-de11f8377eb95f932a03707b583bf3d4ce5bd3e7/CMakeLists.txt &&\
    colcon build --symlink-install

# #Download and build naoqi_driver2
# we build without tests because otherwise libqi-release will fail and I'm too lazy to fix it (related to Gtest not allowing c++14 anymore)
RUN . ~/ros2_humble/install/local_setup.bash &&\
    mkdir -p ~/catkin_ros2/src &&\
    cd ~/catkin_ros2/src/ &&\
    git clone https://github.com/Maelic/naoqi_driver2.git &&\
    git clone --branch humble https://github.com/ros-perception/vision_opencv &&\  
    git clone --branch release/humble/naoqi_libqi https://github.com/ros-naoqi/libqi-release &&\
    git clone --branch release/humble/naoqi_libqicore https://github.com/ros-naoqi/libqicore-release &&\             
    git clone https://github.com/ros-naoqi/naoqi_bridge_msgs2 &&\
    git clone https://github.com/ros/diagnostics && \
    cd .. && colcon build --symlink-install  --cmake-args -DBUILD_TESTING=OFF


# RUN  cmake -DCMAKE_BUILD_TYPE=Release -DTREAT_WARNING_AS_ERROR=OFF -DENABLE_AVX512F=OFF -DBUILD_SHARED_LIBS=OFF -DENABLE_PYTHON=ON -DENABLE_INTEL_GNA=OFF -DENABLE_IR_V7_READER=OFF -DENABLE_GAPI_PREPROCESSING=OFF -DENABLE_ONEDNN_FOR_GPU=OFF -DENABLE_INTEL_GPU=OFF -DENABLE_OV_PADDLE_FRONTEND=OFF -DENABLE_OV_IR_FRONTEND=OFF -DENABLE_INTEL_CPU=OFF -DENABLE_WHEEL=ON -DENABLE_OV_TF_FRONTEND=OFF -DENABLE_TBB_RELEASE_ONLY=OFF -DENABLE_INTEL_MYRIAD=OFF -DTHREADING=SEQ -DENABLE_OV_ONNX_FRONTEND=OFF ..

RUN BLIS_ARCH="generic" pip install transformers spacy
# for wathever reason the wheel is not working
# ADD wheels/spacy-3.7.4-cp311-cp311-linux_i686.whl /tmp/gentoo/spacy-3.7.4-cp311-cp311-linux_i686.whl
# RUN pip install /tmp/gentoo/spacy-3.7.4-cp311-cp311-linux_i686.whl --no-binary blis && rm /tmp/gentoo/spacy-3.7.4-cp311-cp311-linux_i686.whl
RUN python3.11 -m spacy download en_core_web_sm

RUN emerge dev-vcs/git-lfs

# for GStreamer
RUN emerge media-libs/gstreamer media-libs/gst-plugins-base media-libs/gst-plugins-good media-plugins/gst-plugins-jpeg media-plugins/gst-plugins-v4l2

# Build tensorflow lite - legacy, can be used for debuging purposes
# RUN cd .local && git clone https://github.com/tensorflow/tensorflow.git tensorflow_src
# RUN cd .local/tensorflow_src && \
#     PYTHON=python3.11 tensorflow/lite/tools/pip_package/build_pip_package_with_cmake.sh silvermont
# RUN pip install /data/home/nao/.local/tensorflow_src/tensorflow/lite/tools/pip_package/gen/tflite_pip/python3/dist/tflite_runtime-2.16.0-cp311-cp311-linux_i686.whl

#install tflite from wheel
ADD wheels/tflite_runtime-2.16.0-cp311-cp311-linux_i686.whl /tmp/gentoo/tflite_runtime-2.16.0-cp311-cp311-linux_i686.whl
RUN pip install /tmp/gentoo/tflite_runtime-2.16.0-cp311-cp311-linux_i686.whl && rm /tmp/gentoo/tflite_runtime-2.16.0-cp311-cp311-linux_i686.whl

# git config to avoid ssl verification for the clock of Pepper which is not up to date (2014)
RUN git config --global http.sslVerify false

# Takes care of initializing the shell correctly
COPY --chown=nao:nao config/.bash_profile /home/nao/.bash_profile

# copy the content of scripts to ~/.local/share/scripts
COPY --chown=nao:nao scripts /home/nao/.local/share/scripts

# Hack to earn extra 500MB~
SHELL ["/bin/sh", "-c"]
USER root
RUN cd .local && rm -rf vosk-api libqi libqi-python qibuild pybind11
# # # cleanup space first
RUN rm -rf /tmp/gentoo/var/cache/binpkgs/* /tmp/gentoo/var/tmp/* /home/nao/.cache/* /home/nao/gentoo/var/cache/distfiles/*
RUN cd / && rm -rf `find . -type d -name *.cache`
RUN cd / && rm -rf `find . -type d -name __pycache__`
# Optional: remove all .git directories (no update possible after that)
# RUN find /home/nao -name ".git" -type d -exec rm -rf {} \; || true
RUN rm -rf /opt
RUN du -sh /tmp/gentoo
USER nao

SHELL ["/tmp/gentoo/executeonprefix"]

# for faster compression in parallel with pxz
RUN emerge app-arch/pxz app-arch/tar

RUN cd /home/nao && tar -I pxz  -c -f ./pepper_os.tar.lzma -C /home/nao gentoo -C  /home/nao ros2_humble -C /home/nao .local -C /home/nao .bash_profile -C /home/nao naoqi -C /home/nao catkin_ros2 || true

ENTRYPOINT ["/bin/bash"]