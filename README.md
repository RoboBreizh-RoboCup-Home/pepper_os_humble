# pepper_os_humble

Docker with ROS2 Humble and naoqi_driver2 for Pepper 2.5.

- [pepper\_os\_humble](#pepper_os_humble)
  - [Recommendation](#recommendation)
  - [Release](#release)
  - [Generate](#generate)
  - [Compress to lzma](#compress-to-lzma)
  - [Installation on pepper](#installation-on-pepper)
  - [Test ROS2](#test-ros2)
  - [Test naoqi\_driver2](#test-naoqi_driver2)
    - [Start naoqi\_driver2](#start-naoqi_driver2)
    - [Test naoqi\_driver2](#test-naoqi_driver2-1)
  - [Future works](#future-works)

## Recommendation

It is recommended to set Pepper's IP with:

```bash
$ export PEPPER_IP=x.x.x.x #(i.e 169.254.0.0)
```

in the .bash_profile of Pepper or on any computer that will be used to communicate with Pepper.

For clarity and simplification, the documentation assumes that the variable is set.

## Release 

The compressed environement can be downloaded here [here](https://mycore.core-cloud.net/index.php/s/oqqD1khgUeuQCmR) (4.4GB). From there go to [Installation on pepper](#installation-on-pepper) 

## Generate

The Dockerfile requires pepper_os_11.0.4.tar.lzma in the same directory. 

```bash
$ wget --no-check-certificate -O pepper_os_11.0.4.tar.lzma "https://mycore.core-cloud.net/index.php/s/Rh4EbGqxc05W3ap/download?path=%2F&files=pepper_os_11.0.4.tar.lzma"
$ docker build -f Dockerfile -t pepper_os_humble .
```

Alternatively, the first command is present but commented at the top of the Dockerfile. Uncommenting it is possible but not advised. Downloading it is this way is considerably slower this way and may take 2~3 hours.

## Compress to lzma

Compressing the environment to lzma is necessary as Pepper has limited resources. The entire environemnt is about ~15GB and will be reduced to about ~4.3GB. In comparison, Pepper has about 25GB of space available.

```bash
$ docker run -it pepper_os_humble:latest
$ cd /home/nao; tar -c --lzma -f /tmp/pepper_os_humble.tar.lzma -C /home/nao gentoo -C  /home/nao asio-1.28.0 -C /home/nao ros2_humble -C /home/nao .local -C /home/nao .bash_profile -C /home/nao naoqi -C /home/nao catkin_ros2 || true

#Outside docker in another terminal
$ docker cp CONTAINER_ID:/tmp/pepper_os_humble.tar.lzma ./
```

CONTAINER_ID being the ID of the running container (nao@CONTAINER_ID)

Compressing the environement takes about an hour and a half.

## Installation on pepper

First, check whether there is enough space available on Pepper to install the archive.

```bash
$ ssh nao@$PEPPER_IP
$ df -t ext3
```

The arhice requires ~19.5GB to be copied and uncompressed.

If there isn't enough space, remove the prior pepper_os implementations or delete unnecessary files.

For example:
```bash
rm -rf gentoo asio-1.28.0 ros2_humble catkin_ros2 pepper_os.tar.lzma
```

Once there is enough space available, the archive can be copied and uncompressed safely. 

```bash
$ scp pepper_os.tar.lzma nao@$PEPPER_IP:/home/nao/
$ ssh nao@$PEPPER_IP
$ tar --lzma -xvf ./pepper_os_humble.tar.lzma
$ rm pepper_os_humble.tar.lzma
```

## Test ROS2

ROS2 can be tested with the following commands:

```bash
$ ssh nao@$PEPPER_IP
$ ros2 run demo_nodes_cpp talker
#The terminal should display: 'Publishing: "Hello world: x"'
```

![alt text](assets/ros2_talker.png)

2nd terminal
```bash
$ ssh nao@$PEPPER_IP
$ ros2 run demo_nodes_cpp listener
#The terminal should display: 'I heard: [Hello world: x]'
```

![alt text](assets/ros2_listener.png)

The python equivalent can be tested with demo_nodes_py instead of demo_nodes_cpp.

## Test naoqi_driver2

### Start naoqi_driver2

```bash
$ ssh nao@$PEPPER_IP
$ ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=$PEPPER_IP network_interface:=MACHINE_INTERFACE
```

MACHINE_INTERFACE should be 'wlan0' if Pepper is on wifi or 'eth0' if Pepper connected directly via an ethernet cable.

Alternatively, an alias has been made in .bash_profile

```bash
$ nao_driver
```

The ip and network interfaces will be automatically fetched. 

![alt text](assets/ros2_naoqi_driver_1.png)

### Test naoqi_driver2

```bash
$ ssh nao@$PEPPER_IP
$ ros2 topic list
```
Will return a list of all the topic available.

![alt text](assets/ros2_naoqi_driver_2.png)

```bash
#On a computer
$ source /opt/ros/humble/setup.bash
$ export ROS_MASTER_URI=http://$PEPPER_IP:11311
$ rviz2 rviz
```

To start rviz2 and display the topic of interest (Add -> By topic).

## Future works

* Currently, the dockerfile requires an older version of pepper_os_noetic to be built. This is to avoid having to configure the entire environement from scratch (boost, python libraries...). Building one from scratch would slim down the size of the archive and remove obsolete packages.
* Using python3.10 instead of python3.8. Python3.10 is now the default python implementation on gentoo. 