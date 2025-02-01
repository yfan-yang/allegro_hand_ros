# Allegro Hand ROS

This repo contains official ROS package for Allegro Hand, as well as python bindings to control Allegro Hand via ROS 1.

You can find our refered repos below:

- [GitHub: simlabrobotics/allegro_hand_ros_v4](https://github.com/simlabrobotics/allegro_hand_ros_v4)

- [GitHub: felixduvallet/allegro-hand-ros](https://github.com/felixduvallet/allegro-hand-ros)

Also, the links below are useful:

- [Allegro Hand wiki](http://www.simlab.co.kr/AllegroHand/wiki)

- [ROS wiki for original package](https://wiki.ros.org/allegro_hand_ros)

## Installation

This installation process is refered to:

- [GitHub: felixduvallet/allegro-hand-ros](https://github.com/felixduvallet/allegro-hand-ros)

- [AH V4 ROS Project | Allegro Hand](https://www.allegrohand.com/ah-v4-ros-project)

Currently, only **ROS 1** is supported (controlling via ROS 2 will be supported as soon as possible). The steps below are verified on Ubuntu 20.04 with ROS noetic installed, and may also useable for other Linux distros.

If you are using conda environment, you are recommened to stay in current environment when installing.

1. Install PCAN driver:

    - Install neccessary packages:

        ```shell
        sudo apt-get instal libpopt-dev ros-<$ROS_DISTRO>-libpcan
        ```

    - Install PCAN driver from [PCAN-Basic for Linux](https://www.peak-system.com/fileadmin/media/linux/can-pcan-basic.php), the latest version (currently version=4.9.0.7) is okay:

        ```shell
        tar xzvf PCAN-Basic_Linux-<version>
        cd PCAN-Basic_Linux-<version>/libpcanbasic/
        make clean
        make NET=NO_NETDEV_SUPPORT
        sudo make install
        sudo /sbin/modprobe pcan
        ```

        which will build and install `libpcanbasic.so` and `pcaninfo` for CAN connection.

    - You can test installation using:

        ```shell
        cat /proc/pcan
        ```

        If success, it will print information like:

        ```txt
        *------------- PEAK-System CAN interfaces (www.peak-system.com) -------------
        *------------- Release_20240521_n (8.18.0) Jan 15 2025 09:15:43 --------------
        *---------------------- [mod] [isa] [pci] [pec] [usb] -----------------------
        *--------------------- 1 interfaces @ major 506 found -----------------------
        *n -type- -ndev- --base-- irq --btr- --read-- --write- --irqs-- -errors- status
        32    usb   -NA- ffffffff 000 0x0014 00504d47 00504d3e 003c4616 00000012 0x0000
        ```

        else will print:

        ```txt
        cat: /proc/pcan: No such file or directory
        ```

    - As the Allegro Hand is connected, you should see pcanusb0 or pcanusb1 in the list of available interfaces:

        ```shell
        ls -l /dev/pcan*
        ```

        it will list like:

        ```txt
        lrwxrwxrwx 1 root root       9 Jan 16 02:07 /dev/pcan32 -> pcanusb32
        crw-rw-rw- 1 root root 506, 32 Jan 16 02:07 /dev/pcanusb32

        /dev/pcan-usb:
        total 0
        drwxr-xr-x 2 root root 60 Jan 16 02:07 0
      ```

    - If you do not see any available files, you may need to run:

        ```shell
        sudo ./driver/pcan_make_devices 2
        ```

      from the downloaded pcan folder: this theoretically creates the devices files if the system has not done it automatically.

1. Build libBHand

    - As libBHand is a pre-built library, we offer bhand in `./bhand`
    - Also, you can download libBHand from [AH V4 Grasping Library for Linux | Allegro Hand](https://www.allegrohand.com/ah-v4-grasping-library-for-linux). You should select corresponding version of hand (Take Allegro Hand V4 as example, you should download from `./"Allegro Hand V4 | File"/libBHand_64.zip`).

    - Install libBHand follwing the commands below:

      ```shell
      unzip LibBHand_64.zip
      cd libBHand_64
      sudo make install
      sudo ldconfig
      ```

1. Build ROS packages

    - If you're using conda environment, you may need to install several additional python packages such as `rospy`.

    - Before building, you should change the symlink of `./CMakeLists.txt`:

      ```shell
      ln -sf ${ROS_ROOT}/../catkin/cmake/toplevel.cmake CMakeLists.txt
      ```

    - Build packages:

      ```shell
      cd allegro_hand_ros
      catkin build
      ```

      You are recommended to remove directories including `./build` and `./devel` to force building from scratch.

    - Once installed, you can simply lauch rviz using the follwing command:

      ```shell
      source /devel/setup.bash
      roslaunch allegro_hand allegro_hand.launch HAND:=right
      ```

1. Install python package (optional)

    - **Note**: The python packages is based on previously built ROS packages, which is used for advanced control using python scripts. If ROS is already enough for controlling, there is no need for install this package.

    - Install package:

      ```shell
      cd allegro_hand_ros/src/allegro_hand
      python seup.py install -e
      ```

      Make sure the environment you are installing is identical to previous environment. You can check whether the package is successfully installed:

      ```shell
      python
      >>> import allegro_hand
      >>> allegro_hand.__file__
      # will echo
      # 'path/to/allegro_hand_ros/src/allegro_hand/scripts/allegro_hand/__init__.py'
      ```

      If no error message is echoed, then you have finished a success installation.

## Included Packages

1. **allegro_hand**: A python client that enables direct control of the hand in python code, and all generic launch files.

1. **allegro_hand_driver**: Driver for talking with the allegro hand.

1. **allegro_hand_controllers**: Different nodes that actually control the hand. The AllegroNode class handles all the generic driver comms, each class then implements computeDesiredTorque differently (and can have various topic subscribers):

    - **grasp**: Apply various pre-defined grasps, including gravity compensation.

    - **pd**: Joint space control: save and hold positions.

    - **velsat**: velocity saturation joint space control (supposedly experimental)

    - **torque**: Direct torque control.

    - **sim**: Just pass desired joint states through as current joint states.

1. **allegro_hand_description**: Xacro descriptions for the kinematics of the hand, rviz configuration and meshes.

1. **allegro_hand_keyboard**: Node that sends the commanded grasps. All commands are available with the grasp controller, only some are available with the other controllers.

1. **allegro_hand_parameters**: All necessary parameters for loading the hand:

    - **gains_pd.yaml**: Controller gains for PD controller.

    - **gains_velSat.yaml**: Controller gains and parameters for velocity saturation controller.

    - **initial_position.yaml**: Home position for the hand.

    - **zero.yaml**: Offset and servo directions for each of the 16 joints, and some meta information about the hand.

    - **zero_files**: Zero files for all hands.

1. **bhand**: Library files for the predefined grasps, available in 32 and 64 bit versions. 64 bit by default, update symlink for 32 bit.
Note on polling (from SimLabs): The preferred sampling method is utilizing the Hand's own real time clock running @ 333Hz by polling the CAN communication (polling = true, default). In fact, ROS's interrupt/sleep combination might cause instability in CAN communication resulting unstable hand motions.

## Usage

A simple way to start Allegro Hand is using:

```shell
source /path/to/allegro_hand_ros/devel/setup.bash
roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right
```

Also, you can set `HAND:=left` to control left hand.

Bafore executing, you should make sure PCAN-USB is connected and Allegro hand is powered on.

You should see an Allegro Hand in rviz, whose pose is identical to the real Allegro Hand. In addition, you can control the hand using keyboard follwing the showing instructions.
