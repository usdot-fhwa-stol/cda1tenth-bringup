# CDA 1Tenth Bringup

## Introduction

Cooperative driving automation (CDA) supports and enables communication between vehicles
with driving automation features, other road users, and transportation infrastructure. Once
deployed, CDA has the potential to improve transportation efciency, facilitate freight
movement, increase productivity, and save billions by reducing the need to increase roadway
facilities

CDA 1Tenth is an open source platform for testing CDA capabilities without life size vehicles. Whether you are working with a small [physical robotic vehicle](https://github.com/usdot-fhwa-stol/cda1tenth-hardware) (Go to [Physical](#physical-robot)) in a garage or running a simulated vehicle on your computer (Go to [Simulated](#simulated-robot)), this guide will help you get started.

If you are new to this project and some of the technologies, check out the [key terms section](#key-terms)

## Simulated Robot

This guide details the steps to set up, configure, and launch the CDA 1Tenth simulated environment using Docker. This code has been tested on WSL2, Ubuntu 22.04 LTS Desktop, and VM.

### Pre-reqs
- Docker
- Nvidia Container Toolkit (On Ubuntu Desktop)
- Git

### 1. Workspace Setup

Create a new workspace, source directory, and clone the `cda-bringup` repository:

```bash
mkdir -p ~/cda_ws/src
cd ~/cda_ws/src
git clone https://github.com/usdot-fhwa-stol/cda1tenth-bringup.git
```

Ensure your `docker-compose.yaml`, `Dockerfile`, and `entrypoint.sh` are placed in the root of the workspace (`~/cda_ws/`). They will be cloned under the cda1tenth-bringup folder, so do this command.

```bash
cd ~/cda_ws/src/cda1tenth-bringup
mv Dockerfile docker-compose.yml docker-compose.nvidia.yml entrypoint.sh launch.sh ../..
```

### 2. Build and Launch

launch.sh will handle building and launching for you. First create the executable.

```bash
chmod +x launch.sh
```

You'll have different options based on your environment.

```bash
# This should auto-detect your env settings
./launch.sh

# But you can manually change the gpu mode
./launch.sh --gpu
./launch.sh --no-gpu

# And to change launch params, Ex:
VEHICLE=tortoisebot RECORD_BAG=true ./launch.sh
```

*Note: RViz and Gazebo may take a moment to launch and load the maps.*

If rviz is not launching/crashing/not displaying properly try these commands to increase docker permissions.

```bash
export LIBGL_ALWAYS_SOFTWARE=1
xhost +local:docker
```

### 3. Initialization and Operation

#### A. Set Initial Pose

1. In RViz, set the vehicle's initial pose estimate by selecting the **2D Pose Estimate** button at the top.
2. Draw the approximate pose of the vehicle with respect to the map. The ROS 2 Nav2 nodes should become active and display the costmaps after the estimate is drawn.

#### B. Send Port Drayage Mobility Operation

Open a separate terminal to send a simulated V2X mobility message to the vehicle.

Enter the container and source the environment:

```bash
docker exec -it cda_ws-cda1tenth-1 bash
source /opt/bringup_ws/install/setup.bash
```

Send the operation message:

```bash
ros2 topic pub --once /incoming_mobility_operation carma_v2x_msgs/msg/MobilityOperation "{m_header: {sender_id: '', recipient_id: '', sender_bsm_id: '', plan_id: '', timestamp: 0}, strategy: 'carma/port_drayage', strategy_params: '{\"cmv_id\":\"turtlebot\",\"operation\":\"ENTER_PORT\",\"cargo\":false,\"cargo_id\":\"SOME_CARGO\",\"destination\":{\"longitude\":\"-1.6\",\"latitude\":\"-0.2\"},\"action_id\":\"\"}'}"
```

#### Shutdown

To shut down the system, use `CTRL + C` on the `cda1tenth_bringup` and `rviz2` terminals. Run `ros2 node list` to verify all nodes are shut down before relaunching the system.

## Physical Robot

## Prerequisites and Setup

Before you can start the vehicle, your computer environment needs to be prepared.

1. Download and build the custom route server. You can find the required `nav2_route_server` branch [nav2_route_server](https://github.com/usdot-fhwa-stol/navigation2/tree/nav2_route_server).

----------------------------------------------------

1. **udev** Rules Setup

If you are using a physical vehicle, you must install three `udev` rules so your computer can consistently identify and communicate with the VESC, lidar, and joypad.

When the VESC and USB lidar are connected, Linux may assign them device names such as `/dev/ttyACM0` or `/dev/ttyACM1`. These names can change depending on the order the devices are plugged in or initialized during boot. This causes problems because the system configuration needs stable device paths. `udev` solves this by assigning persistent names based on each device’s vendor and product IDs.

Create the following three rules files in `/etc/udev/rules.d/`:

- `99-joypad-f710.rules`
- `99-vesc6.rules`
- `rplidar.rules`

As root, create `/etc/udev/rules.d/rplidar.rules` and paste in the rule for the lidar on a single line:

```bash
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/rplidar"
```

Next, create `/etc/udev/rules.d/99-vesc6.rules` and paste in the rule for the VESC:

```bash
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"
```

Then create `/etc/udev/rules.d/99-joypad-f710.rules` and paste in the rule for the joypad:

```bash
KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"
```

Reload and activate the rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reboot your system, then verify the devices were created:

```bash
ls /dev/sensors
ls /dev/input
```

If you need to identify the vendor or product IDs for a device, run:

```bash
sudo udevadm info --name=<your_device_name> --attribute-walk
```

Replace `<your_device_name>` with the device assigned by the OS, such as `ttyACM0`.

For more information, see the [F1TENTH Firmware Documentation](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#udev-rules-setup).

-------------------------------------------------

1. To ensure your terminal always recognizes the CDA 1Tenth software, run this command to update your system profile:

```bash
echo 'source /home/$USER/cda_ws/src/install/setup.bash' >> ~/.bashrc
```

--------------------------------------------------------------------------

1. The vehicle needs a map to know where it is driving. A sample map is included under the turtlebot configuration. If you create a new map of your lab using a tool like [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), you will need to manually update the configuration files in this repository to point to your new map.

## Launching the System

Run the following command in your terminal to start the system.

```bash
ros2 launch cda1tenth_bringup cda1tenth_bringup_launch.xml vehicle:=[red_truck, blue_truck, turtlebot] record_bag:=[true, false]
```

**Launch Arguments:**

- **vehicle:** Choose one option from the brackets. If you are using a physical car, choose `red_truck` or `blue_truck`. If you want to run a virtual test on your computer without hardware, choose `turtlebot`.
- **record_bag:** Choose `true` if you want to record the vehicle data for later review, or `false` to ignore. Records are saved in a folder named cda_bags.

## Post Launch Steps

If you are using a physical vehicle, you will use the RViz visualizer to provide the initial location and destination.

1. On an external computer connected to the same network as the vehicle, open a new terminal and type `rviz2`.
2. In the RViz program, go to File then Open Config and select the configuration file located in the rviz directory of this repository.
3. Click the 2D Pose Estimate button at the top of the screen. Click and drag on the map to show the software exactly where the car is currently sitting. You should see the vehicle sensor data on the screen.
4. Click the 2D Goal Pose button at the top of the screen. Click and drag on the map to tell the car where it should drive.

### Shutting Down

To safely turn off the vehicle software, go to the terminal where you launched the system and press CTRL and C. You can type `ros2 node list` in the terminal to verify that all systems have shut down completely.

### Key Terms

- **Bringup:** The process of launching and connecting all the software required to make the robot operate.
- **[ROS 2](https://github.com/ros2):** Robot Operating System. The underlying framework that allows all the different parts of the vehicle to communicate.
- **[RViz](https://github.com/ros-visualization/rviz):** A visualizer and interface for the ROS framework.
- **[Gazebo](https://github.com/gazebosim):** An open-source, 3D robotics simulator.
- **Navigation2 (Nav2):** A navigation stack for mobile robots built on the ROS framework [Nav2 Github](https://github.com/ros-navigation/navigation2).
- **.xml:** In ROS 2, these files are used as launch scripts to define exactly which software nodes to start and how they should connect.
- **.yaml:** A format used for configuration files. These files store settings and parameters in a clean layout that is easy to read and edit.
- **.pgm:** Portable Graymap Format. A simple image file used by the navigation system to store a 2D grid map of the physical environment.
- **.urdf:** Unified Robot Description Format. A file that describes the physical dimensions, joints, and visual appearance of the robot so the software knows how the vehicle is structured and how it moves.
- **.rules:** Linux configuration files. They tell the computer operating system how to handle specific hardware devices like sensors or controllers when they are plugged in, ensuring the software has the correct permissions to use them.

## Contribution

Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md)

## Code of Conduct

Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution

The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/ATTRIBUTION.txt)

## License

By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/docs/License.md)

## Contact

Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
