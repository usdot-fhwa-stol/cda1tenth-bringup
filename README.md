# c1t_bringup

This package contains the launch and configuration files needed to bring up a physical or simulated C1T vehicle. Some functionality may depend on the companion packages from [navigation2_extensions](https://github.com/usdot-fhwa-stol/navigation2_extensions) which provides route server and CDA additions to the C1T system.

## Prerequisites

The following configuration steps must be completed before a C1T vehicle or simulation will be ready for bringup.

1. Download the `nav2_route_server` branch and build it from source, which can be found [here](https://github.com/usdot-fhwa-stol/navigation2/tree/nav2_route_server).

2. Install the following udev rules to the `/etc/udev/rules.d/` directory if using a physical vehicle:
    * `99-joypad-f710.rules`
    * `99-vesc6.rules`
    * `rplidar.rules`

    > [NOTE]
    > See [1. udev Rules Setup](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#udev-rules-setup) in the F1Tenth build documentation for details.

3. Source the Nav2 install for future terminal instances with `echo 'source /home/$USER/c1t_ws/install/setup.bash' >> ~/.bashrc`

4. A map and route graph will need to be created for physical test environments. The `turtlebot` configuration includes these for reference. Maps can be created using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) and graphs should follow the examples given in this package. New maps and graphs will need to manually updated in the launch and parameter files.


## Launch

Run the following command to launch the C1T system:

```
$ ros2 launch c1t_bringup c1t_bringup_launch.xml vehicle:=[red_truck, blue_truck, turtlebot] record_bag:=[true, false]
```

This will launch all necessary subsystems, such as drivers, localization, and navigation.

### Launch Arguments
- `vehicle` controls the parameter file used to launch the system and requires an input

  - `red_truck` and `blue_truck` target parameter files specific to their respective C1T vehicles

  - `turtlebot` runs a Gazebo-based simulation that allows for running core `c1t_bringup` packages locally without hardware interface nodes

- `record_bag` is a boolean argument for recording all data from the system in a ROS2 rosbag and defaults to `false`. Bags are saved in `~/c1t_bags` along with a copy of the parameter file used


## Post launch

After launching the nodes included in `c1t_bringup`, additional actions are needed to finalize the system on a physical vehicle.

1. Send an empty message to the `/ackermann_cmd` topic to initialize the VESC by publishing via command line on the vehicle with `ros2 pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped`. Use `CTRL + C` after a few have sent and the original terminal output has updated.

   > [NOTE]
   > The `vesc_to_odom_node` ROS node is responsible for publishing the odometry information it receives from the VESC motor controller. However, it does not start publishing this information until the VESC driver stack receives a command.

2. If using a physical vehicle, RViz will need to be launched on an external PC on the same network and `ROS_DOMAIN_ID` as the vehicle. A specific RViz configuration is needed for visualizing and interacting with Nav2, which can be found in this package under the `rviz` directory. Run RViz in a terminal using the command `rviz2` and open this file using `File > Open Config`.

3. In RViz, set the vehicle's initial pose estimate by selecting the `2D Pose Estimate` button at the top and drawing the approximate pose of the vehicle with respect to the map. The ROS2 nodes being used by the vehicle should be visualized after the estimate is drawn.

4. A navigation command can now be sent to the C1T vehicle by using the `2D Goal Pose` button to draw a goal pose on the map using the RViz GUI.


### Shutdown 
To shut down the system, use `CTRL + C` on the `c1t_bringup` and `rviz2` terminals. Run `ros2 node list` to verify all nodes are shut down before relaunching the system.

# CARMAPlatform
The primary CARMAPlatform repository can be found [here](https://github.com/usdot-fhwa-stol/CARMAPlatform) and is part of the [USDOT FHWA STOL](https://github.com/usdot-fhwa-stol/)
github organization. Documentation on how the CARMAPlatform functions, how it will evolve over time, and how you can contribute can be found at the above links as well

## Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/Contributing.md) 

## Code of Conduct 
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/ATTRIBUTION.md) 

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/docs/License.md) 

## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website. For more information, contact CAVSupportServices@dot.gov.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/CARMAPlatform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
