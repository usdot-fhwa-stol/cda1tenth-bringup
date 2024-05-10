# c1t_bringup

This package contains the launch and configuration files needed to bring up a C1T vehicle.

## Prerequisites

You must complete the following configuration steps before your C1T vehicle will be ready for bringup.

1. Install the `ros-humble-navigation2` package, which installs Nav2.
2. Install the following udev rules to the `/etc/udev/rules.d/` directory:
    * `99-joypad-f710.rules`
    * `99-vesc6.rules`
    * `rplidar.rules`

    > [!NOTE]
    > See [1. udev Rules Setup](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/drive_workspace.html#udev-rules-setup) in the F1Tenth build documentation for details.

## Launch

Run the following command to launch the C1T system:

```console
$ ros2 launch c1t_bringup c1t_bringup_launch.xml
```

This will launch all necessary subsystems, such as drivers, localization, and navigation.

## Post launch

After launching the ROS nodes, you will to follow a specific sequence of actions before you can give the vehicle waypoint.

1. Send an empty message to the `/ackermann_cmd` topic. You can use Foxglove Studio's Publish panel for this or send it directly to the vehicle by publishing via command line with `ros2 pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped`.

   > [!NOTE]
   > The `vesc_to_odom_node` ROS node is responsible for publishing the odometry information it receives from the VESC motor controller. However, it does not start publishing this information until the VESC driver stack receives a command.

2. Using RViz, send a 2D Pose Estimate to initialize the localization system. You will need to load the Nav2 RViz configuration for this, which you can find in `/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz`.

3. You can now command the C1T to drive to arbitrary waypoints by sending `Nav2 Goal` actions.
