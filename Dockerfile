FROM usdotfhwastoldev/navigation2:nav2_route_tool-humble AS nav2
FROM usdotfhwastoldev/navigation2-extensions:develop AS nav2_ext
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-ackermann-msgs \
    ros-humble-joy \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-turtlebot3-gazebo \
    libnanoflann-dev \
    nlohmann-json3-dev \
    libmosquitto1 \
    libmosquitto-dev \
    liblttng-ust-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=nav2 /opt/underlay_ws /opt/underlay_ws
COPY --from=nav2 /opt/overlay_ws /opt/overlay_ws
COPY --from=nav2_ext /root/cda_ws /opt/nav2_ext_ws

COPY . /opt/bringup_ws/

WORKDIR /opt/bringup_ws

RUN sed -i 's|/home/.*/cda1tenth-bringup|/opt/bringup_ws/src/cda1tenth-bringup|g' \
    /opt/bringup_ws/src/cda1tenth-bringup/params/turtlebot_params.yaml && \
    sed -i 's|maps/garage.yaml|maps/turtlebot_sim.yaml|g' \
    /opt/bringup_ws/src/cda1tenth-bringup/launch/localization_launch.xml

RUN sed -i -E \
    -e '/<include.*(v2x_ros_driver|cpp_message|j2735_convertor)/d' \
    -e 's/(cpp_message_node|v2x_ros_driver_node|j2735_convertor_node)(, )?//g' \
    /opt/bringup_ws/src/cda1tenth-bringup/launch/cda1tenth_bringup_launch.xml

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /opt/underlay_ws/install/setup.bash && \
    source /opt/overlay_ws/install/setup.bash && \
    source /opt/nav2_ext_ws/install/setup.bash && \
    colcon build"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]