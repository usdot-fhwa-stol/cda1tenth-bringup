FROM usdotfhwastoldev/navigation2:nav2_route_tool-humble AS nav2
FROM usdotfhwastoldev/navigation2-extensions:develop AS nav2_ext

FROM usdotfhwastoldev/v2x-ros-driver:develop AS v2x_ros_driver
FROM usdotfhwastoldev/v2x-ros-conversion:develop AS v2x_ros_conv
FROM usdotfhwastoldev/carma-msgs:develop AS carma_msgs

FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ackermann-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-turtlebot3-gazebo \
    libnanoflann-dev \
    nlohmann-json3-dev \
    libmosquitto1 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=nav2 /opt/underlay_ws /opt/underlay_ws
COPY --from=nav2 /opt/overlay_ws /opt/overlay_ws
COPY --from=nav2_ext /root/cda_ws /opt/nav2_ext_ws

COPY --from=carma_msgs /home/carma/.base-image/ros2_msgs_ws/install /home/carma/.base-image/ros2_msgs_ws/install
COPY --from=carma_msgs /home/carma/.base-image/workspace/install /home/carma/.base-image/workspace/install

COPY --from=v2x_ros_driver /opt/carma /opt/carma
COPY --from=v2x_ros_conv /opt/carma /opt/carma

RUN mkdir -p /opt/carma/vehicle/config && \
    echo "{}" > /opt/carma/vehicle/config/GlobalParamsOverride.yaml

COPY . /opt/bringup_ws/src/cda1tenth-bringup/
WORKDIR /opt/bringup_ws
RUN sed -i 's|<depend>cpp_message</depend>|<exec_depend>cpp_message</exec_depend>|g' /opt/bringup_ws/src/cda1tenth-bringup/package.xml && \
    sed -i 's|<depend>v2x_ros_driver</depend>|<exec_depend>v2x_ros_driver</exec_depend>|g' /opt/bringup_ws/src/cda1tenth-bringup/package.xml && \
    sed -i 's|<depend>j2735_convertor</depend>|<exec_depend>j2735_convertor</exec_depend>|g' /opt/bringup_ws/src/cda1tenth-bringup/package.xml
RUN sed -i 's|/home/.*/cda1tenth-bringup|/opt/bringup_ws/src/cda1tenth-bringup|g' \
    /opt/bringup_ws/src/cda1tenth-bringup/params/turtlebot_params.yaml && \
    sed -i 's|maps/garage.yaml|maps/turtlebot_sim.yaml|g' \
    /opt/bringup_ws/src/cda1tenth-bringup/launch/localization_launch.xml
    
RUN sed -i '/ros1_msgs_ws/d' /home/carma/.base-image/workspace/install/setup.bash

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /opt/underlay_ws/install/setup.bash && \
    source /opt/overlay_ws/install/setup.bash && \
    source /home/carma/.base-image/workspace/install/setup.bash && \
    source /home/carma/.base-image/ros2_msgs_ws/install/setup.bash && \
    source /opt/carma/install/setup.bash && \
    source /opt/nav2_ext_ws/install/setup.bash && \
    colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/underlay_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/overlay_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/carma/.base-image/workspace/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/carma/.base-image/ros2_msgs_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/carma/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/nav2_ext_ws/install/setup.bash" >> ~/.bashrc

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"
ARG URL="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="cda1tenth-bringup"
LABEL org.label-schema.description="cda1tenth-bringup docker image"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url=${URL}
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/cda1tenth-bringup"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}
    
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
