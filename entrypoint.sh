#!/bin/bash
set -e

source /opt/ros/humble/setup.bash          
source /opt/underlay_ws/install/setup.bash 
source /opt/overlay_ws/install/setup.bash  
source /opt/nav2_ext_ws/install/setup.bash 
source /opt/carma/install/setup.bash       
source /opt/v2x_ws/install/setup.bash       
source /opt/carma_platform/install/setup.bash 
source /opt/bringup_ws/install/setup.bash 

exec "$@"
