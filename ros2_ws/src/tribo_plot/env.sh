#!/bin/bash

# conda activate tribo
# colcon build --packages-select tribo_plot
source ./install/setup.bash

# Check conda Python
export CONDA_SITE=$(python -c "import site; print(site.getsitepackages()[0])")
# Add it to PYTHONPATH along with your ROS2 workspace and your source folder
export PYTHONPATH=$PYTHONPATH:$CONDA_SITE
export PYTHONPATH=$PYTHONPATH:$HOME/Documents/tribo-slide/ros2_ws/src/tribo_plot

## Run your node
# ros2 run tribo_plot app1

# ros2 run tribo_plot sensorT_fake
# ros2 run tribo_plot app_node
# ros2 run tribo_plot inference
# ros2 run tribo_plot state_manager