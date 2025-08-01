#!/bin/bash

# colcon build --packages-select tribo_plot
source ~/ros2_ws/install/setup.bash

# Check conda Python
export CONDA_SITE=$(python -c "import site; print(site.getsitepackages()[0])")
# Add it to PYTHONPATH along with your ROS2 workspace and your source folder
export PYTHONPATH=$PYTHONPATH:$CONDA_SITE
export PYTHONPATH=$PYTHONPATH:$HOME/ros2_ws/src/tribo_plot

## Run your node
# ros2 run tribo_plot app1