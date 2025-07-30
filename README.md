# ROS2 PX4 SITL Gazebo OptiTrack Emulator
## A ROS2 package served as the OptiTrack Emulator
- Forwards the Gazebo message to ROS2 mocap message.
- Use gz_transport to facilitate Gazebo message subscription.

## How to obtain the ground truth from PX4 SITL ROS2 Gazebo simulion
- Read the following [article](docs/px4_sitl_groundtruth.md) to learn how to publish simulation groundtruth.

## How to use the package
### Pull the source code to the src directory of your workspace
- ``git clone https://github.com/FSC-Lab/gz_optitrack_ros2_emulator.git``
### Build the node
- Use ``colcon build --packages-select gz_optitrack_ros2_emulator`` to build the node.
- Source ``<work_space_name>`` by adding the following to the ``~/.bashrc`` or ``~/.zhrc``: 
``source ~/path_to_work_space/<work_space_name>/install/local_setup.bash``

### Run the node
- Use ``ros2 launch gz_optitrack_ros2_emulator emulator_for_gazebo_launch.py`` to launch the node.

## How to change the model name
- The model names are listed in the ``config/params.yaml`` under the name of ``gz_model_list``.