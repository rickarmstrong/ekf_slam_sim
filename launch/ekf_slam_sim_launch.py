import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base_dir = get_package_share_directory('ekf_slam_sim')
    launch_dir = os.path.join(base_dir, 'launch')
    include_dir = os.path.join(launch_dir, 'include')

    # Gazebo sim with our rover and a bunch of AprilTags in the depot world.
    tb4_gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'tb4_simulation_launch.py')),
        launch_arguments={
            'headless': 'False',
            'use_sim_time': 'True',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(tb4_gazebo_sim)
    return ld
