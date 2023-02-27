import os
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    ekf_initialization_node = Node(
        package="tum_state_estimation_sensor_fusion",
        executable="pose_initializer_client",
        name="ekf_initializer",
        output="screen"
    )

    nodes_to_start = [
        ekf_initialization_node
    ]

    return nodes_to_start

def generate_launch_description():
    included_launch = []
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("gnss_poser"),
                           "launch/gnss_poser.launch.xml")
            ]),
            launch_arguments={
                "input_topic_fix": "/sensing/gnss/ublox/nav_sat_fix",
                "output_topic_gnss_pose": "/sensing/gnss/pose",
                "output_topic_gnss_pose_cov": "/sensing/gnss/pose_with_covariance",
                "output_topic_gnss_fixed": "/sensing/gnss/fixed",
                "coordinate_system": "1",
                "use_gnss_ins_orientation": "true",
                "gnss_frame": "gnss_link",
            }.items()
        )
    )
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("tier4_vehicle_launch"),
                           "launch/vehicle.launch.xml")
            ]),
            launch_arguments={
                "vehicle_model": "sample_vehicle",
                "sensor_model": "sample_sensor_kit",
                "vehicle_id": "$(env VEHICLE_ID default)",
                "launch_vehicle_interface": "false",
            }.items()
        )
    )
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("ekf_localizer"),
                           "launch/ekf_localizer.launch.xml")
            ]),
            launch_arguments={
                "enable_yaw_bias_estimation": "true",
                "tf_rate": "50.0",
                "input_trigger_node_service_name": "/localization/twist_estimator/ekf_trigger_node_mod",
                "twist_smoothing_steps": "2",
                "input_initial_pose_name": "/initialpose3d_pub",
                "input_pose_with_cov_name": "/sensing/gnss/pose_with_covariance",
                "input_twist_with_cov_name": "/localization/twist_estimator/twist_with_covariance",
                "output_odom_name": "/localization/pose_twist_fusion_filter/kinematic_state",
                "output_pose_name": "/localization/pose_twist_fusion_filter/pose", 
                "output_pose_with_covariance_name": "/localization/pose_with_covariance",
                "output_biased_pose_name": "/localization/pose_twist_fusion_filter/biased_pose",
                "output_biased_pose_with_covariance_name": "/localization/pose_twist_fusion_filter/biased_pose_with_covariance",
                "output_twist_name": "/localization/pose_twist_fusion_filter/twist",
                "output_twist_with_covariance_name": "/localization/pose_twist_fusion_filter/twist_with_covariance",
                "proc_stddev_vx_c": "10.0",
                "proc_stddev_wz_c": "5.0",
            }.items()
        )
    )

    #included_launch.append(IncludeLaunchDescription(file='$(find-pkg-share stop_filter)/launch/stop_filter.launch.xml', arguments=[arguments_launch_stop]))
    #included_launch.append(IncludeLaunchDescription(file='$(find-pkg-share twist2accel)/launch/twist2accel.launch.xml', arguments=[arguments_launch_acc]))

    #declared_arguments = []
    #declared_arguments.append(DeclareLaunchArgument(""))
    #declared_arguments.append(DeclareLaunchArgument("output_location", default_value="~/ekf_debug.txt"))    

    executed_processes = []
    executed_processes.append(ExecuteProcess(cmd=["ros2", "bag", "play", "/home/eivis/autoware_map/sample-rosbag/sample.db3", "-r", "0.2", "-s", "sqlite3"], output="screen"))  
    #executed_processes.append(ExecuteProcess(cmd=["ros2", "bag", "play", "/home/eivis/Documents/MA_Keras/rosbags/utbm_topics/filtered_tops/rosbag2_2022_12_16-22_35_24"], output="screen")) 
 #+ declared_arguments + executed_processes) #+ [OpaqueFunction(function=launch_setup)])

    return LaunchDescription(included_launch + executed_processes + [OpaqueFunction(function=launch_setup)])