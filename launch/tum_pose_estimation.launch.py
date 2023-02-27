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

def generate_launch_description():
    included_launch = []
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
                "twist_smoothing_steps": "2",
                "input_initial_pose_name": "/initialpose3d",
                "input_pose_with_cov_name": "/localization/pose_estimator/pose_with_covariance",
                "input_twist_with_cov_name": "/localization/twist_estimator/twist_with_covariance",
                "output_odom_name": "kinematic_state",
                "output_pose_name": "pose", 
                "output_pose_with_covariance_name": "/localization/pose_with_covariance",
                "output_biased_pose_name": "biased_pose",
                "output_biased_pose_with_covariance_name": "biased_pose_with_covariance",
                "output_twist_name": "twist",
                "output_twist_with_covariance_name": "twist_with_covariance",
                "proc_stddev_vx_c": "10.0",
                "proc_stddev_wz_c": "5.0",
            }.items()
        )
    )
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("stop_filter"),
                           "launch/stop_filter.launch.xml")
            ]),
            launch_arguments={
                "use_twist_with_covariance": "True",
                "input_odom_name": "localization/pose_twist_fusion_filter/kinematic_state",
                "input_twist_with_covariance_name": "/localization/pose_twist_fusion_filter/twist_with_covariance",
                "output_odom_name": "localization/kinematic_state",
            }.items()
        )
    )
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("twist2accel"),
                           "launch/twist2accel.launch.xml")
            ]),
            launch_arguments={
                "use_odom": "true",
                "in_odom": "/localization/kinematic_state",
                "in_twist": "/localization/twist_estimator/twist_with_covariance",
                "out_accel": "/localization/acceleration",
            }.items()
        )
    )
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("pose_initializer"),
                           "launch/pose_initializer.launch.xml")
            ]),
            launch_arguments={
                "gnss_enabled": "true",
                "ndt_enabled": "true",
                "stop_check_enabled": "false",
            }.items()
        )
    )
    included_launch.append(
        IncludeLaunchDescription
        (
            XMLLaunchDescriptionSource
            ([
               os.path.join(get_package_share_directory("automatic_pose_initializer"),
                           "launch/automatic_pose_initializer.launch.xml")
            ])
        )
    )

    #included_launch.append(IncludeLaunchDescription(file='$(find-pkg-share stop_filter)/launch/stop_filter.launch.xml', arguments=[arguments_launch_stop]))
    #included_launch.append(IncludeLaunchDescription(file='$(find-pkg-share twist2accel)/launch/twist2accel.launch.xml', arguments=[arguments_launch_acc]))

    #declared_arguments = []
    #declared_arguments.append(DeclareLaunchArgument(""))
    #declared_arguments.append(DeclareLaunchArgument("output_location", default_value="~/ekf_debug.txt"))    

    #executed_processes = []
    #executed_processes.append(ExecuteProcess(cmd=["ros2", "bag", "play", "/home/eivis/Downloads/Austausch_Eivydas/novatel_bag/rosbag2_2022_12_15-10_29_57"], output="screen"))  
    #executed_processes.append(ExecuteProcess(cmd=["ros2", "bag", "play", "/home/eivis/Documents/MA_Keras/rosbags/utbm_topics/filtered_tops/rosbag2_2022_12_16-22_35_24"], output="screen")) 
 #+ declared_arguments + executed_processes) #+ [OpaqueFunction(function=launch_setup)])

    return LaunchDescription(included_launch)