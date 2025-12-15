import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # --- Konfigürasyonlar ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Paket yollarını bulma
    config_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_config").find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_description").find("go2_description")
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    champ_gazebo_pkg = get_package_share_directory('champ_gazebo')

    # Dosya yolları
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_VLP.xacro")
    my_world_path = os.path.join(champ_gazebo_pkg, 'worlds', 'my_world.sdf') #os.path.join(champ_gazebo_pkg, 'worlds', 'my_world.sdf')
    
    # --- Argümanlar ---
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_world = DeclareLaunchArgument(
        "world", default_value=my_world_path, description="World file name"
    )

    # --- 1. Gazebo Fortress Başlatma ---
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world'), ' -v 4']}.items(),
    )

    # --- 2. CHAMP Bringup (Kontrol Algoritması) ---
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("champ_bringup"), "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": use_sim_time,
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "rviz": LaunchConfiguration("rviz"),
            # ÖNEMLİ: Champ'in komut göndereceği controller topic'i
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    # --- 3. Robotu Spawn Etme ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.6' # Yere gömülmemesi için hafif yukarıda
        ],
        output='screen'
    )

    # --- 4. Controller Spawners (Eksik Olan Parça Burası!) ---
    # Bu node'lar, ros2_control sistemini tetikler ve controllerları başlatır.
    
    # A) Joint State Broadcaster: Robotun anlık eklem açılarını okur ve yayınlar (/joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # B) Joint Group Effort Controller: Champ'ten gelen komutları motorlara iletir
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_effort_controller", "--controller-manager", "/controller_manager"],
    )

    # --- 5. ROS - Gazebo Bridge ---
    # Joint States artık ros2_control tarafından yayınlanacağı için bridge listesinden çıkardım.
    # Böylece çakışma (titreme) olmaz.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # Spawner'ları spawn işlemi bittikten sonraya erteleyelim ki hata vermesin
    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_name,
        declare_rviz,
        declare_world,
        
        gazebo_sim,
        bringup_ld,
        spawn_robot,
        bridge,
        
        # Robot spawn olduktan sonra controllerları çalıştır
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner, robot_controller_spawner],
            )
        ),
    ])
