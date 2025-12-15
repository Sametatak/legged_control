import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    # ---------------------------------------------------------
    # 1. ORTAM ve PATH AYARLARI (Inspection World)
    # ---------------------------------------------------------
    
    # İlgili paket yolları
    pkg_cpr_inspection = get_package_share_directory('cpr_inspection_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # GAZEBO_MODEL_PATH Ayarı:
    # Inspection paketi genellikle modelleri kendi üst klasöründe veya cpr_accessories içinde arar.
    inspection_path = os.path.dirname(pkg_cpr_inspection)
    
    # Eğer sistemde cpr_accessories varsa onu da ekleyelim (genelde cpr paketleri buna ihtiyaç duyar)
    try:
        pkg_cpr_accessories = get_package_share_directory('cpr_accessories_gazebo')
        accessories_path = os.path.dirname(pkg_cpr_accessories)
    except:
        accessories_path = ""

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # Path setini oluştur
    path_set = {inspection_path}
    if accessories_path:
        path_set.add(accessories_path)
        
    if existing_model_path:
        for p in existing_model_path.split(':'):
            if p: path_set.add(p)

    new_model_path = ':'.join(list(path_set))

    # Gazebo çevre değişkenini ayarla
    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=new_model_path
    )

    # Dünya Dosyası Yolu
    world_file_name = 'inspection_world.world'
    default_world_path = os.path.join(pkg_cpr_inspection, 'worlds', world_file_name)

    # ---------------------------------------------------------
    # 2. INSPECTION GEOMETRİSİNİ SPAWN ETME
    # ---------------------------------------------------------
    # ROS 1 kodundaki <param name="inspection_geom" ...> karşılığı
    inspection_xacro_file = os.path.join(pkg_cpr_inspection, 'urdf', 'inspection_geometry.urdf.xacro')
    inspection_description_content = Command(['xacro ', inspection_xacro_file])

    # Bu geometriyi yayınlayacak bir State Publisher (Robot description ile çakışmaması için remapping yapıyoruz)
    inspection_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='inspection_state_publisher',
        output='screen',
        parameters=[{'robot_description': inspection_description_content}],
        remappings=[('robot_description', 'inspection_description')]
    )

    # Geometriyi Gazebo'ya Spawn etme (0,0,0 noktasına)
    spawn_inspection_geom = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'inspection_description', 
            '-entity', 'inspection_geometry', 
            '-x', '0', '-y', '0', '-z', '0', '-Y', '0'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # 3. ROBOT VE ARGÜMAN AYARLARI
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    
    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
    
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "/config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_VLP.xacro")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )

    # ÖNEMLİ: ROS 1 kodundaki spawn koordinatlarını buraya default olarak aldım.
    # robot_x: 6.0, robot_y: -18.0, robot_z: 2.0
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="6.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="-18.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="2.0")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup2.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            # Koordinatları yukarıdaki declare argümanlarından alıyoruz
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
        }.items(),
    )
    
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    return LaunchDescription(
        [
            set_model_path_cmd,         # 1. Mesh yollarını ayarla
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            inspection_state_publisher, # 2. Inspection ortamını URDF olarak yayınla
            bringup_ld,                 # 3. Robot sistemini başlat
            gazebo_ld,                  # 4. Gazebo'yu başlat
            spawn_inspection_geom,      # 5. Inspection ortamını Gazebo'ya spawn et
            map_to_odom_tf, 
        ]
    )
