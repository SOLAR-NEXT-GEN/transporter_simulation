import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
    # Constants for paths to different files and folders
    gazebo_models_path = 'models'
    package_name = 'transporter_simulation'
    robot_name_in_model = 'transporter_robot'
    rviz_config_file_path = 'rviz/robot_map.rviz'
    urdf_file_path = 'urdf/diff_drive.urdf'
    world_file = 'gps_world.sdf'

    # Spawn robot at desired position in Gazebo
    # Since the world file already has spherical_coordinates set to our datum,
    # these coordinates will be properly aligned with the GPS/OSM coordinate system
    spawn_x_val = '0.0'  # East offset from datum
    spawn_y_val = '0.0'  # North offset from datum  
    spawn_z_val = '0.01'
    spawn_yaw_val = '0.0'

    ############ You do not need to change anything below this line #############

    # Set the path to different files and folders.  
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    world_path = os.path.join(pkg_share, 'worlds', world_file)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)

    # Get controller config path
    simulation_share = get_package_share_directory('transporter_simulation')
    controllers_config = os.path.join(simulation_share, 'config', 'controllers_hinge.yaml')

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # Declare the launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    # Verify controller config file exists
    if not os.path.isfile(controllers_config):
        raise FileNotFoundError(f"controllers_hinge.yaml not found at: {controllers_config}")

    # Read URDF file and replace the controllers path
    with open(default_urdf_model_path, 'r') as infp:
        robot_description_content = infp.read()

    # Replace the placeholder path with the actual path
    robot_description_content = robot_description_content.replace(
        '$(find transporter_simulation)/config/controllers_hinge.yaml',
        controllers_config
    )

    print(f"Controllers config path: {controllers_config}")
    print("Controller config exists:", os.path.exists(controllers_config))

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher for controlling steering
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint State Publisher GUI for manual control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Start Gazebo with world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={
            'world': world_path, 
            'verbose': 'false',
            'gui': PythonExpression(['not ', headless])
        }.items()
    )

    # Add after world_path is set:
    print(f"World file path: {world_path}")
    print(f"World file exists: {os.path.exists(world_path)}")
 
    # Launch the robot at specified position in Gazebo
    # Gazebo world already has spherical_coordinates set to match our GPS datum
    # So these local coordinates will align properly with GPS/OSM data
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, 
                   '-topic', 'robot_description',
                   '-timeout', '120.0',
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen')
    
    # Set up Gazebo environment paths
    description_package_name = "transporter_description"
    install_dir = get_package_prefix(description_package_name)
    gazebo_plugins_name = "gazebo_plugins"
    
    try:
        gazebo_plugins_path = get_package_prefix(gazebo_plugins_name)
    except:
        gazebo_plugins_path = None

    # Set GAZEBO_MODEL_PATH - Only add the models directory if it exists
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH']
    else:
        model_path = ''
    
    # Only add models directory if it exists
    if os.path.exists(gazebo_models_path):
        if model_path:
            model_path += ':' + gazebo_models_path
        else:
            model_path = gazebo_models_path
    
    if model_path:
        os.environ['GAZEBO_MODEL_PATH'] = model_path

    # Set GAZEBO_PLUGIN_PATH
    plugin_path = ''
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        plugin_path = os.environ['GAZEBO_PLUGIN_PATH']
    
    # Add install/lib directory
    lib_path = os.path.join(install_dir, 'lib')
    if os.path.exists(lib_path):
        if plugin_path:
            plugin_path += ':' + lib_path
        else:
            plugin_path = lib_path
    
    # Add gazebo_plugins if available
    if gazebo_plugins_path:
        gazebo_lib_path = os.path.join(gazebo_plugins_path, 'lib')
        if os.path.exists(gazebo_lib_path):
            if plugin_path:
                plugin_path += ':' + gazebo_lib_path
            else:
                plugin_path = gazebo_lib_path
    
    if plugin_path:
        os.environ['GAZEBO_PLUGIN_PATH'] = plugin_path

    print("GAZEBO MODELS PATH==" + str(os.environ.get("GAZEBO_MODEL_PATH", "Not set")))
    print("GAZEBO PLUGINS PATH==" + str(os.environ.get("GAZEBO_PLUGIN_PATH", "Not set")))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add actions in order
    ld.add_action(gazebo_launch)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_node)
    # ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_rviz_cmd)

    return ld