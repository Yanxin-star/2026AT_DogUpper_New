from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    simulate_env_launch_scripe="arm_mujoco_sim.py"

    urdf_path = os.path.join(
        get_package_share_directory("arm"),
        "model", "robotic_arm.urdf"
    )
    # 读取URDF内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc"
    )

    static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.04023', '-0.20514', '0.26134',
        '1.570796', '-1.570796', '1.570796',
        'link5', 'camera_link'
    ]
)

    rviz2_config_path=os.path.join(
        get_package_share_directory("launch_pack"),
        "rviz", "display_config.rviz"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config_path]  # 可选，指定rviz配置文件
    )
    
    sim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('launch_pack'), 'launch', simulate_env_launch_scripe)]))
    
    return LaunchDescription([robot_state_pub,  arm_calc , rviz2 ,sim_launch ])
