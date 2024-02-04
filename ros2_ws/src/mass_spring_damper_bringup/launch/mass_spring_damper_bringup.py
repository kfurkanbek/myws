from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # remap_number_topic = ("name", "new_name")

    state_space_service_node = Node(
        package="state_space_service",
        executable="state_space_service",
        name="state_space_service",
        remappings=[
            # remap_number_topic
        ],
        parameters=[
            {"name": "StateSpaceService"}
        ]
    )

    state_space_node = Node(
        package="mass_spring_damper",
        executable="state_space_node",
        name="state_space_node",
        remappings=[
            # remap_number_topic
        ],
        parameters=[
            {"name": "StateSpaceNode"},
            {"server_wait": 1.0},
            {"accuracy": 0.001},
            {"frequency": 100.0}
        ]
    )

    ld.add_action(state_space_service_node)
    ld.add_action(state_space_node)
    return ld

# ros2 launch mass_spring_damper_bringup mass_spring_damper_bringup.py