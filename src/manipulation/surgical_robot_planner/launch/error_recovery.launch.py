from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.moveit import LBRMoveGroupMixin

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = (mode == "gazebo")

    moveit_configs = LBRMoveGroupMixin.moveit_configs_builder(
        robot_name=model,
        package_name=f"{model}_moveit_config",
    ).planning_pipelines(
        default_planning_pipeline="pilz_industrial_motion_planner",
        pipelines=["pilz_industrial_motion_planner", "ompl"],
    )

    return [
        Node(
            package="surgical_robot_planner",
            executable="error_recovery_node",
            parameters=[
                moveit_configs.to_dict(),
                {"use_sim_time": use_sim_time},
                LBRDescriptionMixin.param_robot_name(),
            ],
        )
    ]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_mode())
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld