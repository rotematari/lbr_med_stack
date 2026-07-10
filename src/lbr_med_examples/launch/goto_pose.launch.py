# Copyright 2026 Rotem Atari
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Launch the ``goto_pose`` MoveItPy node with the med7 MoveIt config injected.

This is the diagnosed fix for the ``goto_pose`` failure: a bare ``python3 goto_pose.py`` gave
MoveItPy no robot_description / SRDF / kinematics / OMPL parameters and ran in the root namespace,
so RDFLoader timed out after 10 s. Here the node is launched under the ``/lbr`` namespace (matching
move_group) with the full med7 config injected as per-node parameters via ``moveit_config.to_dict()``.

Requires the sibling ``lbr_med_ws`` overlay to be sourced: ``lbr_bringup`` (the mixin) and
``med7_moveit_config`` / ``lbr_description`` (the config packages) live there, not in this repo.
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Build the med7 MoveIt config for the resolved model and launch the namespaced node.

    The ``model`` arg (``med7``) is the *MoveIt builder* robot_name: it selects
    ``urdf/{model}/{model}.xacro`` and the ``{model}_moveit_config`` package. The ``robot_name`` arg
    (``lbr``) is the *ROS namespace* only, applied on ``Node(namespace=...)`` so the client and the
    move_group server share ``/lbr``. Conflating the two is the trap this launch file guards against.
    """
    # Imported lazily so `-s` introspection and `ruff`/`basedpyright` do not hard-require the sibling
    # overlay on sys.path; the import only fires when the launch actually runs against a sourced ws.
    # basedpyright cannot see lbr_bringup: it lives in the sibling lbr_med_ws overlay's install, not
    # /opt/ros/jazzy, and is not on the checker's search path. Resolves at runtime under a sourced ws.
    from lbr_bringup.moveit import LBRMoveGroupMixin  # pyright: ignore[reportMissingImports]

    model = LaunchConfiguration("model").perform(context)
    # .moveit_cpp(...) adds the MoveItPy-specific config (nested planning_pipelines.pipeline_names +
    # default plan_request_params) that the move_group .to_dict() layout omits; without it MoveItPy
    # aborts with "Failed to load planning pipelines from parameter server". See config/moveit_cpp.yaml.
    moveit_cpp_yaml = os.path.join(
        get_package_share_directory("lbr_med_examples"), "config", "moveit_cpp.yaml"
    )
    moveit_config = (
        LBRMoveGroupMixin.moveit_configs_builder(
            robot_name=model,
            package_name=f"{model}_moveit_config",
        )
        .moveit_cpp(file_path=moveit_cpp_yaml)
        .to_moveit_configs()
    )

    robot_name = LaunchConfiguration("robot_name")
    node = Node(
        package="lbr_med_examples",
        executable="goto_pose",
        namespace=robot_name,
        parameters=[moveit_config.to_dict()],
        # --robot-namespace is forwarded so MoveItPy's own node (which ignores the launch namespace
        # remap) is created under /<robot_name> and its state monitor finds /<robot_name>/joint_states.
        arguments=[
            "--x",
            LaunchConfiguration("x"),
            "--y",
            LaunchConfiguration("y"),
            "--z",
            LaunchConfiguration("z"),
            "--robot-namespace",
            robot_name,
        ],
        output="screen",
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    """Declare the model / namespace / target-position args and defer node setup to ``_setup``."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="med7",
                description="MoveIt config robot model (selects {model}_moveit_config).",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="lbr",
                description="ROS namespace the node runs under (must match move_group).",
            ),
            DeclareLaunchArgument(
                "x",
                default_value="0.4",
                description="target position x in the planning frame (m).",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0.0",
                description="target position y in the planning frame (m).",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.6",
                description="target position z in the planning frame (m).",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
