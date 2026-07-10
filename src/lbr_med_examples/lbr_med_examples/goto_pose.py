#!/usr/bin/env python3
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
"""Plan and execute a single Cartesian go-to-pose on the KUKA LBR Med 7 R800.

A thin MoveItPy driver: read a target end-effector pose from the command line, plan against the
stock med7 MoveIt configuration brought up by ``lbr_bringup``, and execute it on whatever hardware
interface is active (mock or real). This is the only custom Python in the overlay; it adds no wrapper
layer beyond a CLI-to-MoveItPy bridge.

Usage (mock first; bring up MoveIt, then run under the ``/lbr`` namespace via the launch file):

    ros2 launch lbr_med_examples goto_pose.launch.py x:=0.4 y:=0.0 z:=0.6

Exits nonzero if planning or execution fails.

The planning group (``arm``), planning frame (``lbr_link_0``), tip link (``lbr_link_ee``), and robot
namespace (``lbr``) defaults are confirmed against the med7 SRDF and the ``lbr_bringup`` mixin defaults.
Real-hardware controller names remain [VERIFY ON FIRST RUN] at bringup.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanningComponent
from rclpy.logging import get_logger
from rclpy.utilities import remove_ros_args

# SRDF-confirmed med7 defaults (med7.srdf group "arm": base_link=lbr_link_0, tip_link=lbr_link_ee).
# The tf prefix is <robot_name>_link (underscore, not slash) per the lbr_bringup mixins.
DEFAULT_GROUP = "arm"
DEFAULT_FRAME = "lbr_link_0"
DEFAULT_ROBOT_NAMESPACE = "lbr"
DEFAULT_TIP_LINK = "lbr_link_ee"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the target pose and robot wiring from the command line.

    Position (``--x/--y/--z``) is required; the orientation quaternion defaults to identity. The
    group, frame, tip-link, and namespace defaults are SRDF-confirmed med7 values, overridable here.
    """
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--x", type=float, required=True, help="target position x in --frame (m)")
    parser.add_argument("--y", type=float, required=True, help="target position y in --frame (m)")
    parser.add_argument("--z", type=float, required=True, help="target position z in --frame (m)")
    parser.add_argument("--qx", type=float, default=0.0, help="target orientation quaternion x")
    parser.add_argument("--qy", type=float, default=0.0, help="target orientation quaternion y")
    parser.add_argument("--qz", type=float, default=0.0, help="target orientation quaternion z")
    parser.add_argument(
        "--qw", type=float, default=1.0, help="target orientation quaternion w (identity default)"
    )
    parser.add_argument("--group", default=DEFAULT_GROUP, help="MoveIt planning group")
    parser.add_argument("--frame", default=DEFAULT_FRAME, help="planning/reference frame")
    parser.add_argument(
        "--robot-namespace",
        default=DEFAULT_ROBOT_NAMESPACE,
        help="lbr_bringup robot namespace (topic/TF prefix)",
    )
    parser.add_argument(
        "--tip-link",
        default=DEFAULT_TIP_LINK,
        help="end-effector link the goal pose is solved for",
    )
    # `ros2 run` and the launch Node(...) append `--ros-args -r __ns:=/lbr --params-file ...` to the
    # process argv; strip those so argparse sees only this node's own flags (else it dies on
    # "unrecognized arguments"). remove_ros_args expects a full argv (prog name at [0], like sys.argv),
    # so only the sys.argv fall-through path is filtered; an explicit argv is treated as clean flags.
    if argv is None:
        argv = remove_ros_args(args=sys.argv)[1:]
    return parser.parse_args(argv)


def build_goal_pose(args: argparse.Namespace) -> PoseStamped:
    """Assemble the target ``PoseStamped`` from the parsed CLI arguments."""
    pose = PoseStamped()
    pose.header.frame_id = args.frame
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z
    pose.pose.orientation.x = args.qx
    pose.pose.orientation.y = args.qy
    pose.pose.orientation.z = args.qz
    pose.pose.orientation.w = args.qw
    return pose


def plan_and_execute(
    robot: MoveItPy,
    component: PlanningComponent,
    goal: PoseStamped,
    pose_link: str,
) -> bool:
    """Plan to ``goal`` for ``component`` and execute the result.

    Returns ``True`` on a successful plan+execute, ``False`` if planning yields no trajectory.
    """
    component.set_start_state_to_current_state()
    component.set_goal_state(pose_stamped_msg=goal, pose_link=pose_link)

    plan_result = component.plan()
    if not plan_result:
        return False

    robot.execute(plan_result.trajectory, controllers=[])
    return True


def main(argv: list[str] | None = None) -> int:
    """Entry point: init MoveItPy, plan+execute the CLI pose, return a process exit code."""
    args = parse_args(argv)
    logger = get_logger("goto_pose")

    rclpy.init()
    # MoveItPy starts its own rclcpp node and does NOT inherit the launch `namespace=` remap, so pass
    # name_space explicitly: its planning-scene monitor must subscribe to /<ns>/joint_states and read
    # /<ns>/robot_description, matching the namespaced move_group. Without this MoveItPy lands in the
    # root namespace and its state monitor times out waiting for joint angles.
    robot = MoveItPy(node_name="goto_pose", name_space=args.robot_namespace)
    component = robot.get_planning_component(args.group)

    goal = build_goal_pose(args)
    logger.info(
        f"goto_pose: namespace={args.robot_namespace} group={args.group} tip={args.tip_link}"
    )

    if not plan_and_execute(robot, component, goal, args.tip_link):
        logger.error("planning failed: no trajectory for the requested pose")
        exit_code = 1
    else:
        logger.info("goto_pose: plan executed")
        exit_code = 0

    # Tear MoveItPy down (releasing its rclcpp context) BEFORE rclpy.shutdown(): calling shutdown while
    # the MoveItPy C++ node is still alive races its destructor and segfaults on exit (upstream-known).
    del component
    del robot
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
