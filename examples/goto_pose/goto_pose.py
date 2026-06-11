#!/usr/bin/env python3
"""Plan and execute a single Cartesian go-to-pose on the KUKA LBR Med 7 R800.

A thin MoveItPy driver: read a target end-effector pose from the command line, plan against the
stock med7 MoveIt configuration brought up by ``lbr_bringup``, and execute it on whatever hardware
interface is active (mock or real). This is the only custom Python in the overlay; it adds no wrapper
layer beyond a CLI-to-MoveItPy bridge.

Usage (mock first; bring up MoveIt in another sourced shell, then):

    python3 goto_pose.py --x 0.4 --y 0.0 --z 0.6

Exits nonzero if planning or execution fails.

[VERIFY ON FIRST RUN] The planning-group name (``--group``), the planning frame (``--frame``), and the
robot namespace (``--robot-namespace``) are documented defaults from lbr-stack's med7 flow, not yet
confirmed against the pinned tag. See ``README.md`` for how to verify each.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanningComponent
from rclpy.logging import get_logger

# Documented defaults for the lbr-stack med7 flow. Each carries a [VERIFY ON FIRST RUN] in README.md.
DEFAULT_GROUP = "arm"
DEFAULT_FRAME = "lbr/link_0"
DEFAULT_ROBOT_NAMESPACE = "lbr"
DEFAULT_TIP_LINK = "lbr/link_ee"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the target pose and robot wiring from the command line.

    Position (``--x/--y/--z``) is required; the orientation quaternion defaults to identity. The
    group, frame, and namespace defaults are lbr-stack med7 conventions pending first-run verification.
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
    parser.add_argument(
        "--group", default=DEFAULT_GROUP, help="MoveIt planning group [VERIFY ON FIRST RUN]"
    )
    parser.add_argument(
        "--frame", default=DEFAULT_FRAME, help="planning/reference frame [VERIFY ON FIRST RUN]"
    )
    parser.add_argument(
        "--robot-namespace",
        default=DEFAULT_ROBOT_NAMESPACE,
        help="lbr_bringup robot namespace [VERIFY ON FIRST RUN]",
    )
    parser.add_argument(
        "--tip-link",
        default=DEFAULT_TIP_LINK,
        help="end-effector link the goal pose is solved for [VERIFY ON FIRST RUN]",
    )
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
    try:
        robot = MoveItPy(node_name="goto_pose")
        component = robot.get_planning_component(args.group)

        goal = build_goal_pose(args)
        # lbr_bringup applies the robot namespace as a topic/TF prefix; the tip link the goal is solved
        # for lives under it (default lbr/link_ee). Both are [VERIFY ON FIRST RUN] against the pinned tag.
        logger.info(f"goto_pose: namespace={args.robot_namespace} group={args.group} tip={args.tip_link}")

        if not plan_and_execute(robot, component, goal, args.tip_link):
            logger.error("planning failed: no trajectory for the requested pose")
            return 1

        logger.info("goto_pose: plan executed")
        return 0
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
