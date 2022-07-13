#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from turtle import goto
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import py_trees_ros_interfaces.action as py_trees_actions
from nav2_msgs.action import NavigateToPose

done = False

class IsDone(py_trees.behaviour.Behaviour):

    def __init__(self, name="IsDone"):
        super(IsDone, self).__init__(name)


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Sequence("GoToPose")

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.pose.position.x = 150.0
    goal_msg.pose.pose.position.y = 50.0

    goto_action = py_trees_ros.actions.ActionClient(
        name="Rotate",
        action_type=NavigateToPose,
        action_name="navigate_to_pose/wall_e",
        action_goal=goal_msg,
        generate_feedback_message=lambda msg: "remaining: {0}".format(msg.feedback.distance_remaining)
    )

    root.add_child(goto_action)

    return root


def main():
    """
    Entry point for the demo script.
    """
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()