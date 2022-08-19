#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import itertools
import typing
import operator
import py_trees_ros_interfaces.action as py_trees_actions
from nav2_msgs.action import NavigateToPose

done = False

class IsDone(py_trees.behaviour.Behaviour):

    def __init__(self, name="IsDone"):
        super(IsDone, self).__init__(name)

class Fallback(py_trees.composites.Composite):
    """
    A fallback will progressively tick over each of its children so long as
    each child returns :data:`~py_trees.common.Status.FAILURE`. If any child returns
    :data:`~py_trees.common.Status.SUCCESS` or :data:`~py_trees.common.Status.RUNNING` the sequence will halt and the parent will adopt
    the result of this child. If it reaches the last child, it returns with
    that result regardless.

    .. note::

       The sequence halts once it sees a child is RUNNING and then returns
       the result. *It does not get stuck in the running behaviour*.

    .. seealso:: The :ref:`py-trees-demo-sequence-program` program demos a simple sequence in action.

    Args:
        name: the composite behaviour name
        memory: if :data:`~py_trees.common.Status.RUNNING` on the previous tick, resume with the :data:`~py_trees.common.Status.RUNNING` child
        children: list of children to add

    """

    def __init__(
        self,
        name: str="Fallback",
        memory: bool=True,
        children: typing.List[py_trees.behaviour.Behaviour]=None
    ):
        super(Fallback, self).__init__(name, children)
        self.memory = memory

    def tick(self):
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # initialise
        index = 0
        if self.status != py_trees.common.Status.RUNNING or not self.memory:
            self.current_child = self.children[0] if self.children else None
            for child in self.children:
                if child.status != py_trees.common.Status.INVALID:
                    child.stop(py_trees.common.Status.INVALID)
            # user specific initialisation
            self.initialise()
        else:  # self.memory is True and status is RUNNING
            index = self.children.index(self.current_child)

        # customised work
        self.update()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(py_trees.common.Status.FAILURE)
            yield self
            return

        # actual work
        for child in itertools.islice(self.children, index, None):
            for node in child.tick():
                yield node
                if node is child and node.status != py_trees.common.Status.FAILURE:
                    self.status = node.status
                    yield self
                    return
            try:
                # advance if there is 'next' sibling
                self.current_child = self.children[index + 1]
                index += 1
            except IndexError:
                pass

        self.stop(py_trees.common.Status.FAILURE)
        yield self

class NavToPoseActionClient(py_trees_ros.actions.ActionClient):

    def __init__(
        self,
        name="NavToPose",
        robot_name="wall_e",
        goal=None,
        generate_feedback_message=lambda msg: {}
    ):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="navigate_to_pose/" + robot_name,
            action_goal=goal,
            generate_feedback_message=generate_feedback_message
        )

        self.done = py_trees.blackboard.Client(name="Arrived")
        self.done.register_key(key="arrived", access=py_trees.common.Access.WRITE)

    def update(self):
        status = super().update()
        if status == py_trees.common.Status.SUCCESS:
            self.done.arrived = py_trees.common.Status.SUCCESS
        return status

def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Sequence("GoToPoseSequence")

    goto_fallback = Fallback("GoToPoseFallback")

    arrivedBB = py_trees.blackboard.Client(name="Arrived")
    arrivedBB.register_key(key="arrived", access=py_trees.common.Access.WRITE)
    arrivedBB.arrived = py_trees.common.Status.RUNNING
    arrived = py_trees.behaviours.CheckBlackboardVariableValue(
        name="CheckArrived",
        check=py_trees.common.ComparisonExpression(
            variable="arrived",
            value=py_trees.common.Status.SUCCESS,
            operator=operator.eq
        )
    )

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.pose.position.x = 150.0
    goal_msg.pose.pose.position.y = 50.0
    goto_action = NavToPoseActionClient(
        name="GoToPoseAction",
        robot_name="wall_e",
        goal=goal_msg,
        generate_feedback_message=lambda msg: "remaining: {0}".format(msg.feedback.distance_remaining)
    )

    root.add_child(goto_fallback)
    goto_fallback.add_children([arrived, goto_action])

    return root

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_root()
    root.setup()

    print("{}".format(py_trees.display.unicode_tree(root, show_status=True)))

    tick = 1
    while True:
        print("\n-------------- Tick %d --------------\n" % tick)
        root.tick_once()
        tick += 1
        if root.status == py_trees.common.Status.SUCCESS or root.status == py_trees.common.Status.FAILURE:
            break

def main_with_tree():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

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

def test_fallback():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    fail_counter1 = py_trees.behaviours.TickCounter(name="First Fail Counter", duration=3, completion_status=py_trees.common.Status.FAILURE)
    fail_counter2 = py_trees.behaviours.TickCounter(name="Second Fail Counter", duration=2, completion_status=py_trees.common.Status.FAILURE)
    success_counter1 = py_trees.behaviours.TickCounter(name="First Success Counter", duration=3, completion_status=py_trees.common.Status.SUCCESS)
    test = Fallback(name="Fallback test", children=[success_counter1, fail_counter1, fail_counter2])
    test.setup()

    print("{}".format(py_trees.display.unicode_tree(test, show_status=True)))

    tick = 1
    while True:
        print("\n-------------- Tick %d --------------\n" % tick)
        test.tick_once()
        tick += 1
        if test.status == py_trees.common.Status.SUCCESS or test.status == py_trees.common.Status.FAILURE:
            break

if __name__ == '__main__':
    main_with_tree()