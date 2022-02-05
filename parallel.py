from cProfile import run
from lib2to3 import pytree
import py_trees
from youtube_dl import main

def create_root():
    root = py_trees.composites.Parallel(name="I'm the guy", policy=py_trees.common.ParallelPolicy.SuccessOnOne())

    count_four = py_trees.behaviours.Count(name="Count till four",fail_until=0, running_until=3, success_until=4)
    count_six = py_trees.behaviours.Count(name="Count till six", fail_until=0, running_until=5, success_until=6)
    count_five = py_trees.behaviours.Count(name="Count till five", fail_until=0, running_until=2, success_until=5)

    root.add_child(count_four)
    root.add_child(count_six)
    root.add_child(count_five)
    return root

if __name__ == '__main__':
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    root.setup_with_descendants()


    for i in range(1, 7):
        print("\n--------------- Tick {0} ----------------".format(i))
        root.tick_once()
        print("{}".format(py_trees.display.unicode_tree(root, show_status=True)))