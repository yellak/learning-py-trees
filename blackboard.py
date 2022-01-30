import py_trees
import random

class Writer(py_trees.behaviour.Behaviour):
    def __init__(self, name="Gesser"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = kwargs.get("blackboard")
    
    def initialise(self):
        return super().initialise()
    
    def update(self):
        number_to_gess = random.randint(0, 15)
        self.blackboard.number = number_to_gess
        self.logger.debug("Gessing %d" % number_to_gess)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return super().terminate(new_status)


class Reader(py_trees.behaviour.Behaviour):
    def __init__(self, name="Reader"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.blackboard = kwargs.get("blackboard")
    
    def initialise(self):
        return super().initialise()
    
    def update(self):
        if self.blackboard.number == 4:
            self.logger.debug("You're right :D")
            return py_trees.common.Status.SUCCESS
        self.logger.debug("You're wrong :(")
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return super().terminate(new_status)


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    writer = py_trees.blackboard.Client(name="Writer")
    writer.register_key(key="number", access=py_trees.common.Access.WRITE)

    reader = py_trees.blackboard.Client(name="Reader")
    reader.register_key(key="number", access=py_trees.common.Access.READ)

    who_tries_to_guess = Writer(name="Gesser")
    who_tries_to_guess.setup(blackboard=writer)
    who_knows_the_number = Reader(name="The One above all")
    who_knows_the_number.setup(blackboard=reader)

    for gess in range(10):
        who_tries_to_guess.tick_once()
        who_knows_the_number.tick_once()
        if who_knows_the_number.status == py_trees.common.Status.SUCCESS:
            break