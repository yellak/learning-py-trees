import py_trees

class Position:
    """
    Defines a position
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    
    def __str__(self) -> str:
        return "%d, %d" % (self.x, self.y)

class Robot:
    """
    This represents a Robot
    """

    def __init__(self, name="Robot", pos=Position()):
        self.__class__.__name__ = name
        self.pos = pos

class Navigate(py_trees.behaviour.Behaviour):
    """
    Simulates a navigation
    """
    def __init__(self, name="Navigate"):
        super(Navigate, self).__init__(name)

    def setup(self, **kwargs):
        self.entity = kwargs.get("entity")
        self.destiny = kwargs.get("destiny")
    
    def initialise(self):
        self.done = False

    def update(self):
        """
        Here, we simulate the behaviour of going to the destiny
        """
        if self.done:
            self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, "I arrived"))
            self.logger.debug("%s.update()[I'm in %s]" % (self.__class__.__name__, self.entity.pos))
            return py_trees.common.Status.SUCCESS

        self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, "Going to the destiny"))
        self.logger.debug("%s.update()[I'm in %s]" % (self.__class__.__name__, self.entity.pos))
        self.entity.pos = self.destiny
        self.done = True
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass

def pickup_sample_mission():
    mission = py_trees.composites.Sequence(name="Mission")
    robot1 = Robot(name="R1", pos=Position(5,5))
    navto_room3 = Navigate(name="Navigate to Room3")
    navto_pharmacy = Navigate(name="Navigate to Pharmacy")

    navto_room3.setup(entity=robot1, destiny=Position(x=1, y=1))
    navto_pharmacy.setup(entity=robot1, destiny=Position(x=6, y=6))

    mission.add_child(navto_room3)
    mission.add_child(navto_pharmacy)
    return mission

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    mission = pickup_sample_mission()
    mission.setup()
    for i in range(5):
        mission.tick_once()

if __name__ == '__main__':
    main()