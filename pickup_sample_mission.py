import py_trees

class Position:
    """
    Represents a position
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return "%d, %d" % (self.x, self.y)

class Robot:
    """
    This class represents a very simple Robot
    """
    def __init__(self, name="Robot", pos=Position()):
        self.name = name
        self.pos = pos

    def __str__(self):
        return self.name

class Navigator(py_trees.behaviour.Behaviour):
    """
    Simulates a navigation
    """
    def __init__(self, name="Navigate"):
        super(Navigator, self).__init__(name)

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
            self.logger.debug("update()[%s]" % ("I arrived"))
            self.logger.debug("update()[I'm in %s]" % (self.entity.pos))
            return py_trees.common.Status.SUCCESS

        self.logger.debug("update()[I'm in %s]" % (self.entity.pos))
        self.logger.debug("update()[Going to %s]" % (self.destiny))
        self.entity.pos = self.destiny
        self.done = True
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass

class Approacher(py_trees.behaviour.Behaviour):
    """
    This behaviour represents an entity thats able to aproach a target
    """

    def __init__(self, name="Random Approacher"):
        super(Approacher, self).__init__(name)
    
    def setup(self, **kwargs):
        self.target = kwargs.get("target")
        self.entity = kwargs.get("entity")

    def update(self):
        self.logger.debug("update()[%s trying to approach %s]" % (self.entity, self.target))
        return py_trees.common.Status.SUCCESS

class Authenticator(py_trees.behaviour.Behaviour):

    def __init__(self, name="Random Authenticator"):
        super(Authenticator, self).__init__(name)
    
    def setup(self, **kwargs):
        self.authenticatable = kwargs.get("authenticatable")
        self.entity = kwargs.get("entity")

    def update(self):
        self.logger.debug("update()[%s authenticating %s]" % (self.entity, self.authenticatable))
        return py_trees.common.Status.SUCCESS

class RetrieveSample(py_trees.composites.Sequence):
    """
    This a part of a mission where the sample is retrieved from a target
    """

    def __init__(self, name: str = "Sequence", entity=Robot(), retrieveFrom="Random target"):
        super().__init__(name)
        self.entity = entity
        self.retrieveFrom = retrieveFrom
    
    def setup(self, **kwargs):
        approachMsg = "Approach %s" % (self.retrieveFrom)
        approach = Approacher(name=approachMsg)
        approach.setup(entity=self.entity, target=self.retrieveFrom)

        authenticateMsg = "Authenticate %s" % (self.retrieveFrom)
        authenticate = Authenticator(name=authenticateMsg)
        authenticate.setup(entity=self.entity, authenticatable=self.retrieveFrom)

        deposit = DepositSample(name="Deposit sample on delivery entity",
                                entity=self.entity,
                                whoDeposits=self.retrieveFrom)
        deposit.setup()

        self.add_child(approach)
        self.add_child(authenticate)
        self.add_child(deposit)

class DrawerControl(py_trees.behaviour.Behaviour):
    """
    This behaviour controls the drawer of an entity
    """

    def __init__(self, name="Random Drawer control"):
        super(DrawerControl, self).__init__(name)

    def initialise(self):
        self.messages_by_actions = {
            "open" : "opening drawer",
            "close" : "closing drawer",
            "wait" : "waiting for deposit"
        }
    
    def setup(self, **kwargs):
        self.action = kwargs.get("action")

    def update(self):
        message = self.messages_by_actions.get(self.action)
        self.logger.debug("update()[%s]" % (message))
        return py_trees.common.Status.SUCCESS

class DepositSample(py_trees.composites.Sequence):
    """
    This behaviour makes an entity take a deposit from someone
    """

    def __init__(self, name: str = "Sequence", entity=Robot(), whoDeposits="Random Person"):
        super().__init__(name)
        self.entity = entity
        self.whoDeposits = whoDeposits

    def setup(self, **kwargs):
        open_drawer = DrawerControl(name="Open drawer")
        open_drawer.setup(action="open")

        wait_for_deposit = DrawerControl(name="Wating deposit")
        wait_for_deposit.setup(action="wait")

        close_drawer = DrawerControl(name="Close Drawer")
        close_drawer.setup(action="close")

        self.add_child(open_drawer)
        self.add_child(wait_for_deposit)
        self.add_child(close_drawer)

def pickup_sample_mission():
    mission = py_trees.composites.Sequence(name="Mission")
    robot1 = Robot(name="R1", pos=Position(5,5))

    navto_room3 = Navigator(name="Navigate to Room3")
    navto_room3.setup(entity=robot1, destiny=Position(x=1, y=1))

    retieve_sample = RetrieveSample(name="Retrieve Sample",
                                    entity=robot1,
                                    retrieveFrom="Nurse")
    retieve_sample.setup()

    navto_pharmacy = Navigator(name="Navigate to Pharmacy")
    navto_pharmacy.setup(entity=robot1, destiny=Position(x=6, y=6))

    mission.add_child(navto_room3)
    mission.add_child(retieve_sample)
    mission.add_child(navto_pharmacy)
    return mission

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    mission = pickup_sample_mission()
    mission.setup()
    tick = 1
    while True:
        print("\n-------------- Tick %d --------------\n" % tick)
        mission.tick_once()
        tick += 1
        if mission.status == py_trees.common.Status.SUCCESS or mission.status == py_trees.common.Status.FAILURE:
            break

if __name__ == '__main__':
    main()