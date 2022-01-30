import py_trees
import random

class MusicCreator(py_trees.behaviour.Behaviour):
    """
    This a test class to create show random notes to form music
    """
    def __init__(self, name="MusicCreator"):
        super(MusicCreator, self).__init__(name)
        self.notes = ["Dó", "Ré", "Mi", "Fá", "Sol", "Lá", "Si"]
        self.logger.debug("Initing the music creator class")

    def setup(self):
        self.logger.debug("Setting up the Music creator")

    def initialise(self):
        self.countNotes = 0
        self.logger.debug("Initializing Music creator")

    def update(self):
        new_note = self.notes[random.randint(0, len(self.notes)-1)]
        self.logger.debug("New note: %s" % new_note)
        if self.countNotes == 5:
            return py_trees.common.Status.SUCCESS
        self.countNotes += 1
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("We're finishing")

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    music_creator = MusicCreator()
    music_creator.setup()
    for i in range(15):
        music_creator.tick_once()

if __name__ == "__main__":
    main()