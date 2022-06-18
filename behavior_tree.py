import py_trees
from behavior_tree_pacman import PacmanBehavior
from pacman import Ghost, Wall
from map_graph import *


class FoodAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, arena, name="FoodAvailable",):
        """Configure the name of the behaviour."""
        super(FoodAvailable, self).__init__(name)
        self.arena = arena
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.status = self.arena.playing()

    def update(self):
        """Increment the counter and decide on a new status."""
        self.status = self.arena.playing()
        new_status = py_trees.common.Status.SUCCESS if self.status == 0 else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Still cookies to eat !"
        else:
            self.feedback_message = "No more Cookies left"
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class PillsAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, arena, name="PillsAvailable",):
        """Configure the name of the behaviour."""
        super(PillsAvailable, self).__init__(name)
        self.arena = arena
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.powerStatus = self.arena.powerLeft()

    def update(self):
        """Increment the counter and decide on a new status."""
        self.powerStatus = self.arena.powerLeft()
        new_status = py_trees.common.Status.SUCCESS if self.powerStatus else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Still power to take !"
        else:
            self.feedback_message = "No more power left"
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class GhostTooClose(py_trees.behaviour.Behaviour):
    def __init__(self, arena, name="GhostTooClose",):
        """Configure the name of the behaviour."""
        super(GhostTooClose, self).__init__(name)
        self.arena = arena
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.ghostsStatus = self.arena.ghostTooClose()

    def update(self):
        """Increment the counter and decide on a new status."""
        self.ghostsStatus = self.arena.ghostTooClose()
        new_status = py_trees.common.Status.SUCCESS if self.ghostsStatus else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Ghost is too close !"
        else:
            self.feedback_message = "No ghost is close"
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class FindClosestPill(py_trees.behaviour.Behaviour):
    def __init__(self, arena, name="FindClosestPill",):
        """Configure the name of the behaviour."""
        super(FindClosestPill, self).__init__(name)
        self.arena = arena
        self.pill = self.arena.getClosestPower()
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.pill_found = False
        #self.pill = None

    def update(self):
        """Increment the counter and decide on a new status."""
        self.pill = self.arena.getClosestPower()
        self.pill_found = True
        #print(self.pill.get_pos())
        new_status = py_trees.common.Status.SUCCESS if self.pill_found else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Closest power found at " + str(self.pill.get_pos())
        else:
            self.feedback_message = "Not found"
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def position(self):
        return self.pill.get_pos()

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        self.pill_found = False


class FindClosestFood(py_trees.behaviour.Behaviour):
    def __init__(self, arena, name="FindClosestFood",):
        """Configure the name of the behaviour."""
        super(FindClosestFood, self).__init__(name)
        self.arena = arena
        self.food = self.arena.getClosestFood()
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.food_found = False
        #self.food = None

    def update(self):
        """Increment the counter and decide on a new status."""
        self.food = self.arena.getClosestFood()
        self.food_found = True
        #print("closest food ", self.food.get_pos())
        new_status = py_trees.common.Status.SUCCESS if self.food_found else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Closest food found at " + str(self.food.get_pos())
        else:
            self.feedback_message = "Not found"
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def position(self):
        return self.food.get_pos()

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        self.food_found = False


class ExecutePlan(py_trees.behaviour.Behaviour):
    def __init__(self, arena, pacman, destination, type, name="ExecutePlan",):
        """Configure the name of the behaviour."""
        super(ExecutePlan, self).__init__(name)
        self.arena = arena
        self.pacman = pacman
        self.destination_object = destination
        self.type = type
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.pb = PacmanBehavior()
        self.destination = self.destination_object.position()

    def update(self):
        """Increment the counter and decide on a new status."""
        if self.type == "cookie":
            corrected_destination = (self.destination[0] - 6, self.destination[1] - 6)
        else:
            corrected_destination = (self.destination[0] - 4, self.destination[1] - 4)
        if corrected_destination == self.pacman.get_pos():
            new_status = py_trees.common.Status.SUCCESS
        else:
            new_status = py_trees.common.Status.RUNNING
            dx, dy = self.pb.action_from_state(self.arena.actors(), self.pacman, corrected_destination)
            self.pacman.direction(dx, dy)

        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Arrived at destination!"
        else:
            self.feedback_message = "Moving to destination " + str(corrected_destination)
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class AvoidGhost(py_trees.behaviour.Behaviour):
    def __init__(self, arena, pacman, name="AvoidGhost",):
        """Configure the name of the behaviour."""
        super(AvoidGhost, self).__init__(name)
        self.arena = arena
        self.pacman = pacman
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.pb = PacmanBehavior()


    def update(self):
        """Increment the counter and decide on a new status."""
        new_status = py_trees.common.Status.RUNNING

        pacman_x, pacman_y, w, h = self.pacman.rect()
        ghosts = []
        walls = []
        for elem in self.arena._actors:
            if isinstance(elem, Ghost):
                ghosts.append(elem)
            elif isinstance(elem, Wall):
                walls.append(elem)

        possible_moves = self.pb.get_next_moves((pacman_x, pacman_y), w, h, ghosts + walls)

        if len(possible_moves) > 0:
            pm = possible_moves[0]
            action_dx, action_dy = int(pm[0]), int(pm[1])
            self.pacman.direction(action_dx, action_dy)
            new_status = py_trees.common.Status.SUCCESS

        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Ghost avoided"
        else:
            self.feedback_message = "Avoiding ghost "
        self.logger.debug(
            "%s.update()[%s->%s][%s]" % (
                self.__class__.__name__,
                self.status, new_status,
                self.feedback_message
            )
        )
        return new_status

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


class BehaviorTree:

    def __init__(self, arena, pacman):
        root = py_trees.composites.Selector("Selector")

        foodAvailable = FoodAvailable(arena)
        inverter1 = py_trees.decorators.Inverter(foodAvailable)
        
         
        seq1 = py_trees.composites.Sequence("Sequence")

        sel2 = py_trees.composites.Selector("Selector")
        pillsAvailable = PillsAvailable(arena)
        inverter2 = py_trees.decorators.Inverter(pillsAvailable)

        seq2 = py_trees.composites.Sequence("Sequence")
        sel3 = py_trees.composites.Selector("Selector")
        ghostTooClose = GhostTooClose(arena)
        inverter3 = py_trees.decorators.Inverter(ghostTooClose)
        avoidGhost = AvoidGhost(arena, pacman)
        sel3.add_children([inverter3, avoidGhost])

        seq3 = py_trees.composites.Sequence("Sequence")
        findClosestPill = FindClosestPill(arena)
        type1 = "pill"
        executePlan = ExecutePlan(arena, pacman, findClosestPill, type1)
        seq3.add_children([findClosestPill, executePlan])

        seq2.add_children([sel3, seq3])
        sel2.add_children([inverter2, seq2])

        seq4 = py_trees.composites.Sequence("Sequence")

        
        sel4 = py_trees.composites.Selector("Selector")
        ghostTooClose2 = GhostTooClose(arena)
        inverter4 = py_trees.decorators.Inverter(ghostTooClose2)
        avoidGhost2 = AvoidGhost(arena, pacman)
        sel4.add_children([inverter4, avoidGhost2])

        seq5 = py_trees.composites.Sequence("Sequence")
        findClosestFood = FindClosestFood(arena)
        type2 = "cookie"
        executePlan2 = ExecutePlan(arena, pacman, findClosestFood, type2)
        seq5.add_children([findClosestFood, executePlan2])

        seq4.add_children([sel4, seq5])
        
        #low = py_trees.behaviours.Failure(name="Low Priority")
        seq1.add_children([sel2, seq4])


        root.add_children([inverter1, seq1])

        #findClosestFood = FindClosestFood(arena)

        #high = py_trees.behaviours.Success(name="High Priority")
        med = py_trees.behaviours.Success(name="Med Priority")
        low = py_trees.behaviours.Failure(name="Low Priority")
        #root.add_children([inverter, inverter2, inverter3, findClosestPill, findClosestFood, med, low])
        #root.add_children([inverter, inverter2, rr])
        #root.add_children([inverter, inverter2, avoidGhost])



        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)

        print(py_trees.display.unicode_tree(root=root))
        #behaviour_tree.setup(timeout=15)


    def tick_tree(self):
        def print_tree(tree):
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

        try:
            self.behaviour_tree.tick_tock(
                period_ms=500,
                number_of_iterations=1,     # py_trees.trees.CONTINUOUS_TICK_TOCK,
                pre_tick_handler=None,
                post_tick_handler=print_tree
            )
        except KeyboardInterrupt:
            self.behaviour_tree.interrupt()


    def tick_once(self):
        def print_tree(tree):
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        self.behaviour_tree.tick(post_tick_handler=print_tree)