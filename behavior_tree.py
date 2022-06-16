import py_trees


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
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.pill_found = False
        self.pill = None

    def update(self):
        """Increment the counter and decide on a new status."""
        self.pill = self.arena.getClosestPower()
        self.pill_found = True
        print(self.pill.get_pos())
        new_status = py_trees.common.Status.SUCCESS if self.pill_found else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Closest power found !"
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

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        self.pill_found = False


class FindClosestFood(py_trees.behaviour.Behaviour):
    def __init__(self, arena, name="FindClosestFood",):
        """Configure the name of the behaviour."""
        super(FindClosestFood, self).__init__(name)
        self.arena = arena
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        """No delayed initialisation required for this example."""
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """Reset a counter variable."""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.food_found = False
        self.food = None

    def update(self):
        """Increment the counter and decide on a new status."""
        self.food = self.arena.getClosestFood()
        self.food_found = True
        print("closest food ", self.food.get_pos())
        new_status = py_trees.common.Status.SUCCESS if self.food_found else py_trees.common.Status.FAILURE
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Closest food found !"
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

    def terminate(self, new_status):
        """Nothing to clean up in this example."""
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
        self.food_found = False


class BehaviorTree:

    def __init__(self, arena):
        root = py_trees.composites.Selector("Selector")

        foodAvailable = FoodAvailable(arena)
        inverter = py_trees.decorators.Inverter(foodAvailable)

        pillsAvailable = PillsAvailable(arena)
        inverter2 = py_trees.decorators.Inverter(pillsAvailable)

        ghostTooClose = GhostTooClose(arena)
        inverter3 = py_trees.decorators.Inverter(ghostTooClose)

        findClosestPill = FindClosestPill(arena)

        findClosestFood = FindClosestFood(arena)

        #high = py_trees.behaviours.Success(name="High Priority")
        med = py_trees.behaviours.Success(name="Med Priority")
        low = py_trees.behaviours.Failure(name="Low Priority")
        root.add_children([inverter, inverter2, inverter3, findClosestPill, findClosestFood, med, low])



        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)

        print(py_trees.display.unicode_tree(root=root))
        #behaviour_tree.setup(timeout=15)


    def tick_tree(self):
        def print_tree(tree):
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

        try:
            self.behaviour_tree.tick_tock(
                period_ms=500,
                number_of_iterations=1,  #py_trees.trees.CONTINUOUS_TICK_TOCK,
                pre_tick_handler=None,
                post_tick_handler=print_tree
            )
        except KeyboardInterrupt:
            self.behaviour_tree.interrupt()

