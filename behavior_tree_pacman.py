from arena import *
from pacman import *

def action_from_state(obs):
    """
    :param obs: current game state (arena.actors, list)
    :return:
    """

    # example
    ghosts = []
    for elem in obs:
        if isinstance(elem, Ghost):
            ghosts.append(elem)
    action_dx, action_dy = -2, 0  # move pacman up, temp for test
    #

    ###
    # PROCESS THE GAME STATE HERE , RETURN ACTION TO DO FOR PACMAN
    ###

    return action_dx, action_dy
