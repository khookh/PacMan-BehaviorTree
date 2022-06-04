from arena import *
from pacman import *
import numpy as np


def action_from_state(obs, player):
    """
    :param obs: current game state (arena.actors, list)
    :return:
    """

    pacman_x, pacman_y = player.get_pos()

    ghosts = []
    food = []
    for elem in obs:
        if isinstance(elem, Ghost):
            ghosts.append(elem.get_pos())
        elif isinstance(elem, Cookie):
            food.append(elem.get_pos())

    # example
     # list containing manathan distance from pacman to each ghost
    dist_ghost = [d[0] + d[1] for d in np.abs([np.subtract(elem, (pacman_x, pacman_y)) for elem in ghosts])]
    for count, elem in enumerate(dist_ghost):
        if elem < 50:
            print(f'ghost is near dist = {elem}, ghost number = {count}, at position = {ghosts[count]}')
    # example

    action_dx, action_dy = -2, 0  # move pacman up, temp for test
    #

    ###
    # PROCESS THE GAME STATE HERE , RETURN ACTION TO DO FOR PACMAN
    ###

    return action_dx, action_dy
