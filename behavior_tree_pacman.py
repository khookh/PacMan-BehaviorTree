from arena import *
from pacman import *
import numpy as np


def check_collision(x1, y1, w, h, a) -> bool:
    """
    :param x1: test pos (x) of pacman
    :param y1: test pos (y) of pacman
    :param w: width of pacman
    :param h: height of pacman
    :param a: actor element
    :return: bool, true if collision
    """
    x2, y2, w2, h2 = a.rect()
    return y2 < y1 + h and y1 < y2 + h2 and x2 < x1 + w and x1 < x2 + w2


def action_from_state(obs, player):
    """
    :param obs: current game state (arena.actors, list)
    :return:
    """

    pacman_x, pacman_y, w, h = player.rect()

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


    # example , 4 dir possible --> 1,0 | 0,1 | -1,0 | 0,-1
    action_dx, action_dy = -2, 0  # move pacman up, temp for test

    return action_dx, action_dy
