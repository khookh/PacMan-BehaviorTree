from arena import *
from pacman import *
import numpy as np


def get_next_moves(start_point, w, h, obs):
    move_set = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    next_move_set = []
    for elem in move_set:
        if not check_collision(start_point[0] + elem[0], start_point[1] + elem[1], w, h, obs):
            next_move_set.append(elem)
    return next_move_set


def check_elem(nms, nmsfp):
    for elem in nmsfp:
        if elem not in nms:
            return True
    return False


def graph(player, obs, destination, graph_list, start_point, dicti={0:[]}, depth=0):
    dic_index = graph_list.index(start_point)
    if dic_index not in dicti:
        dicti[dic_index] = []
    pacman_x, pacman_y, w, h = player.rect()
    next_move_set = get_next_moves(start_point, w, h, obs)
    next_point_set = []
    for elem in next_move_set:
        i = 5
        while True:
            next_point = (start_point[0] + i * elem[0], start_point[1] + i * elem[1])
            next_move_set_from_point = get_next_moves(next_point, w, h, obs)
            if check_elem(next_move_set, next_move_set_from_point) or check_collision(next_point[0], next_point[1], w, h, obs) or next_point == destination:
                break
            i += 5

        if next_point not in graph_list:
            graph_list.append(next_point)
            next_point_set.append(next_point)
        dicti[dic_index].append([graph_list.index(next_point), i])
    if depth < 8:
        for elem in next_point_set:
            graph_list, dicti = graph(player=player, obs=obs, destination=destination, graph_list=graph_list,
                                      start_point=elem, dicti=dicti, depth=depth + 1)
    return graph_list, dicti


def check_collision(x1, y1, w, h, a) -> bool:
    """
    :param x1: test pos (x) of pacman
    :param y1: test pos (y) of pacman
    :param w: width of pacman
    :param h: height of pacman
    :param a: actor element
    :return: bool, true if collision
    """
    for elem in a:
        x2, y2, w2, h2 = elem.rect()
        if y2 < y1 + h and y1 < y2 + h2 and x2 < x1 + w and x1 < x2 + w2:
            return True
    return False


def action_from_state(obs, player):
    """
    :param obs: current game state (arena.actors, list)
    :param player: pacman object
    :return: selected move
    """

    pacman_x, pacman_y, w, h = player.rect()

    ghosts = []
    food = []
    walls = []
    for elem in obs:
        if isinstance(elem, Ghost):
            ghosts.append(elem)
        elif isinstance(elem, Cookie):
            food.append(elem)
        elif isinstance(elem, Wall):
            walls.append(elem)

    # list containing manathan distance from pacman to each ghost
    dist_ghost = [d[0] + d[1] for d in np.abs([np.subtract(elem.get_pos(), (pacman_x, pacman_y)) for elem in ghosts])]

    graph_list, dici = graph(player, obs=walls + ghosts, destination=(0, 0), dicti={}, graph_list=[(pacman_x, pacman_y)], start_point=(pacman_x, pacman_y))
    # graph_list is the list of the nodes that compose the game map (wth ghost obstruction included)
    # dici : dictionnary that links each node between them and the related cost
    print(dici)

    # example
    for count, elem in enumerate(dist_ghost):
        if elem < 50:
            print(f'ghost is near dist = {elem}, ghost number = {count}, at position = {ghosts[count].get_pos()}')

    # 4 direction possible --> 1,0 | 0,1 | -1,0 | 0,-1 = right | up | left | down
    # example
    action_dx, action_dy = -1, 0  # move pacman left

    return action_dx, action_dy
