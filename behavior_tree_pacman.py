from arena import *
from pacman import *
import numpy as np
import heapq


class PacmanBehavior:
    def __init__(self):
        pass

    def printPath(self, parent, j):

        # Base Case : If j is source
        if parent[j] == -1:
            print(j, end=" ")
            return
        self.printPath(parent, parent[j])
        print(j, end=" ")

    # A utility function to print
    # the constructed distance
    # array
    def printSolution(self, dist, parent):
        src = 0
        print("Vertex \t\tDistance from Source\tPath")
        for i in range(1, len(dist)):
            print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i]), end=" ")
            self.printPath(parent, i)

    def lazy_dijkstras(self, graph, root):
        # https://pythonalgos.com/dijkstras-algorithm-in-5-steps-with-python/
        n = len(graph)
        # set up "inf" distances
        dist = [np.Inf for _ in range(n)]
        parent = [-1 for _ in range(n)]
        # set up root distance
        dist[root] = 0
        # set up visited node list
        visited = [False for _ in range(n)]
        # set up priority queue
        pq = [(0, root)]
        # while there are nodes to process
        while len(pq) > 0:
            # get the root, discard current distance
            _, u = heapq.heappop(pq)
            # if the node is visited, skip
            if visited[u]:
                continue
            # set the node to visited
            visited[u] = True
            # check the distance and node and distance
            for v, l in graph[u]:
                # if the current node's distance + distance to the node we're visiting
                # is less than the distance of the node we're visiting on file
                # replace that distance and push the node we're visiting into the priority queue
                if dist[u] + l < dist[v]:
                    dist[v] = dist[u] + l
                    parent[v] = u
                    heapq.heappush(pq, (dist[v], v))
        return dist, parent

    def get_next_moves(self, start_point, w, h, obs):
        move_set = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        next_move_set = []
        for elem in move_set:
            if not self.check_collision(start_point[0] + elem[0], start_point[1] + elem[1], w, h, obs):
                next_move_set.append(elem)
        return next_move_set

    def check_elem(self, nms, nmsfp):
        for elem in nmsfp:
            if elem not in nms:
                return True
        return False

    def check_destination(self, dest, cand, i):
        if cand[0] + i > dest[0] > cand[0] - i and cand[1] + i > dest[1] > cand[1] - i:
            return True
        else:
            return False

    def graph(self, player, obstacles, destination, graph_list, start_point, dicti={0: []}, depth=0):
        dic_index = graph_list.index(start_point)
        if dic_index not in dicti:
            dicti[dic_index] = []
        pacman_x, pacman_y, w, h = player.rect()
        next_move_set = self.get_next_moves(start_point, w, h, obstacles)
        next_point_set = []
        for elem in next_move_set:
            i = 1
            while True:
                next_point = (start_point[0] + i * elem[0], start_point[1] + i * elem[1])
                next_move_set_from_point = self.get_next_moves(next_point, w, h, obstacles)
                if self.check_elem(next_move_set, next_move_set_from_point) \
                        or self.check_collision(next_point[0], next_point[1], w, h,
                                                obstacles) or self.check_destination(destination, next_point, 1):
                    break
                i += 1

            if next_point not in graph_list and depth < 8:
                graph_list.append(next_point)
                next_point_set.append(next_point)
            if next_point in graph_list:
                dicti[dic_index].append([graph_list.index(next_point), i])
            if destination not in graph_list:
                next_point_set.append(destination)
                graph_list.append(destination)
        if depth < 8:
            for elem in next_point_set:
                graph_list, dicti = self.graph(player=player, obstacles=obstacles, destination=destination,
                                               graph_list=graph_list,
                                               start_point=elem, dicti=dicti, depth=depth + 1)
        return graph_list, dicti

    def check_collision(self, x1, y1, w, h, a) -> bool:
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

    def action_from_state(self, obs, player):
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
        dist_ghost = [d[0] + d[1] for d in
                      np.abs([np.subtract(elem.get_pos(), (pacman_x, pacman_y)) for elem in ghosts])]

        destination = (175, 208)
        graph_list, graph_dict = self.graph(player, obstacles=walls, destination=destination, dicti={},
                                            graph_list=[(pacman_x, pacman_y)], start_point=(pacman_x, pacman_y))

        index_destination = graph_list.index(destination)
        # we'll try to find to shortest path from pacman position to destination
        dist, parent = self.lazy_dijkstras(graph_dict, 0)
        self.printSolution(dist,parent)

        # print(len(graph_list),graph_list)

        # example
        for count, elem in enumerate(dist_ghost):
            if elem < 50:
                print(f'ghost is near dist = {elem}, ghost number = {count}, at position = {ghosts[count].get_pos()}')

        # 4 direction possible --> 1,0 | 0,1 | -1,0 | 0,-1 = right | up | left | down
        # example
        action_dx, action_dy = -1, 0  # move pacman left

        return action_dx, action_dy
