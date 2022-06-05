from arena import *
from pacman import *
import numpy as np
import heapq


class PacmanBehavior:
    def __init__(self):
        self.graph_list = None
        self.graph_dict = None
        self.count = 0

    def next_node(self, parent, j, previous=None):
        """
        :param parent: list of parents nodes outputed by Dijkstra
        :param j: index of destination node in the parent & graph list
        :param previous: previous parent
        :return: previous parent of the last iteration = next node from source
        """
        # Base Case : If j is source
        if parent[j] == -1:
            return previous
        return self.next_node(parent, parent[j], j)

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
        """
        :param start_point: current pacman position
        :param w: width
        :param h: 'height'
        :param obs: set of obstacles elements (wall & ghosts)
        :return: set of possible movements
        """
        move_set = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        next_move_set = []
        for elem in move_set:
            if not self.check_collision(start_point[0] + elem[0], start_point[1] + elem[1], w, h, obs):
                next_move_set.append(elem)
        return next_move_set

    def check_elem(self, nms, nmsfp):
        """
        This function is used to check if there is a direction change (node) inside a corridor
        :param nms: set of next moves given at the beginning of the corridor
        :param nmsfp: current set of next moves
        :return: True if a node has been reached
        """
        for elem in nmsfp:
            if elem not in nms:
                return True
        return False

    def check_destination(self, dest, cand, i):
        """
        :param dest: pacman destination point
        :param cand: current point explored
        :param i: precision
        :return: True if the destination is reached in the graph
        """
        if cand[0] + i > dest[0] > cand[0] - i and cand[1] + i > dest[1] > cand[1] - i:
            print('trou')
            print(cand)
            return True
        else:
            return False

    def graph(self, player, obstacles, destination, graph_list, start_point, dicti={0: []}, depth=0):
        """
        :param player: pacman player object
        :param obstacles: list of obstacles (walls & ghosts)
        :param destination: destination point
        :param graph_list: list of current nodes explored (starts empty)
        :param start_point: starting position (of previous node)
        :param dicti: dictionnary linking each node between them and the related distance cost
        :param depth: current depth reached
        :return: list of nodes explored and relation dictionnary
        """
        dic_index = graph_list.index(start_point)
        if dic_index not in dicti:
            dicti[dic_index] = []
        pacman_x, pacman_y, w, h = player.rect()
        next_move_set = self.get_next_moves(start_point, w, h, obstacles)
        next_point_set = []
        for elem in next_move_set:
            i = 1
            while i < 300:
                next_point = (start_point[0] + i * elem[0], start_point[1] + i * elem[1])
                next_move_set_from_point = self.get_next_moves(next_point, w, h, obstacles)
                if self.check_elem(next_move_set, next_move_set_from_point) \
                        or self.check_collision(next_point[0], next_point[1], w, h,
                                                obstacles) :#or self.check_destination(destination, next_point, 1):
                    break
                i += 1
            if next_point not in graph_list and depth < 5:
                graph_list.append(next_point)
                next_point_set.append(next_point)
            if next_point in graph_list:
                dicti[dic_index].append([graph_list.index(next_point), i])
            if destination not in graph_list:
                next_point_set.append(destination)
                graph_list.append(destination)
        if depth < 5:
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

    def get_next_dir(self, graph, destination, nodes_list, current_pos):
        """
        :param graph: nodes (dictionnary)
        :param destination_key : key of the destination in the graph (source is 0)
        :return: next move
        """
        # we'll try to find to shortest path from pacman position to destination
        destination_key = nodes_list.index(destination)
        dist, parent = self.lazy_dijkstras(graph, 0)
        next_node = self.next_node(parent, destination_key)
        next_point = nodes_list[next_node]
        dir = (next_point[0] - current_pos[0], next_point[1] - current_pos[1])
        norm = np.max(np.abs(dir))
        return dir / norm

    def action_from_state(self, obs, player):
        """
        :param obs: current game state (arena.actors, list)
        :param player: pacman object
        :return: selected move
        """

        global count
        pacman_x, pacman_y, w, h = player.rect()
        print(f'Pacman position, x={pacman_x}, y={pacman_y}')  # debug

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

        # example list containing manathan distance from pacman to each ghost
        dist_ghost = [d[0] + d[1] for d in
                      np.abs([np.subtract(elem.get_pos(), (pacman_x, pacman_y)) for elem in ghosts])]
        # example
        for count, elem in enumerate(dist_ghost):
            if elem < 50:
                print(
                    f'ghost is near at dist = {elem}, ghost number = {count}, at position = {ghosts[count].get_pos()}')

        """
        PUT CODE HERE TO PROCESS THE INFORMATIONS (GHOSTS, WALLS, FOOD) based on tree rules
        Then gives the destination you want pacman to go
        """

        # point on the map where you want to move pacman
        destination = (48, 7)  # destination = (175, 208) # example
        print(f'Pacman goal is, x={destination[0]}, y={destination[1]}')
        # generates a graph from the map
        self.graph_list, self.graph_dict = self.graph(player, obstacles=walls, destination=destination, dicti={},
                                                      graph_list=[(pacman_x, pacman_y)],
                                                      start_point=(pacman_x, pacman_y))

        # return the next move to reach the destination (from dijkstra shortest path)
        next_move = self.get_next_dir(self.graph_dict, destination, self.graph_list, (pacman_x, pacman_y))
        action_dx, action_dy = int(next_move[0]), int(next_move[1])

        # 4 direction possible --> 1,0 | 0,1 | -1,0 | 0,-1 = right | up | left | down
        # example
        # action_dx, action_dy = -1, 0  # move pacman left
        self.count += 1
        return action_dx, action_dy
