from arena import *
from pacman import *
from map_graph import *
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
        print(dist, parent)
        return dist, parent

    def check_between(self, val_1, val_2, check):
        if val_1 >= check >= val_2 or val_2 >= check >= val_1:
            return True
        else:
            return False

    def get_index_connection(self, graph_list, graph_dict, position):

        for key in graph_dict.keys():
            node_1 = graph_list[key]
            for elem in graph_dict[key]:
                node_2 = graph_list[elem[0]]
                if (node_1[0] == node_2[0] == position[0] and self.check_between(node_1[1], node_2[1], position[1])) or \
                        (node_1[1] == node_2[1] == position[1] and self.check_between(node_1[0], node_2[0],
                                                                                      position[0])) and not (
                        key == 13 and elem[0] == 46) and not (key == 46 and elem[0] == 13):
                    return (key, elem[0])
        return None, None

    def trim_graph(self, start, ghosts, destination, graph_list, graph_dict):
        # step 1 : removes connection between nodes where a ghost is
        for ghost_pos in [ghost.get_pos() for ghost in ghosts]:
            from_, to_ = self.get_index_connection(graph_list, graph_dict, ghost_pos)
            if from_ is not None:
                new_ = []
                for e in graph_dict[from_]:
                    if e[0] != to_:
                        new_.append(e)
                graph_dict[from_] = new_
                new_ = []
                for e in graph_dict[to_]:
                    if e[0] != from_:
                        new_.append(e)
                graph_dict[to_] = new_
        # step 2 : add destination node to graph
        from_, to_ = self.get_index_connection(graph_list, graph_dict, destination)
        node_1, node_2 = graph_list[from_], graph_list[to_]
        cost_1, cost_2 = abs(destination[0] - node_1[0]) + abs(destination[1] - node_1[1]), abs(
            node_2[0] - destination[0]) + abs(node_2[1] - destination[1])
        graph_list.append(destination)
        new_index = len(graph_list) - 1
        graph_dict[new_index] = [[from_, cost_1], [to_, cost_2]]
        graph_dict[from_] = graph_dict[from_] + [[new_index, cost_1]]
        graph_dict[to_] = graph_dict[to_] + [[new_index, cost_2]]
        new_ = []
        for e in graph_dict[from_]:
            if e[0] != to_:
                new_.append(e)
        graph_dict[from_] = new_
        new_ = []
        for e in graph_dict[to_]:
            if e[0] != from_:
                new_.append(e)
        graph_dict[to_] = new_

        # step 3 : add starting node to graph

        from_, to_ = self.get_index_connection(graph_list, graph_dict, start)
        node_1, node_2 = graph_list[from_], graph_list[to_]
        cost_1, cost_2 = abs(start[0] - node_1[0]) + abs(start[1] - node_1[1]), abs(
            node_2[0] - start[0]) + abs(node_2[1] - start[1])
        graph_list.append(start)
        new_index = len(graph_list) - 1
        graph_dict[new_index] = [[from_, cost_1], [to_, cost_2]]
        graph_dict[from_] = graph_dict[from_] + [[new_index, cost_1]]
        graph_dict[to_] = graph_dict[to_] + [[new_index, cost_2]]


        return graph_list, graph_dict

    def get_next_dir(self, graph, destination, nodes_list_, current_pos):
        """
        :param current_pos:
        :param nodes_list:
        :param destination:
        :param graph: nodes (dictionnary)
        :param destination_key : key of the destination in the graph (source is 0)
        :return: next move
        """
        # we'll try to find to shortest path from pacman position to destination
        destination_key = nodes_list_.index(destination)
        root_key = nodes_list_.index(current_pos)
        dist, parent = self.lazy_dijkstras(graph, root_key)
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
        bonus = []
        for elem in obs:
            if isinstance(elem, Ghost):
                ghosts.append(elem)
            elif isinstance(elem, Cookie):
                food.append(elem)
            elif isinstance(elem, Wall):
                walls.append(elem)
            elif isinstance(elem, Bonus):
                # when pacman eats a bonus, he enters a bonus state (check with pacman.bonus_sprite) , it lasts a few
                # seconds during which he cant eat ghosts
                bonus.append(elem)
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
        destination = (40, 232)  # destination = (48, 8)  # destination = (208, 208) # example
        print(f'Pacman goal is, x={destination[0]}, y={destination[1]}')
        # generates a graph from the map
        self.graph_list, self.graph_dict = self.trim_graph((pacman_x, pacman_y), ghosts=ghosts, destination=destination,
                                                           graph_list=nodes_list, graph_dict=nodes_dic)
        print(nodes_dic)

        # return the next move to reach the destination (from dijkstra shortest path)
        next_move = self.get_next_dir(self.graph_dict, destination, self.graph_list, (pacman_x, pacman_y))
        action_dx, action_dy = int(next_move[0]), int(next_move[1])

        # 4 direction possible --> 1,0 | 0,1 | -1,0 | 0,-1 = right | up | left | down
        # example
        # action_dx, action_dy = -1, 0  # move pacman left
        self.count += 1
        return action_dx, action_dy
