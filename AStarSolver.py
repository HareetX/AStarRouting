import argparse
import heapq
import time

import numpy as np

import PlotDraw
from GridEnvironment import GridEnv
from KicadParser import grid_parameters
# from ProblemParser import grid_parameters
from utils import diagonal_distance_3d, circle_space, rect_space, diagonal_distance_2d, via_cost_function_3
from RouteEvaluation import route_length, via_amount, min_via_amount


class CostGraph:
    def __init__(self, grid_size: list):
        self.track_space_cost_graph = (- np.ones(grid_size)).tolist()
        """ store the space cost calculated by track width and clearance, the undefined space cost is -1 """
        self.via_space_cost_graph = (- np.ones(grid_size)).tolist()
        """ store the space cost calculated by microvia width and clearance, the undefined space cost is -1 """

    def is_extended(self, pos: list) -> bool:
        x, y, z = pos
        return self.track_space_cost_graph[x][y][z] != -1

    def extend_grid(self, pos: list, track_cost: float, via_cost: float):
        x, y, z = pos
        self.track_space_cost_graph[x][y][z] = track_cost
        self.via_space_cost_graph[x][y][z] = via_cost

    def track_space_cost(self, pos: list) -> float:
        x, y, z = pos
        return self.track_space_cost_graph[x][y][z]

    def via_space_cost(self, pos: list) -> float:
        x, y, z = pos
        return self.via_space_cost_graph[x][y][z]


def calculate_space(pos, radius, occupied_msg):
    cached_trace_cost = 0
    pos_list = rect_space(pos, radius, radius)
    for coord in pos_list:
        if str(coord) in occupied_msg:
            cached_trace_cost += occupied_msg[str(coord)]
    return cached_trace_cost


class Item:
    """
    Item is the node of A* Algorithm Searching Tree
    """
    def __init__(self, cur_pos: list, start_pos: list, start_pos_set: set, end_pos: list, end_pos_set: set,
                 g_score: float, route_cost: float, parent):
        self.cur_pos = cur_pos
        """ the coordinate of the current position """
        self.start_pos = start_pos
        """ the coordinate of the starting position """
        self.start_pos_set = start_pos_set
        """ the set of the starting positions """
        self.end_pos = end_pos
        """ the coordinate of the ending position """
        self.end_pos_set = end_pos_set
        """ the set of the ending positions """

        self.h_score = diagonal_distance_3d(cur_pos, end_pos)
        self.g_score = g_score + route_cost

        self.f_score = self.g_score + self.h_score
        """ f(x) = g(x) + h(x) """
        self.min_f_score = diagonal_distance_3d(start_pos, end_pos)
        """ the theoretical value of f(x) """

        self.parent = parent
        """ parent node of current node """

    def __lt__(self, other) -> bool:
        return self.f_score < other.f_score

    def set_g_score(self, g_score_with_cost: float):
        """
        set new g_score and recalculate the f_score

        :param g_score_with_cost: new g_score
        :return:
        """
        self.g_score = g_score_with_cost
        self.f_score = self.g_score + self.h_score

    def get_neighbors(self, occupied_msg: dict, pin_hsize_array: list, net_pin_list: list,
                      space_cost_graph: CostGraph, electrical_width: list, grid_size: list) -> list:
        """
        create neighbors of current item, return the list of the neighbors

        :param occupied_msg: the dict of the occupied grids
        :param pin_hsize_array: the half-size list of pads from current net
                                the element include x_hsize, y_hsize, pad_shape
        :param net_pin_list: the center position list of pads from current net
        :param space_cost_graph: a CostGraph that stores the space cost
        :param electrical_width: the first element is calculated by track width and clearance
                                 the second element is calculated by microvia width and clearance
        :param grid_size: define the size of the grid graph
        :return: the list of the neighbors
        """
        x, y, z = self.cur_pos

        direct = None
        if self.parent is not None:
            x_parent, y_parent, z_parent = self.parent.cur_pos
            direct = [x - x_parent, y - y_parent, z - z_parent]

        neighbors = []
        # go to the east
        if x < grid_size[0] - 1 and direct != [-1, 0, 0]:
            pos = [x + 1, y, z]
            g_cost = 0

            if direct is not None and direct != [1, 0, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-north
        if x < grid_size[0] - 1 and y < grid_size[1] - 1 and direct != [-1, -1, 0]:
            pos = [x + 1, y + 1, z]
            g_cost = 0

            if direct is not None and direct != [1, 1, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1.414

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the north
        if y < grid_size[1] - 1 and direct != [0, -1, 0]:
            pos = [x, y + 1, z]
            g_cost = 0

            if direct is not None and direct != [0, 1, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-north
        if x > 0 and y < grid_size[1] - 1 and direct != [1, -1, 0]:
            pos = [x - 1, y + 1, z]
            g_cost = 0

            if direct is not None and direct != [-1, 1, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1.414

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the west
        if x > 0 and direct != [1, 0, 0]:
            pos = [x - 1, y, z]
            g_cost = 0

            if direct is not None and direct != [-1, 0, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-south
        if x > 0 and y > 0 and direct != [1, 1, 0]:
            pos = [x - 1, y - 1, z]
            g_cost = 0

            if direct is not None and direct != [-1, -1, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1.414

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the south
        if y > 0 and direct != [0, 1, 0]:
            pos = [x, y - 1, z]
            g_cost = 0

            if direct is not None and direct != [0, -1, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-south
        if x < grid_size[0] - 1 and y > 0 and direct != [-1, 1, 0]:
            pos = [x + 1, y - 1, z]
            g_cost = 0

            if direct is not None and direct != [1, -1, 0]:
                g_cost = 0.1  # bend cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1.414

            if str(pos) not in self.end_pos_set:
                if space_cost_graph.is_extended(pos):
                    space_cost = space_cost_graph.track_space_cost(pos)
                else:
                    track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                    via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                    space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                    space_cost = track_space_cost
                g_cost += space_cost

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1.414, self)
            neighbors.append(item)

        # go to upside through a via
        if z < grid_size[2] - 1 and direct != [0, 0, -1]:
            pos = [x, y, z + 1]
            g_cost = 9  # go through a via need a high cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1

            g_cost += space_cost_graph.via_space_cost(self.cur_pos)
            if space_cost_graph.is_extended(pos):
                space_cost = space_cost_graph.via_space_cost(pos)
            else:
                track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                space_cost = via_space_cost
            g_cost += space_cost

            g_cost_list = []
            for i in range(len(pin_hsize_array)):
                pin_hsize = pin_hsize_array[i]
                pin_pos = net_pin_list[i]
                if pin_hsize[2] == 'circle':
                    distance = diagonal_distance_2d(self.cur_pos, pin_pos)
                    g_cost_list.append(via_cost_function_3(distance, pin_hsize[0]))
                else:
                    relative_x = self.cur_pos[0] - pin_pos[0]
                    relative_y = self.cur_pos[1] - pin_pos[1]
                    cost_x = via_cost_function_3(relative_x, pin_hsize[0])
                    cost_y = via_cost_function_3(relative_y, pin_hsize[1])
                    if cost_x == 1000 and cost_y == 1000:
                        g_cost_list.append(1000)
                    else:
                        if cost_x < 1000 and cost_y < 1000:
                            g_cost_list.append((cost_x + cost_y) * 0.5)
                        elif cost_x < 1000:
                            g_cost_list.append(cost_x)
                        else:
                            g_cost_list.append(cost_y)

            flag = False
            count = 0
            for tmp_cost in g_cost_list:
                if tmp_cost == 1000:
                    flag = True
                if tmp_cost > 1:
                    count += 1

            if flag:
                g_cost += 1000
            else:
                if count == 0:
                    count = 1
                g_cost += sum(g_cost_list) / count

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to downside through a via
        if z > 0 and direct != [0, 0, 1]:
            pos = [x, y, z - 1]
            g_cost = 9  # go through a via need a high cost

            # if str(pos) in net_pin_set:
            #     g_cost = -1

            g_cost += space_cost_graph.via_space_cost(self.cur_pos)
            if space_cost_graph.is_extended(pos):
                space_cost = space_cost_graph.via_space_cost(pos)
            else:
                track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
                via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
                space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
                space_cost = via_space_cost
            g_cost += space_cost

            g_cost_list = []
            for i in range(len(pin_hsize_array)):
                pin_hsize = pin_hsize_array[i]
                pin_pos = net_pin_list[i]
                if pin_hsize[2] == 'circle':
                    distance = diagonal_distance_2d(self.cur_pos, pin_pos)
                    g_cost_list.append(via_cost_function_3(distance, pin_hsize[0]))
                else:
                    relative_x = self.cur_pos[0] - pin_pos[0]
                    relative_y = self.cur_pos[1] - pin_pos[1]
                    cost_x = via_cost_function_3(relative_x, pin_hsize[0])
                    cost_y = via_cost_function_3(relative_y, pin_hsize[1])
                    if cost_x == 1000 and cost_y == 1000:
                        g_cost_list.append(1000)
                    else:
                        if cost_x < 1000 and cost_y < 1000:
                            g_cost_list.append((cost_x + cost_y) * 0.5)
                        elif cost_x < 1000:
                            g_cost_list.append(cost_x)
                        else:
                            g_cost_list.append(cost_y)

            flag = False
            count = 0
            for tmp_cost in g_cost_list:
                if tmp_cost == 1000:
                    flag = True
                if tmp_cost > 1:
                    count += 1

            if flag:
                g_cost += 1000
            else:
                if count == 0:
                    count = 1
                g_cost += sum(g_cost_list) / count

            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set,
                        self.g_score, g_cost + 1, self)
            neighbors.append(item)

        return neighbors

    def generate_path(self) -> (list, float):
        """
        generate the routing path based on parent node
        :return: routing path, routing cost
        """
        end_item = self
        path = []
        g_cost = end_item.f_score
        while end_item.parent is not None:
            path.append(end_item.cur_pos)
            end_item = end_item.parent
        path.append(end_item.cur_pos)
        return path, g_cost


global get_neighbors_time
global add_neighbors_time

global get_neighbors_num


class AStarSolver:
    def __init__(self, occupied_msg: dict, pin_hsize_array: list, net_pin_list: list, electrical_width: list,
                 grid_size: list):
        """
        create a AStarSolver

        :param occupied_msg: the dict of the occupied grids
        :param pin_hsize_array: the half-size list of pads from current net
                                the element include x_hsize, y_hsize, pad_shape
        :param net_pin_list: the center position list of pads from current net
        :param electrical_width: the first element is calculated by track width and clearance
                                 the second element is calculated by microvia width and clearance
        :param grid_size: define the size of the grid graph
        """
        self.occupied_msg = occupied_msg

        self.pin_hsize_array = pin_hsize_array

        self.net_pin_list = net_pin_list

        self.electrical_width = electrical_width

        self.grid_size = grid_size

        self.space_cost_graph = CostGraph(grid_size)
        """ a CostGraph that stores the space cost """

    def a_star_route(self, start: list, start_set: set, net_pin_set: set, end: list, end_set: set) -> (list, float):
        """
        Use A* Algorithm to find a legal path between net_pin_set and end_set

        :param start: center position of the starting pad
        :param start_set: the set of the positions of the starting pad
        :param net_pin_set: the set of the routed pads and paths from the same net
        :param end: center position of the ending pad
        :param end_set: the set of the positions of the ending pad
        :return: the routing path and cost
        """
        global get_neighbors_time, add_neighbors_time, get_neighbors_num

        open_set = []
        # global index table of open set
        open_set_graph = np.zeros(self.grid_size).tolist()

        closed_set = set([])

        for pos in net_pin_set:
            # convert the start position from string (which is hashable) to list
            start_pos = list(int(char) for char in pos.strip('[]').split(', '))
            # create a starting item and add it to open set and its global index table
            start_item = Item(start_pos, start, start_set, end, end_set, 0.0, 0.0, None)
            heapq.heappush(open_set, start_item)
            open_set_graph[start_pos[0]][start_pos[1]][start_pos[2]] = start_item

            if not self.space_cost_graph.is_extended(start_pos):
                # calculate space costs and store them to cost graph
                track_space_cost = calculate_space(start_pos, self.electrical_width[0], self.occupied_msg)
                via_space_cost = calculate_space(start_pos, self.electrical_width[1], self.occupied_msg)
                self.space_cost_graph.extend_grid(start_pos, track_space_cost, via_space_cost)

        # For Debug
        debug_log = open('debug.txt', 'w', encoding="utf-8")
        while open_set:
            cur_item = heapq.heappop(open_set)
            # For Debug
            print("cur_pos = {}".format(cur_item.cur_pos), file=debug_log)
            print("route = {}".format(cur_item.generate_path()), file=debug_log)

            if str(cur_item.cur_pos) in end_set or cur_item.f_score > 1000 + cur_item.min_f_score:
                # if the solver has found a legal path or cannot find a legal path
                return cur_item.generate_path()
            else:
                closed_set.add(str(cur_item.cur_pos))

                if arg.trace:
                    get_neighbors_time_start = time.time()

                    neighbor_list = cur_item.get_neighbors(self.occupied_msg, self.pin_hsize_array, self.net_pin_list,
                                                           self.space_cost_graph, self.electrical_width,
                                                           self.grid_size)

                    get_neighbors_time_end = time.time()
                    get_neighbors_time += get_neighbors_time_end - get_neighbors_time_start
                    get_neighbors_num += 1
                else:
                    neighbor_list = cur_item.get_neighbors(self.occupied_msg, self.pin_hsize_array, self.net_pin_list,
                                                           self.space_cost_graph, self.electrical_width,
                                                           self.grid_size)

                if arg.trace:
                    add_neighbors_time_start = time.time()

                    for neighbor in neighbor_list:
                        if str(neighbor.cur_pos) in closed_set:
                            continue
                        else:
                            item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                            if item != 0:
                                # the current node has expended
                                if neighbor.g_score < item.g_score:
                                    item.set_g_score(neighbor.g_score)
                                    item.parent = neighbor.parent
                                    heapq.heapify(open_set)
                            else:
                                heapq.heappush(open_set, neighbor)
                                open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor

                    add_neighbors_time_end = time.time()
                    add_neighbors_time += add_neighbors_time_end - add_neighbors_time_start
                else:
                    for neighbor in neighbor_list:
                        if str(neighbor.cur_pos) in closed_set:
                            continue
                        else:
                            item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                            if item != 0:
                                # the current node has expended
                                if neighbor.g_score < item.g_score:
                                    item.set_g_score(neighbor.g_score)
                                    item.parent = neighbor.parent
                                    heapq.heapify(open_set)
                            else:
                                heapq.heappush(open_set, neighbor)
                                open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor

        debug_log.close()


def solver_arguments():
    parser = argparse.ArgumentParser('AStarSolver')
    parser.add_argument('--episode', type=int, dest='episode', default=1)
    parser.add_argument('--log', type=str, dest='log', default="log.txt")
    parser.add_argument('--trace', type=bool, dest='trace', default=True)
    parser.add_argument('--kicad_pcb', type=str, dest='kicad_pcb', default="bench2/bm2.unrouted.kicad_pcb")
    parser.add_argument('--kicad_pro', type=str, dest='kicad_pro', default="bench2/bm2.unrouted.kicad_pro")
    parser.add_argument('--save_file', type=str, dest='save_file', default="bench2/bm2.routed.kicad_pcb")

    return parser.parse_args()


if __name__ == '__main__':
    arg = solver_arguments()

    log = open(arg.log, 'w', encoding="utf-8")

    benchmark_file = arg.kicad_pcb
    project_file = arg.kicad_pro
    save_file = arg.save_file
    gridParameters = grid_parameters(benchmark_file, project_file, save_file)

    # For Test : swap routing order
    tmp = gridParameters.netList[1]
    gridParameters.netList[1] = gridParameters.netList[34]
    gridParameters.netList[34] = tmp

    gridEnv = GridEnv(gridParameters)

    start_time = time.time()  # Record starting time
    route_time = 0
    get_neighbors_time = 0
    add_neighbors_time = 0
    modify_time = 0

    get_neighbors_num = 0

    a_star_solver = None
    while gridEnv.episode < arg.episode:

        gridEnv.reset()

        if gridEnv.episode == arg.episode:
            # PlotDraw.draw_cost_plot(gridEnv.episode_cost)

            # write back the routing results
            gridParameters.store_route(gridEnv.merge_route())

        if gridEnv.episode == arg.episode:
            break

        # rip up the path
        if arg.trace:
            modify_time_start = time.time()  # Record modify starting time

            gridEnv.breakup()

            modify_time_end = time.time()  # Record modify ending time
            modify_time += modify_time_end - modify_time_start
        else:
            gridEnv.breakup()

        t1 = time.time()
        if gridEnv.init_pos is not None:
            # For Debug
            # if gridEnv.multiPinNet_i == 23 and gridEnv.twoPinNet_i == 5:
            #     print("breakpoint")

            # initial AStarSolver
            if gridEnv.twoPinNet_i == 0:
                # two-pin nets that belong to the same net share a AStarSolver
                a_star_solver = AStarSolver(gridEnv.occupied_coord, gridEnv.pin_hsize_array[gridEnv.multiPinNet_i],
                                            gridEnv.netlist[gridEnv.multiPinNet_i],
                                            gridEnv.electric_width[gridEnv.multiPinNet_i], gridEnv.grid_size)
            if arg.trace:
                route_time_start = time.time()  # Record route starting time

                route, cost = a_star_solver.a_star_route(gridEnv.init_pos,
                                                         gridEnv.pin_grid_set[str(gridEnv.init_pos)],
                                                         gridEnv.netPinSet,
                                                         gridEnv.goal_pos,
                                                         gridEnv.pin_grid_set[str(gridEnv.goal_pos)])

                route_time_end = time.time()  # Record route ending time
                route_time += route_time_end - route_time_start
            else:
                route, cost = a_star_solver.a_star_route(gridEnv.init_pos,
                                                         gridEnv.pin_grid_set[str(gridEnv.init_pos)],
                                                         gridEnv.netPinSet,
                                                         gridEnv.goal_pos,
                                                         gridEnv.pin_grid_set[str(gridEnv.goal_pos)])
        else:
            route = []
            cost = 0

        t2 = time.time()
        print('route time = {} s'.format(t2 - t1))

        # update the route and cost
        if arg.trace:
            modify_time_start = time.time()  # Record modify starting time

            gridEnv.update(route, cost)

            modify_time_end = time.time()  # Record modify ending time
            modify_time += modify_time_end - modify_time_start
        else:
            gridEnv.update(route, cost)

    end_time = time.time()  # Record ending time

    print("{i} time = {t} s".format(i=benchmark_file, t=end_time - start_time))
    print("\tcost = {c}\n".format(c=gridEnv.route_cost))
    print("{i} time = {t} s".format(i=benchmark_file, t=end_time - start_time), file=log)
    print("\tcost = {c}\n".format(c=gridEnv.route_cost), file=log)
    if arg.trace:
        print("{i}".format(i=benchmark_file))
        print("{i} time = {t} s".format(i=benchmark_file, t=end_time - start_time))
        print("{i} route time = {t} s".format(i=benchmark_file, t=route_time))
        print("\tget neighbors time = {t} s".format(t=get_neighbors_time))
        print("\tget neighbors number = {num}".format(num=get_neighbors_num))
        print("\tadd neighbors time = {t} s".format(t=add_neighbors_time))
        print("{i} modify time = {t} s".format(i=benchmark_file, t=modify_time))
        print("{i} route length = {len}".format(i=benchmark_file, len=route_length(gridEnv.route_combo)))
        print("{i} via amount = {num}\n".format(i=benchmark_file, num=via_amount(gridEnv.route_combo)))
        # print("{i} via amount = {num} / {min}\n".format(i=benchmark_file,
        #                                                 num=via_amount(gridEnv.route_combo),
        #                                                 min=min_via_amount(gridEnv.netlist)))

        print("{i}".format(i=benchmark_file), file=log)
        print("{i} time = {t} s".format(i=benchmark_file, t=end_time - start_time), file=log)
        print("{i} route time = {t} s".format(i=benchmark_file, t=route_time), file=log)
        print("\tget neighbors time = {t} s".format(t=get_neighbors_time), file=log)
        print("\tget neighbors number = {num}".format(num=get_neighbors_num), file=log)
        print("\tadd neighbors time = {t} s".format(t=add_neighbors_time), file=log)
        print("{i} modify time = {t} s\n".format(i=benchmark_file, t=modify_time), file=log)
        print("{i} route length = {len}".format(i=benchmark_file, len=route_length(gridEnv.route_combo)), file=log)
        print("{i} via amount = {num}\n".format(i=benchmark_file, num=via_amount(gridEnv.route_combo)), file=log)
        # print("{i} via amount = {num} / {min}\n".format(i=benchmark_file,
        #                                                 num=via_amount(gridEnv.route_combo),
        #                                                 min=min_via_amount(gridEnv.netlist)), file=log)

    log.close()
