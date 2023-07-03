import argparse
import copy
import heapq
import math
import os
import random
import time

import numpy as np

import PlotDraw
from GridEnvironment import GridEnv
from KicadParser import grid_parameters
# from ProblemParser import grid_parameters
from utils import diagonal_distance_3d, circle_space, rect_space, diagonal_distance_2d
from RouteEvaluation import route_length, via_amount, min_via_amount


class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def push(self, item, priority):
        # heappush insert an element to _queue
        heapq.heappush(self._queue, (-priority, self._index, item))
        self._index += 1

    def pop(self):
        # heappop delete an element from _queue
        return heapq.heappop(self._queue)[-1]

    def is_empty(self):
        return self._queue == []


class Item:
    def __init__(self, cur_pos, start_pos, start_pos_set, end_pos, end_pos_set, g_score, route_cost, parent):
        self.cur_pos = cur_pos
        self.start_pos = start_pos
        self.start_pos_set = start_pos_set
        self.end_pos = end_pos
        self.end_pos_set = end_pos_set

        self.h_score = diagonal_distance_3d(cur_pos, end_pos)
        self.g_score = g_score + route_cost

        self.f_score = self.g_score + self.h_score
        self.min_f_score = diagonal_distance_3d(start_pos, end_pos)

        self.parent = parent

    def __lt__(self, other):
        return self.f_score < other.f_score

    # def __eq__(self, other):
    #     return self.cur_pos == other.cur_pos

    def set_g_score(self, g_score_with_cost):
        self.g_score = g_score_with_cost
        self.f_score = self.g_score + self.h_score

    def get_neighbors(self, occupied_msg, space_cost_graph, electrical_width, net_pin_set, grid_size):  # TODO
        x, y, z = self.cur_pos
        direct = None
        if self.parent is not None:
            x_parent, y_parent, z_parent = self.parent.cur_pos
            direct = [x - x_parent, y - y_parent, z - z_parent]
        neighbors = []
        # For Debug
        # if self.cur_pos == [204, 233, 0] or self.cur_pos == [202, 233, 0] or self.cur_pos == [203, 234, 0] or \
        #         self.cur_pos == [203, 232, 0] or self.cur_pos == [202, 232, 0] or self.cur_pos == [202, 234, 0] or \
        #         self.cur_pos == [204, 232, 0] or self.cur_pos == [204, 234, 0] or self.cur_pos == [203, 233, 1]:
        #     print('breakpoint')
        # go to the east
        if x < grid_size[0] - 1 and direct != [-1, 0, 0]:
            pos = [x + 1, y, z]
            g_cost = 0
            if direct is not None and direct != [1, 0, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-north
        if x < grid_size[0] - 1 and y < grid_size[1] - 1 and direct != [-1, -1, 0]:
            pos = [x + 1, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [1, 1, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1.414
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the north
        if y < grid_size[1] - 1 and direct != [0, -1, 0]:
            pos = [x, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [0, 1, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-north
        if x > 0 and y < grid_size[1] - 1 and direct != [1, -1, 0]:
            pos = [x - 1, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [-1, 1, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1.414
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the west
        if x > 0 and direct != [1, 0, 0]:
            pos = [x - 1, y, z]
            g_cost = 0
            if direct is not None and direct != [-1, 0, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-south
        if x > 0 and y > 0 and direct != [1, 1, 0]:
            pos = [x - 1, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [-1, -1, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1.414
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the south
        if y > 0 and direct != [0, 1, 0]:
            pos = [x, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [0, -1, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-south
        if x < grid_size[0] - 1 and y > 0 and direct != [-1, 1, 0]:
            pos = [x + 1, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [1, -1, 0]:
                g_cost = 0.1  # bend cost
            if str(pos) in net_pin_set:
                g_cost = -1.414
            # elif pos != self.end_pos:
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.track_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = track_space_cost
            #     g_cost += space_cost
            elif str(pos) not in self.end_pos_set:
                g_cost += calculate_space(pos, electrical_width[0], occupied_msg)  # TODO
            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)

        # go to upside through a via
        if z < grid_size[2] - 1 and direct != [0, 0, -1]:
            pos = [x, y, z + 1]
            g_cost = 9  # go through a via need a high cost
            if str(pos) in net_pin_set:
                g_cost = -1
            # elif pos != self.end_pos:
            #     g_cost += space_cost_graph.via_space_cost(self.cur_pos)
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.via_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)  # TODO
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = via_space_cost
            #     g_cost += space_cost

            # elif str(pos) not in self.end_pos_set:
            #     g_cost += calculate_space(pos, electrical_width[1], occupied_msg)
            #     g_cost += calculate_space(self.cur_pos, electrical_width[1], occupied_msg)

            g_cost += calculate_space(pos, electrical_width[1], occupied_msg)
            g_cost += calculate_space(self.cur_pos, electrical_width[1], occupied_msg)

            distance_1 = diagonal_distance_2d(self.cur_pos, self.end_pos)
            distance_2 = diagonal_distance_2d(self.cur_pos, self.start_pos)
            via_distance_1 = 2 * math.sqrt(len(self.end_pos_set))
            via_distance_2 = math.sqrt(len(self.start_pos_set))
            if distance_1 < via_distance_1 or distance_2 < via_distance_2:
                g_cost += 1000

            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to downside through a via
        if z > 0 and direct != [0, 0, 1]:
            pos = [x, y, z - 1]
            g_cost = 9  # go through a via need a high cost
            if str(pos) in net_pin_set:
                g_cost = -1
            # elif pos != self.end_pos:
            #     g_cost += space_cost_graph.via_space_cost(self.cur_pos)
            #     if space_cost_graph.is_extended(pos):
            #         space_cost = space_cost_graph.via_space_cost(pos)
            #     else:
            #         track_space_cost = calculate_space(pos, electrical_width[0], occupied_msg)
            #         via_space_cost = calculate_space(pos, electrical_width[1], occupied_msg)  # TODO
            #         space_cost_graph.extend_grid(pos, track_space_cost, via_space_cost)
            #         space_cost = via_space_cost
            #     g_cost += space_cost

            # elif str(pos) not in self.end_pos_set:
            #     g_cost += calculate_space(pos, electrical_width[1], occupied_msg)
            #     g_cost += calculate_space(self.cur_pos, electrical_width[1], occupied_msg)

            g_cost += calculate_space(pos, electrical_width[1], occupied_msg)
            g_cost += calculate_space(self.cur_pos, electrical_width[1], occupied_msg)

            distance_1 = diagonal_distance_2d(self.cur_pos, self.end_pos)
            distance_2 = diagonal_distance_2d(self.cur_pos, self.start_pos)
            via_distance_1 = 2 * math.sqrt(len(self.end_pos_set))
            via_distance_2 = math.sqrt(len(self.start_pos_set))
            if distance_1 < via_distance_1 or distance_2 < via_distance_2:
                g_cost += 1000

            # elif pos != self.end_pos and str(pos) in occupied_msg:
            #     g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.start_pos, self.start_pos_set, self.end_pos, self.end_pos_set, self.g_score, g_cost + 1, self)
            neighbors.append(item)

        return neighbors

    def get_neighbors_v1(self, grid_graph, net_pin_set, grid_size):
        x, y, z = self.cur_pos
        direct = None
        if self.parent is not None:
            x_parent, y_parent, z_parent = self.parent.cur_pos
            direct = [x - x_parent, y - y_parent, z - z_parent]
        neighbors = []
        # go to the east
        if x < grid_size[0] - 1:
            next_x = x + 1
            next_y = y
            next_z = z
            g_cost = 0
            if direct is not None and direct != [1, 0, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-north
        if x < grid_size[0] - 1 and y < grid_size[1] - 1:
            next_x = x + 1
            next_y = y + 1
            next_z = z
            g_cost = 0
            if direct is not None and direct != [1, 1, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1.414
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the north
        if y < grid_size[1] - 1:
            next_x = x
            next_y = y + 1
            next_z = z
            g_cost = 0
            if direct is not None and direct != [0, 1, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-north
        if x > 0 and y < grid_size[1] - 1:
            next_x = x - 1
            next_y = y + 1
            next_z = z
            g_cost = 0
            if direct is not None and direct != [-1, 1, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1.414
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the west
        if x > 0:
            next_x = x - 1
            next_y = y
            next_z = z
            g_cost = 0
            if direct is not None and direct != [-1, 0, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-south
        if x > 0 and y > 0:
            next_x = x - 1
            next_y = y - 1
            next_z = z
            g_cost = 0
            if direct is not None and direct != [-1, -1, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1.414
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the south
        if y > 0:
            next_x = x
            next_y = y - 1
            next_z = z
            g_cost = 0
            if direct is not None and direct != [0, -1, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-south
        if x < grid_size[0] - 1 and y > 0:
            next_x = x + 1
            next_y = y - 1
            next_z = z
            g_cost = 0
            if direct is not None and direct != [1, -1, 0]:
                g_cost = 0.1  # bend cost
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -1.414
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)

        # go to upside through a via
        if z < grid_size[2] - 1:
            next_x = x
            next_y = y
            next_z = z + 1
            g_cost = 0
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -10
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score,
                        g_cost + 10, self)  # go through a via need a high cost
            neighbors.append(item)
        # go to downside through a via
        if z > 0:
            next_x = x
            next_y = y
            next_z = z - 1
            g_cost = 0
            if str([next_x, next_y, next_z]) in net_pin_set:
                g_cost = -10
            elif [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score,
                        g_cost + 10, self)  # go through a via need a high cost
            neighbors.append(item)

        return neighbors


class CostGraph:
    def __init__(self, grid_size):
        self.track_space_cost_graph = (- np.ones(grid_size)).tolist()
        self.via_space_cost_graph = (- np.ones(grid_size)).tolist()

    def is_extended(self, pos):
        x, y, z = pos
        return self.track_space_cost_graph[x][y][z] != -1

    def extend_grid(self, pos, track_cost, via_cost):
        x, y, z = pos
        self.track_space_cost_graph[x][y][z] = track_cost
        self.via_space_cost_graph[x][y][z] = via_cost

    def track_space_cost(self, pos):
        x, y, z = pos
        return self.track_space_cost_graph[x][y][z]

    def via_space_cost(self, pos):
        x, y, z = pos
        return self.via_space_cost_graph[x][y][z]


def generate_path(end_item):
    path = []
    g_cost = end_item.f_score
    while end_item.parent is not None:
        path.append(end_item.cur_pos)
        end_item = end_item.parent
    path.append(end_item.cur_pos)
    return path, g_cost


def calculate_space(pos, radius, occupied_msg):
    cached_trace_cost = 0
    pos_list = rect_space(pos, radius, radius)
    for coord in pos_list:
        if str(coord) in occupied_msg:
            cached_trace_cost += occupied_msg[str(coord)]
    return cached_trace_cost


global get_neighbors_time
global add_neighbors_time

global get_neighbors_num


def a_star_route(start, end, end_set, occupied_msg, electrical_width, net_pin_set, grid_size):  # TODO
    # arg = solver_arguments()
    # if arg.trace:
    global get_neighbors_time, add_neighbors_time, get_neighbors_num
    open_set = []
    open_set_graph = np.zeros(grid_size).tolist()
    space_cost_graph = CostGraph(grid_size)
    # closed_set = []
    closed_set = set([])

    for pos in start:
        start_pos = list(int(char) for char in pos.strip('[]').split(', '))
        start_item = Item(start_pos, start_pos, start, end, end_set, 0.0, 0.0, None)
        heapq.heappush(open_set, start_item)
        open_set_graph[start_pos[0]][start_pos[1]][start_pos[2]] = start_item
    # start_item = Item(start, end, 0.0, 0.0, None)
    # heapq.heappush(open_set, start_item)

    # For Debug
    debug_log = open('debug.txt', 'w', encoding="utf-8")
    while open_set:
        cur_item = heapq.heappop(open_set)
        # For Debug
        # if cur_item.cur_pos == [203, 233, 0]:
        #     print('breakpoint')
        print("cur_pos = {}".format(cur_item.cur_pos), file=debug_log)
        print("route = {}".format(generate_path(cur_item)), file=debug_log)
        # For Debug
        # if cur_item.cur_pos[2] == 1:
        #     print('breakpoint bottom layer')
        # if cur_item.cur_pos[2] == 0:
        #     print('breakpoint top layer')

        if str(cur_item.cur_pos) in end_set or cur_item.f_score > 1000 + cur_item.min_f_score:
            return generate_path(cur_item)
        else:
            # closed_set.append(cur_item)
            closed_set.add(str(cur_item.cur_pos))

            if arg.trace:
                get_neighbors_time_start = time.time()
                neighbor_list = cur_item.get_neighbors(occupied_msg, space_cost_graph, electrical_width,
                                                       net_pin_set, grid_size)  # TODO
                get_neighbors_time_end = time.time()
                get_neighbors_time += get_neighbors_time_end - get_neighbors_time_start
                get_neighbors_num += 1
            else:
                neighbor_list = cur_item.get_neighbors(occupied_msg, space_cost_graph, electrical_width,
                                                       net_pin_set, grid_size)  # TODO

            if arg.trace:
                add_neighbors_time_start = time.time()
                for neighbor in neighbor_list:
                    # if neighbor in closed_set:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                        if item != 0:
                            if neighbor.g_score < item.g_score:
                                item.set_g_score(neighbor.g_score)
                                item.parent = neighbor.parent
                                heapq.heapify(open_set)
                        else:
                            heapq.heappush(open_set, neighbor)
                            open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor
                    #     flag = True
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             flag = False
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    #             break
                    #     if flag:
                    #         heapq.heappush(open_set, neighbor)
                    # elif neighbor in open_set:
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    # else:
                    #     heapq.heappush(open_set, neighbor)
                add_neighbors_time_end = time.time()
                add_neighbors_time += add_neighbors_time_end - add_neighbors_time_start
            else:
                for neighbor in neighbor_list:
                    # if neighbor in closed_set:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                        if item != 0:
                            if neighbor.g_score < item.g_score:
                                item.set_g_score(neighbor.g_score)
                                item.parent = neighbor.parent
                                heapq.heapify(open_set)
                        else:
                            heapq.heappush(open_set, neighbor)
                            open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor
                    #     flag = True
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             flag = False
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    #             break
                    #     if flag:
                    #         heapq.heappush(open_set, neighbor)
                    # elif neighbor in open_set:
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    # else:
                    #     heapq.heappush(open_set, neighbor)


def a_star_route_v1(start, end, end_set, grid_graph, net_pin_set, grid_size):
    # arg = solver_arguments()
    # if arg.trace:
    global get_neighbors_time, add_neighbors_time, get_neighbors_num
    open_set = []
    open_set_graph = np.zeros(grid_size).tolist()
    # closed_set = []
    closed_set = set([])

    for pos in start:
        start_pos = list(int(char) for char in pos.strip('[]').split(', '))
        start_item = Item(start_pos, end, 0.0, 0.0, None)
        heapq.heappush(open_set, start_item)
        open_set_graph[start_pos[0]][start_pos[1]][start_pos[2]] = start_item
    # start_item = Item(start, end, 0.0, 0.0, None)
    # heapq.heappush(open_set, start_item)

    while open_set:
        cur_item = heapq.heappop(open_set)
        if str(cur_item.cur_pos) in end_set:
            return generate_path(cur_item)
        else:
            # closed_set.append(cur_item)
            closed_set.add(str(cur_item.cur_pos))

            if arg.trace:
                get_neighbors_time_start = time.time()
                neighbor_list = cur_item.get_neighbors_v1(grid_graph, net_pin_set, grid_size)
                get_neighbors_time_end = time.time()
                get_neighbors_time += get_neighbors_time_end - get_neighbors_time_start
                get_neighbors_num += 1
            else:
                neighbor_list = cur_item.get_neighbors_v1(grid_graph, net_pin_set, grid_size)

            if arg.trace:
                add_neighbors_time_start = time.time()
                for neighbor in neighbor_list:
                    # if neighbor in closed_set:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                        if item != 0:
                            if neighbor.g_score < item.g_score:
                                item.set_g_score(neighbor.g_score)
                                item.parent = neighbor.parent
                                heapq.heapify(open_set)
                        else:
                            heapq.heappush(open_set, neighbor)
                            open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor
                    #     flag = True
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             flag = False
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    #             break
                    #     if flag:
                    #         heapq.heappush(open_set, neighbor)
                    # elif neighbor in open_set:
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    # else:
                    #     heapq.heappush(open_set, neighbor)
                add_neighbors_time_end = time.time()
                add_neighbors_time += add_neighbors_time_end - add_neighbors_time_start
            else:
                for neighbor in neighbor_list:
                    # if neighbor in closed_set:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        item = open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]]
                        if item != 0:
                            if neighbor.g_score < item.g_score:
                                item.set_g_score(neighbor.g_score)
                                item.parent = neighbor.parent
                                heapq.heapify(open_set)
                        else:
                            heapq.heappush(open_set, neighbor)
                            open_set_graph[neighbor.cur_pos[0]][neighbor.cur_pos[1]][neighbor.cur_pos[2]] = neighbor
                    #     flag = True
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             flag = False
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    #             break
                    #     if flag:
                    #         heapq.heappush(open_set, neighbor)
                    # elif neighbor in open_set:
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    # else:
                    #     heapq.heappush(open_set, neighbor)


def solver_arguments():
    parser = argparse.ArgumentParser('AStarSolver')
    parser.add_argument('--type', type=int, dest='type', default=0)
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

    # benchmark_dir = 'benchmark'
    # benchmark_i = 0
    #
    # for benchmark_file in os.listdir(benchmark_dir):
    #     benchmark_file = benchmark_dir + '/' + benchmark_file
    benchmark_file = arg.kicad_pcb
    project_file = arg.kicad_pro
    save_file = arg.save_file
    gridParameters = grid_parameters(benchmark_file, project_file, save_file)
    gridEnv = GridEnv(gridParameters, arg.type)

    # if arg.trace:
    start_time = time.time()  # Record starting time
    route_time = 0
    get_neighbors_time = 0
    add_neighbors_time = 0
    modify_time = 0

    get_neighbors_num = 0

    while gridEnv.episode < arg.episode:
        if arg.type == 0:
            gridEnv.reset()
        else:
            gridEnv.reset_v1()
        if gridEnv.episode == arg.episode:
            PlotDraw.draw_cost_plot(gridEnv.episode_cost)
            PlotDraw.draw_origin_grid_plot(gridEnv)
            PlotDraw.draw_grid_plot(gridEnv)

            gridParameters.store_route(gridEnv.merge_route())

        if gridEnv.episode == arg.episode:
            break

        if arg.trace:
            modify_time_start = time.time()  # Record modify starting time
            if arg.type == 0:
                gridEnv.breakup()
            else:
                gridEnv.breakup_v1()
            modify_time_end = time.time()  # Record modify ending time
            modify_time += modify_time_end - modify_time_start
        else:
            if arg.type == 0:
                gridEnv.breakup()
            else:
                gridEnv.breakup_v1()

        t1 = time.time()
        if gridEnv.init_pos is not None:
            if arg.trace:
                route_time_start = time.time()  # Record route starting time
                if arg.type == 0:
                    route, cost = a_star_route(gridEnv.init_pos, gridEnv.goal_pos,  # TODO
                                               gridEnv.pin_grid_set[str(gridEnv.goal_pos)],
                                               gridEnv.occupied_coord, gridEnv.electric_width[gridEnv.multiPinNet_i],
                                               gridEnv.netPinSet, gridEnv.grid_size)
                else:
                    route, cost = a_star_route_v1(gridEnv.init_pos, gridEnv.goal_pos,
                                                  gridEnv.pin_grid_set[str(gridEnv.goal_pos)],
                                                  gridEnv.grid_graph, gridEnv.netPinSet, gridEnv.grid_size)
                route_time_end = time.time()  # Record route ending time
                route_time += route_time_end - route_time_start
            else:
                if arg.type == 0:
                    route, cost = a_star_route(gridEnv.init_pos, gridEnv.goal_pos,  # TODO
                                               gridEnv.pin_grid_set[str(gridEnv.goal_pos)],
                                               gridEnv.occupied_coord, gridEnv.electric_width[gridEnv.multiPinNet_i],
                                               gridEnv.netPinSet, gridEnv.grid_size)
                else:
                    route, cost = a_star_route_v1(gridEnv.init_pos, gridEnv.goal_pos,
                                                  gridEnv.pin_grid_set[str(gridEnv.goal_pos)],
                                                  gridEnv.grid_graph, gridEnv.netPinSet, gridEnv.grid_size)
        else:
            route = []
            cost = 0
        t2 = time.time()
        print('route time = {} s'.format(t2 - t1))

        # gridEnv.route = route
        # gridEnv.set_route(route)
        # gridEnv.cost = cost

        if arg.trace:
            modify_time_start = time.time()  # Record modify starting time
            if arg.type == 0:
                gridEnv.update(route, cost)
            else:
                gridEnv.update_v1(route, cost)
            modify_time_end = time.time()  # Record modify ending time
            modify_time += modify_time_end - modify_time_start
        else:
            if arg.type == 0:
                gridEnv.update(route, cost)
            else:
                gridEnv.update_v1(route, cost)

    #     benchmark_i += 1
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
