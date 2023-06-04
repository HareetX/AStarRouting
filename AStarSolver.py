import copy
import heapq
import os
import random
import time

import numpy as np

import PlotDraw
from GridEnvironment import GridEnv
from MathFunc import diagonal_distance
from ProblemParser import read, grid_parameters


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
    def __init__(self, cur_pos, end_pos, g_score, cost, parent):
        self.cur_pos = cur_pos
        self.end_pos = end_pos

        self.h_score = diagonal_distance(cur_pos, end_pos)
        self.g_score = g_score + cost

        self.f_score = self.g_score + self.h_score

        self.parent = parent

    def __lt__(self, other):
        return self.f_score < other.f_score

    def __eq__(self, other):
        return self.cur_pos == other.cur_pos

    def set_g_score(self, g_score_with_cost):
        self.g_score = g_score_with_cost
        self.f_score = self.g_score + self.h_score

    def get_neighbors(self, occupied_msg, grid_size):
        x, y, z = self.cur_pos
        direct = None
        if self.parent is not None:
            x_parent, y_parent, z_parent = self.parent.cur_pos
            direct = [x - x_parent, y - y_parent, z - z_parent]
        neighbors = []
        # go to the east
        if x < grid_size[0] - 1:
            pos = [x + 1, y, z]
            g_cost = 0
            if direct is not None and direct != [1, 0, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-north
        if x < grid_size[0] - 1 and y < grid_size[1] - 1:
            pos = [x + 1, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [1, 1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the north
        if y < grid_size[1] - 1:
            pos = [x, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [0, 1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-north
        if x > 0 and y < grid_size[1] - 1:
            pos = [x - 1, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [-1, 1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the west
        if x > 0:
            pos = [x - 1, y, z]
            g_cost = 0
            if direct is not None and direct != [-1, 0, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-south
        if x > 0 and y > 0:
            pos = [x - 1, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [-1, -1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the south
        if y > 0:
            pos = [x, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [0, -1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-south
        if x < grid_size[0] - 1 and y > 0:
            pos = [x + 1, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [1, -1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)

        # go to upside through a via
        if z < grid_size[2] - 1:
            pos = [x, y, z + 1]
            g_cost = 9  # go through a via need a high cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to downside through a via
        if z > 0:
            pos = [x, y, z - 1]
            g_cost = 9  # go through a via need a high cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost = occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)

        return neighbors


def generate_path(end_item):
    path = []
    g_cost = end_item.f_score
    while end_item.parent is not None:
        path.append(end_item.cur_pos)
        end_item = end_item.parent
    path.append(end_item.cur_pos)
    return path, g_cost


def a_star_route(start, end, occupied_msg, grid_size):
    open_set = []
    closed_set = []

    start_item = Item(start, end, 0.0, 0.0, None)
    heapq.heappush(open_set, start_item)

    while open_set:
        cur_item = heapq.heappop(open_set)
        # if cur_item.cur_pos == [18, 12, 0]:
        #     print('good\n')
        if cur_item.cur_pos == end:
            return generate_path(cur_item)
        else:
            closed_set.append(cur_item)

            neighbor_list = cur_item.get_neighbors(occupied_msg, grid_size)
            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                elif neighbor in open_set:
                    for item in open_set:
                        if neighbor == item:
                            if neighbor.g_score < item.g_score:
                                item.set_g_score(neighbor.g_score)
                                item.parent = neighbor.parent
                                heapq.heapify(open_set)
                else:
                    heapq.heappush(open_set, neighbor)


if __name__ == '__main__':
    benchmark_dir = 'benchmark'
    benchmark_i = 0
    for benchmark_file in os.listdir(benchmark_dir):
        benchmark_file = benchmark_dir + '/' + benchmark_file
        benchmark_info = read(benchmark_file)
        gridParameters = grid_parameters(benchmark_info)
        gridEnv = GridEnv(gridParameters)
        start_time = time.time()  # Record starting time
        while gridEnv.episode < 1:
            gridEnv.reset()
            if gridEnv.episode == 1:
                PlotDraw.draw_cost_plot(gridEnv.episode_cost, benchmark_i)
                PlotDraw.draw_origin_grid_plot(gridParameters, benchmark_i)
                PlotDraw.draw_grid_plot(gridEnv, benchmark_i)
            if gridEnv.episode == 1:
                break
            gridEnv.breakup()
            route, cost = a_star_route(gridEnv.init_pos, gridEnv.goal_pos,
                                       gridEnv.occupied_coord, gridParameters['gridSize'])
            gridEnv.route = route
            gridEnv.cost = cost
            gridEnv.update()
        benchmark_i += 1
        end_time = time.time()  # Record ending time
        print(benchmark_file)
        print("time{i} = {t} s".format(i=benchmark_i, t=end_time - start_time))
