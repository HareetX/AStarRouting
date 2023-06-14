import argparse
import copy
import heapq
import os
import random
import time

import numpy as np

import PlotDraw
from GridEnvironment import GridEnv
from utils import diagonal_distance
from ProblemParser import grid_parameters
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
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-north
        if x < grid_size[0] - 1 and y < grid_size[1] - 1:
            pos = [x + 1, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [1, 1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the north
        if y < grid_size[1] - 1:
            pos = [x, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [0, 1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-north
        if x > 0 and y < grid_size[1] - 1:
            pos = [x - 1, y + 1, z]
            g_cost = 0
            if direct is not None and direct != [-1, 1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the west
        if x > 0:
            pos = [x - 1, y, z]
            g_cost = 0
            if direct is not None and direct != [-1, 0, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the west-south
        if x > 0 and y > 0:
            pos = [x - 1, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [-1, -1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)
        # go to the south
        if y > 0:
            pos = [x, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [0, -1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
            item = Item(pos, self.end_pos, self.g_score, g_cost + 1, self)
            neighbors.append(item)
        # go to the east-south
        if x < grid_size[0] - 1 and y > 0:
            pos = [x + 1, y - 1, z]
            g_cost = 0
            if direct is not None and direct != [1, -1, 0]:
                g_cost = 0.1  # bend cost
            if pos != self.end_pos and str(pos) in occupied_msg:
                g_cost += occupied_msg[str(pos)]
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

    def get_neighbors_v1(self, grid_graph, grid_size):
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score, g_cost + 1.414, self)
            neighbors.append(item)

        # go to upside through a via
        if z < grid_size[2] - 1:
            next_x = x
            next_y = y
            next_z = z + 1
            g_cost = 0
            if [next_x, next_y, next_z] != self.end_pos:
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
            if [next_x, next_y, next_z] != self.end_pos:
                g_cost += grid_graph[next_x, next_y, next_z]
            item = Item([next_x, next_y, next_z], self.end_pos, self.g_score,
                        g_cost + 10, self)  # go through a via need a high cost
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


global get_neighbors_time
global add_neighbors_time

global get_neighbors_num


def a_star_route(start, end, occupied_msg, grid_size):
    # arg = solver_arguments()
    # if arg.trace:
    global get_neighbors_time, add_neighbors_time, get_neighbors_num
    open_set = []
    # closed_set = []
    closed_set = set([])

    start_item = Item(start, end, 0.0, 0.0, None)
    heapq.heappush(open_set, start_item)

    while open_set:
        cur_item = heapq.heappop(open_set)
        if cur_item.cur_pos == end:
            return generate_path(cur_item)
        else:
            # closed_set.append(cur_item)
            closed_set.add(str(cur_item.cur_pos))

            if arg.trace:
                get_neighbors_time_start = time.time()
                neighbor_list = cur_item.get_neighbors(occupied_msg, grid_size)
                get_neighbors_time_end = time.time()
                get_neighbors_time += get_neighbors_time_end - get_neighbors_time_start
                get_neighbors_num += 1
            else:
                neighbor_list = cur_item.get_neighbors(occupied_msg, grid_size)

            if arg.trace:
                add_neighbors_time_start = time.time()
                for neighbor in neighbor_list:
                    # if neighbor in closed_set:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        flag = True
                        for item in open_set:
                            if neighbor == item:
                                flag = False
                                if neighbor.g_score < item.g_score:
                                    item.set_g_score(neighbor.g_score)
                                    item.parent = neighbor.parent
                                    heapq.heapify(open_set)
                                break
                        if flag:
                            heapq.heappush(open_set, neighbor)
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
                        flag = True
                        for item in open_set:
                            if neighbor == item:
                                flag = False
                                if neighbor.g_score < item.g_score:
                                    item.set_g_score(neighbor.g_score)
                                    item.parent = neighbor.parent
                                    heapq.heapify(open_set)
                                break
                        if flag:
                            heapq.heappush(open_set, neighbor)
                    # elif neighbor in open_set:
                    #     for item in open_set:
                    #         if neighbor == item:
                    #             if neighbor.g_score < item.g_score:
                    #                 item.set_g_score(neighbor.g_score)
                    #                 item.parent = neighbor.parent
                    #                 heapq.heapify(open_set)
                    # else:
                    #     heapq.heappush(open_set, neighbor)


def a_star_route_v1(start, end, grid_graph, grid_size):
    # arg = solver_arguments()
    # if arg.trace:
    global get_neighbors_time, add_neighbors_time, get_neighbors_num
    open_set = []
    # closed_set = []
    closed_set = set([])

    start_item = Item(start, end, 0.0, 0.0, None)
    heapq.heappush(open_set, start_item)

    while open_set:
        cur_item = heapq.heappop(open_set)
        if cur_item.cur_pos == end:
            return generate_path(cur_item)
        else:
            # closed_set.append(cur_item)
            closed_set.add(str(cur_item.cur_pos))

            if arg.trace:
                get_neighbors_time_start = time.time()
                neighbor_list = cur_item.get_neighbors_v1(grid_graph, grid_size)
                get_neighbors_time_end = time.time()
                get_neighbors_time += get_neighbors_time_end - get_neighbors_time_start
                get_neighbors_num += 1
            else:
                neighbor_list = cur_item.get_neighbors_v1(grid_graph, grid_size)

            if arg.trace:
                add_neighbors_time_start = time.time()
                for neighbor in neighbor_list:
                    # if neighbor in closed_set:
                    if str(neighbor.cur_pos) in closed_set:
                        continue
                    else:
                        flag = True
                        for item in open_set:
                            if neighbor == item:
                                flag = False
                                if neighbor.g_score < item.g_score:
                                    item.set_g_score(neighbor.g_score)
                                    item.parent = neighbor.parent
                                    heapq.heapify(open_set)
                                break
                        if flag:
                            heapq.heappush(open_set, neighbor)
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
                        flag = True
                        for item in open_set:
                            if neighbor == item:
                                flag = False
                                if neighbor.g_score < item.g_score:
                                    item.set_g_score(neighbor.g_score)
                                    item.parent = neighbor.parent
                                    heapq.heapify(open_set)
                                break
                        if flag:
                            heapq.heappush(open_set, neighbor)
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
    parser.add_argument('--type', type=int, dest='type', default=1)
    parser.add_argument('--episode', type=int, dest='episode', default=5)
    parser.add_argument('--log', type=str, dest='log', default="log.txt")
    parser.add_argument('--trace', type=bool, dest='trace', default=False)

    return parser.parse_args()


if __name__ == '__main__':
    arg = solver_arguments()

    log = open(arg.log, 'w', encoding="utf-8")

    benchmark_dir = 'benchmark'
    benchmark_i = 0

    for benchmark_file in os.listdir(benchmark_dir):
        benchmark_file = benchmark_dir + '/' + benchmark_file
        gridParameters = grid_parameters(benchmark_file)
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
                PlotDraw.draw_cost_plot(gridEnv.episode_cost, benchmark_i)
                PlotDraw.draw_origin_grid_plot(gridParameters, benchmark_i)
                PlotDraw.draw_grid_plot(gridEnv, benchmark_i)
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

            if arg.trace:
                route_time_start = time.time()  # Record route starting time
                if arg.type == 0:
                    route, cost = a_star_route(gridEnv.init_pos, gridEnv.goal_pos,
                                               gridEnv.occupied_coord, gridParameters['gridSize'])
                else:
                    route, cost = a_star_route_v1(gridEnv.init_pos, gridEnv.goal_pos,
                                                  gridEnv.grid_graph, gridParameters['gridSize'])
                route_time_end = time.time()  # Record route ending time
                route_time += route_time_end - route_time_start
            else:
                if arg.type == 0:
                    route, cost = a_star_route(gridEnv.init_pos, gridEnv.goal_pos,
                                               gridEnv.occupied_coord, gridParameters['gridSize'])
                else:
                    route, cost = a_star_route_v1(gridEnv.init_pos, gridEnv.goal_pos,
                                                  gridEnv.grid_graph, gridParameters['gridSize'])

            # gridEnv.route = route
            gridEnv.set_route(route)
            gridEnv.cost = cost

            if arg.trace:
                modify_time_start = time.time()  # Record modify starting time
                if arg.type == 0:
                    gridEnv.update()
                else:
                    gridEnv.update_v1()
                modify_time_end = time.time()  # Record modify ending time
                modify_time += modify_time_end - modify_time_start
            else:
                if arg.type == 0:
                    gridEnv.update()
                else:
                    gridEnv.update_v1()

        benchmark_i += 1
        end_time = time.time()  # Record ending time

        print("benchmark{i} time = {t} s".format(i=benchmark_i, t=end_time - start_time))
        print("benchmark{i} time = {t} s".format(i=benchmark_i, t=end_time - start_time), file=log)
        if arg.trace:
            print("benchmark{i}".format(i=benchmark_i))
            print("benchmark{i} time = {t} s".format(i=benchmark_i, t=end_time - start_time))
            print("benchmark{i} route time = {t} s".format(i=benchmark_i, t=route_time))
            print("\tget neighbors time = {t} s".format(t=get_neighbors_time))
            print("\tget neighbors number = {num}".format(num=get_neighbors_num))
            print("\tadd neighbors time = {t} s".format(t=add_neighbors_time))
            print("benchmark{i} modify time = {t} s".format(i=benchmark_i, t=modify_time))
            print("benchmark{i} route length = {len}".format(i=benchmark_i, len=route_length(gridEnv.route_combo)))
            print("benchmark{i} via amount = {num}\n".format(i=benchmark_i, num=via_amount(gridEnv.route_combo)))
            # print("benchmark{i} via amount = {num} / {min}\n".format(i=benchmark_i,
            #                                                          num=via_amount(gridEnv.route_combo),
            #                                                          min=min_via_amount(gridEnv.netlist)))

            print("benchmark{i}".format(i=benchmark_i), file=log)
            print("benchmark{i} time = {t} s".format(i=benchmark_i, t=end_time - start_time), file=log)
            print("benchmark{i} route time = {t} s".format(i=benchmark_i, t=route_time), file=log)
            print("\tget neighbors time = {t} s".format(t=get_neighbors_time), file=log)
            print("\tget neighbors number = {num}".format(num=get_neighbors_num), file=log)
            print("\tadd neighbors time = {t} s".format(t=add_neighbors_time), file=log)
            print("benchmark{i} modify time = {t} s\n".format(i=benchmark_i, t=modify_time), file=log)
            print("benchmark{i} route length = {len}".format(i=benchmark_i, len=route_length(gridEnv.route_combo)),
                  file=log)
            print("benchmark{i} via amount = {num}\n".format(i=benchmark_i, num=via_amount(gridEnv.route_combo)), file=log)
            # print("benchmark{i} via amount = {num} / {min}\n".format(i=benchmark_i,
            #                                                          num=via_amount(gridEnv.route_combo),
            #                                                          min=min_via_amount(gridEnv.netlist)), file=log)

    log.close()
