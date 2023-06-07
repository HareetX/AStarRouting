import copy
import os
import random

import numpy as np

from MathFunc import diagonal_distance
from ProblemParser import read, grid_parameters


class GridEnv:
    def __init__(self, grid_parameter):
        self.grid_parameter = grid_parameter

        self.twoPinNetCombo, self.twoPinNetNums = self.generate_two_pin_net()
        self.occupied_coord = self.generate_occupied_coord()
        self.route_combo = []
        self.route_cost = []
        self.episode_cost = []

        self.init_pos = None
        self.goal_pos = None

        self.twoPinNet_i = 0
        self.route = []
        self.old_route = []
        self.cost = 1000  # Large Enough

        self.episode = 0

    def generate_two_pin_net(self):
        single_net_pins = []
        netlist = []
        net_order = []
        for i in range(self.grid_parameter['numNet']):
            for j in range(self.grid_parameter['netInfo'][i]['numPins']):
                single_net_pins.append(self.grid_parameter['netInfo'][i][str(j + 1)])
            netlist.append(single_net_pins)
            single_net_pins = []
            net_order.append(i)

        # sort the netlist with halfWireLength
        # TODO

        # use MST to optimize two-pin nets
        two_pin_net_nums = []
        two_pin_set = []
        two_pin_nets = []
        for i in range(self.grid_parameter['numNet']):
            netlist_len = len(netlist[i])
            two_pin_net_nums.append(netlist_len - 1)
            for j in range(netlist_len - 1):
                for p in range(netlist_len - j - 1):
                    pin_start = netlist[i][j]
                    pin_end = netlist[i][p + j + 1]
                    distance = diagonal_distance(pin_start, pin_end)
                    two_pin_set.append(([pin_start, pin_end], distance))
            two_pin_set.sort(key=lambda x: x[1])
            fetched_pins = []
            for j in range(len(two_pin_set)):
                not_loop = False
                if two_pin_set[j][0][0] not in fetched_pins:
                    not_loop = True
                    fetched_pins.append(two_pin_set[j][0][0])
                if two_pin_set[j][0][1] not in fetched_pins:
                    not_loop = True
                    fetched_pins.append(two_pin_set[j][0][1])
                if not_loop:
                    two_pin_nets.append(two_pin_set[j][0])
                if len(fetched_pins) >= netlist_len:
                    break

            two_pin_set = []

        return two_pin_nets, two_pin_net_nums

    def generate_occupied_coord(self):
        occupied_dict = {}
        for i in range(self.grid_parameter['numNet']):
            for j in range(self.grid_parameter['netInfo'][i]['numPins']):
                occupied_dict[str(self.grid_parameter['netInfo'][i][str(j + 1)])] = 1000  # Large Enough
        return occupied_dict

    def add_occupied_coord(self):
        for i in range(len(self.route)):
            if i > 0:
                hide_pos_1 = None
                hide_pos_2 = None
                if self.route[i][0] - self.route[i-1][0] == -1 and self.route[i][1] - self.route[i-1][1] == 1:
                    hide_pos_1 = [self.route[i][0] + 1, self.route[i][1], self.route[i][2]]
                    hide_pos_2 = [self.route[i][0], self.route[i][1] - 1, self.route[i][2]]
                elif self.route[i][0] - self.route[i-1][0] == -1 and self.route[i][1] - self.route[i-1][1] == -1:
                    hide_pos_1 = [self.route[i][0] + 1, self.route[i][1], self.route[i][2]]
                    hide_pos_2 = [self.route[i][0], self.route[i][1] + 1, self.route[i][2]]
                elif self.route[i][0] - self.route[i-1][0] == 1 and self.route[i][1] - self.route[i-1][1] == 1:
                    hide_pos_1 = [self.route[i][0] - 1, self.route[i][1], self.route[i][2]]
                    hide_pos_2 = [self.route[i][0], self.route[i][1] - 1, self.route[i][2]]
                elif self.route[i][0] - self.route[i-1][0] == 1 and self.route[i][1] - self.route[i-1][1] == -1:
                    hide_pos_1 = [self.route[i][0] - 1, self.route[i][1], self.route[i][2]]
                    hide_pos_2 = [self.route[i][0], self.route[i][1] + 1, self.route[i][2]]
                elif abs(self.route[i][0] - self.route[i-1][0]) == 1 and self.route[i][1] - self.route[i-1][1] == 0:
                    hide_pos_1 = [self.route[i][0], self.route[i][1] + 1, self.route[i][2]]
                    hide_pos_2 = [self.route[i][0], self.route[i][1] + 1, self.route[i][2]]
                elif self.route[i][0] - self.route[i-1][0] == 0 and abs(self.route[i][1] - self.route[i-1][1]) == 1:
                    hide_pos_1 = [self.route[i][0] + 1, self.route[i][1], self.route[i][2]]
                    hide_pos_2 = [self.route[i][0] + 1, self.route[i][1], self.route[i][2]]
                if hide_pos_1 is not None and hide_pos_2 is not None:
                    if str(hide_pos_1) in self.occupied_coord:
                        self.occupied_coord[str(hide_pos_1)] += 1000
                    else:
                        self.occupied_coord[str(hide_pos_1)] = 1000  # Large Enough
                    if str(hide_pos_2) in self.occupied_coord:
                        self.occupied_coord[str(hide_pos_2)] += 1000
                    else:
                        self.occupied_coord[str(hide_pos_2)] = 1000  # Large Enough
            if str(self.route[i]) in self.occupied_coord:
                self.occupied_coord[str(self.route[i])] += 1000
            else:
                self.occupied_coord[str(self.route[i])] = 1000  # Large Enough

    def del_occupied_coord(self):
        for i in range(len(self.old_route)):
            if i > 0:
                hide_pos_1 = None
                hide_pos_2 = None
                if self.old_route[i][0] - self.old_route[i - 1][0] == -1 and self.old_route[i][1] - self.old_route[i - 1][1] == 1:
                    hide_pos_1 = [self.old_route[i][0] + 1, self.old_route[i][1], self.old_route[i][2]]
                    hide_pos_2 = [self.old_route[i][0], self.old_route[i][1] - 1, self.old_route[i][2]]
                elif self.old_route[i][0] - self.old_route[i - 1][0] == -1 and self.old_route[i][1] - self.old_route[i - 1][1] == -1:
                    hide_pos_1 = [self.old_route[i][0] + 1, self.old_route[i][1], self.old_route[i][2]]
                    hide_pos_2 = [self.old_route[i][0], self.old_route[i][1] + 1, self.old_route[i][2]]
                elif self.old_route[i][0] - self.old_route[i - 1][0] == 1 and self.old_route[i][1] - self.old_route[i - 1][1] == 1:
                    hide_pos_1 = [self.old_route[i][0] - 1, self.old_route[i][1], self.old_route[i][2]]
                    hide_pos_2 = [self.old_route[i][0], self.old_route[i][1] - 1, self.old_route[i][2]]
                elif self.old_route[i][0] - self.old_route[i - 1][0] == 1 and self.old_route[i][1] - self.old_route[i - 1][1] == -1:
                    hide_pos_1 = [self.old_route[i][0] - 1, self.old_route[i][1], self.old_route[i][2]]
                    hide_pos_2 = [self.old_route[i][0], self.old_route[i][1] + 1, self.old_route[i][2]]
                elif abs(self.old_route[i][0] - self.old_route[i - 1][0]) == 1 and self.old_route[i][1] - self.old_route[i - 1][1] == 0:
                    hide_pos_1 = [self.old_route[i][0], self.old_route[i][1] + 1, self.old_route[i][2]]
                    hide_pos_2 = [self.old_route[i][0], self.old_route[i][1] + 1, self.old_route[i][2]]
                elif self.old_route[i][0] - self.old_route[i - 1][0] == 0 and abs(self.old_route[i][1] - self.old_route[i - 1][1]) == 1:
                    hide_pos_1 = [self.old_route[i][0] + 1, self.old_route[i][1], self.old_route[i][2]]
                    hide_pos_2 = [self.old_route[i][0] + 1, self.old_route[i][1], self.old_route[i][2]]
                if hide_pos_1 is not None and hide_pos_2 is not None:
                    self.occupied_coord[str(hide_pos_1)] -= 1000
                    if self.occupied_coord[str(hide_pos_1)] == 0:
                        del self.occupied_coord[str(hide_pos_1)]
                    self.occupied_coord[str(hide_pos_2)] -= 1000
                    if self.occupied_coord[str(hide_pos_2)] == 0:
                        del self.occupied_coord[str(hide_pos_2)]
            self.occupied_coord[str(self.old_route[i])] -= 1000
            if self.occupied_coord[str(self.old_route[i])] == 0:
                del self.occupied_coord[str(self.old_route[i])]

    def update(self):
        if self.episode > 0:
            if self.cost > self.route_cost[self.twoPinNet_i - 1]:
                self.route = self.old_route
                self.cost = self.route_cost[self.twoPinNet_i - 1]
            self.add_occupied_coord()
            self.route_combo[self.twoPinNet_i - 1] = self.route
            self.route_cost[self.twoPinNet_i - 1] = self.cost
        else:
            self.add_occupied_coord()
            self.route_combo.append(self.route)
            self.route_cost.append(self.cost)

    def breakup(self):
        # breakup the routing
        if self.episode > 0:
            self.old_route = self.route_combo[self.twoPinNet_i - 1]
            self.route_combo[self.twoPinNet_i - 1] = []
            self.del_occupied_coord()

    def reset(self):
        # if self.twoPinNet_i > 0:
        #     if self.episode > 0:
        #         if self.cost > self.route_cost[self.twoPinNet_i - 1]:
        #             self.route = self.old_route
        #             self.cost = self.route_cost[self.twoPinNet_i - 1]
        #         self.add_occupied_coord()
        #         self.route_combo[self.twoPinNet_i - 1] = self.route
        #         self.route_cost[self.twoPinNet_i - 1] = self.cost
        #     else:
        #         self.add_occupied_coord()
        #         self.route_combo.append(self.route)
        #         self.route_cost.append(self.cost)
        self.route = []
        self.cost = 1000  # Large Enough

        if self.twoPinNet_i >= len(self.twoPinNetCombo):
            # all the two pin nets are routed
            self.episode_cost.append(sum(self.route_cost))
            self.episode += 1
            self.twoPinNet_i = 0

        # initialize state of gridEnv by the two pin net(self.twoPinNet_i)
        self.init_pos = self.twoPinNetCombo[self.twoPinNet_i][0]
        self.goal_pos = self.twoPinNetCombo[self.twoPinNet_i][1]

        # breakup the routing
        # if self.episode > 0:
        #     self.old_route = self.route_combo[self.twoPinNet_i]
        #     self.route_combo[self.twoPinNet_i] = []
        #     self.del_occupied_coord()

        self.twoPinNet_i += 1


def a_star_solver_sim(start_pos, end_pos, i):
    result1_table = [([[1, 2, 0], [2, 3, 0], [2, 4, 0]], 2),
                     ([[2, 2, 0], [3, 3, 0], [2, 3, 0], [3, 4, 0]], 3),
                     ([[3, 2, 0], [4, 3, 0], [4, 4, 0]], 2),
                     ([[4, 2, 0], [5, 3, 0], [5, 4, 0]], 2),
                     ([[5, 2, 0], [6, 3, 0], [6, 4, 0]], 2)]
    result2_table = [([[1, 2, 0], [1, 3, 0], [2, 4, 0]], 2),
                     ([[2, 2, 0], [3, 3, 0], [3, 4, 0]], 2),
                     ([[3, 2, 0], [4, 3, 0], [4, 4, 0]], 2),
                     ([[4, 2, 0], [5, 3, 0], [5, 4, 0]], 2),
                     ([[5, 2, 0], [6, 3, 0], [6, 4, 0]], 2)]
    if i < 5:
        result = result1_table[i]
    else:
        result = result2_table[i-5]
    return result


if __name__ == '__main__':
    benchmark_dir = 'benchmark'
    for benchmark_file in os.listdir(benchmark_dir):
        benchmark_file = benchmark_dir + '/' + benchmark_file
        benchmark_info = read(benchmark_file)
        gridParameters = grid_parameters(benchmark_info)
        gridEnv = GridEnv(gridParameters)
        k = 0
        while gridEnv.episode < 10:
            gridEnv.reset()
            if gridEnv.episode == 10:
                break
            gridEnv.breakup()
            route, cost = a_star_solver_sim(gridEnv.init_pos, gridEnv.goal_pos, k)
            gridEnv.route = route
            gridEnv.cost = cost
            gridEnv.update()
            k += 1
        print(gridEnv)
