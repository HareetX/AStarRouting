import copy
import os
import random

import numpy as np

from utils import diagonal_distance
from ProblemParser import grid_parameters


class GridEnv:
    def __init__(self, grid_parameter, type_=0):
        self.grid_parameter = grid_parameter
        self.grid_size = grid_parameter['gridSize']
        self.num_net = grid_parameter['numNet']

        self.d_line = 1
        self.d = 2 * self.d_line + 1

        self.pin_hsize = 1
        self.pin_size = 2 * self.pin_hsize + 1

        self.twoPinNetCombo, self.twoPinNetNums, self.netlist = self.generate_two_pin_net()

        if type_ == 0:
            self.occupied_coord = self.generate_occupied_coord()
        else:
            self.grid_graph = self.generate_grid_graph()

        self.route_combo = []
        self.route_cost = []
        self.episode_cost = []

        self.init_pos = None
        self.goal_pos = None

        self.twoPinNet_i = 0
        self.multiPinNet_i = 0
        self.netPinSet = set([])
        self.netPinRoute = []
        self.old_netPinRoute = []
        self.route = []
        self.old_route = []
        self.cost = 0

        self.episode = 0

    def generate_two_pin_net(self):
        single_net_pins = []
        netlist = []
        net_order = []
        for i in range(self.num_net):
            net_info = self.grid_parameter['netInfo'][i]
            for j in range(net_info['numPins']):
                single_net_pins.append(net_info[str(j + 1)])
            netlist.append(single_net_pins)
            single_net_pins = []
            net_order.append(i)

        # sort the netlist with halfWireLength
        # TODO

        # use MST to optimize two-pin nets
        two_pin_net_nums = []
        two_pin_set = []
        two_pin_nets = []
        two_pin_nets_combo = []
        for i in range(self.num_net):
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
                if len(fetched_pins) >= netlist_len:
                    not_loop = True
                if not_loop:
                    two_pin_nets.append(two_pin_set[j][0])
                if len(two_pin_nets) >= netlist_len - 1:
                    break
            two_pin_nets_combo.append(two_pin_nets)
            two_pin_nets = []
            two_pin_set = []

        return two_pin_nets_combo, two_pin_net_nums, netlist

    def generate_occupied_coord(self):
        occupied_dict = {}
        for i in range(self.num_net):
            net_info = self.grid_parameter['netInfo'][i]
            for j in range(net_info['numPins']):
                x_coord = net_info[str(j + 1)][0]
                y_coord = net_info[str(j + 1)][1]
                z_coord = net_info[str(j + 1)][2]

                x_min = x_coord - self.pin_hsize
                x_max = x_coord + self.pin_hsize + 1
                y_min = y_coord - self.pin_hsize
                y_max = y_coord + self.pin_hsize + 1
                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                # Set the cost of the pin
                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        hide_pos = [x_min + x_i, y_min + y_i, z_coord]
                        if str(hide_pos) in occupied_dict:
                            occupied_dict[str(hide_pos)] += 1000
                        else:
                            occupied_dict[str(hide_pos)] = 1000  # Large Enough

                # occupied_dict[str(net_info[str(j + 1)])] = 1000  # Large Enough
        return occupied_dict

    def add_pin_effect(self, pin_x, pin_y, pin_z):
        x_min = pin_x - self.pin_hsize
        x_max = pin_x + self.pin_hsize + 1
        y_min = pin_y - self.pin_hsize
        y_max = pin_y + self.pin_hsize + 1
        if x_min < 0:
            x_min = 0
        if x_max > self.grid_size[0]:
            x_max = self.grid_size[0]
        if y_min < 0:
            y_min = 0
        if y_max > self.grid_size[1]:
            y_max = self.grid_size[1]
        # Add the cost of the specified pin
        for x_i in range(x_max - x_min):
            for y_i in range(y_max - y_min):
                hide_pos = [x_min + x_i, y_min + y_i, pin_z]
                if str(hide_pos) in self.occupied_coord:
                    self.occupied_coord[str(hide_pos)] += 1000
                else:
                    self.occupied_coord[str(hide_pos)] = 1000  # Large Enough

    def add_pin_effect_v1(self, pin_x, pin_y, pin_z):
        x_min = pin_x - self.pin_hsize
        x_max = pin_x + self.pin_hsize + 1
        y_min = pin_y - self.pin_hsize
        y_max = pin_y + self.pin_hsize + 1
        if x_min < 0:
            x_min = 0
        if x_max > self.grid_size[0]:
            x_max = self.grid_size[0]
        if y_min < 0:
            y_min = 0
        if y_max > self.grid_size[1]:
            y_max = self.grid_size[1]
        # Add the cost of the specified pin
        self.grid_graph[x_min:x_max, y_min:y_max, pin_z] += 1000

    def eliminate_pin_effect(self, pin_x, pin_y, pin_z):
        x_min = pin_x - self.pin_hsize
        x_max = pin_x + self.pin_hsize + 1
        y_min = pin_y - self.pin_hsize
        y_max = pin_y + self.pin_hsize + 1
        if x_min < 0:
            x_min = 0
        if x_max > self.grid_size[0]:
            x_max = self.grid_size[0]
        if y_min < 0:
            y_min = 0
        if y_max > self.grid_size[1]:
            y_max = self.grid_size[1]
        # Del the cost of the specified pin
        for x_i in range(x_max - x_min):
            for y_i in range(y_max - y_min):
                hide_pos = [x_min + x_i, y_min + y_i, pin_z]
                self.occupied_coord[str(hide_pos)] -= 1000
                if self.occupied_coord[str(hide_pos)] <= 0:
                    del self.occupied_coord[str(hide_pos)]

    def eliminate_pin_effect_v1(self, pin_x, pin_y, pin_z):
        x_min = pin_x - self.pin_hsize
        x_max = pin_x + self.pin_hsize + 1
        y_min = pin_y - self.pin_hsize
        y_max = pin_y + self.pin_hsize + 1
        if x_min < 0:
            x_min = 0
        if x_max > self.grid_size[0]:
            x_max = self.grid_size[0]
        if y_min < 0:
            y_min = 0
        if y_max > self.grid_size[1]:
            y_max = self.grid_size[1]
        # Del the cost of the specified pin
        self.grid_graph[x_min:x_max, y_min:y_max, pin_z] -= 1000

    def generate_grid_graph(self):
        grid_graph = np.zeros(self.grid_size)
        for i in range(self.num_net):
            net_info = self.grid_parameter['netInfo'][i]
            for j in range(net_info['numPins']):
                x_coord = net_info[str(j + 1)][0]
                y_coord = net_info[str(j + 1)][1]
                z_coord = net_info[str(j + 1)][2]

                x_min = x_coord - self.pin_hsize
                x_max = x_coord + self.pin_hsize + 1
                y_min = y_coord - self.pin_hsize
                y_max = y_coord + self.pin_hsize + 1
                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                # Set the cost of the pin
                grid_graph[x_min:x_max, y_min:y_max, z_coord] += 1000
        return grid_graph

    def set_route(self, origin_route):
        self.route = []
        for i in range(len(origin_route)):
            if i < 1:
                self.route.append(origin_route[i])
            elif i == len(origin_route) - 1:
                self.route.append(origin_route[i])
            else:
                x_0, y_0, z_0 = origin_route[i - 1]
                x_1, y_1, z_1 = origin_route[i]
                x_2, y_2, z_2 = origin_route[i + 1]
                direct_0 = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
                direct_1 = [x_2 - x_1, y_2 - y_1, z_2 - z_1]
                if direct_0 != direct_1:
                    self.route.append(origin_route[i])

    def add_occupied_coord(self, route):
        for i in range(len(route) - 1):
            x_0, y_0, z_0 = route[i]
            x_1, y_1, z_1 = route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                x_max = x_0 + self.d_line + 1
                x_min = x_0 - self.d_line
                if direct[1] == 0:  # go through a via
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + self.d_line + 1
                        y_min = y_0 - self.d_line
                    else:  # go to the south
                        y_max = y_0 + self.d_line + 1
                        y_min = y_1 - self.d_line
                    z_max = z_0 + 1
                    z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            if str(hide_pos) in self.occupied_coord:
                                self.occupied_coord[str(hide_pos)] += 1000
                            else:
                                self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
            elif direct[1] == 0:  # go to the east or west
                if x_0 < x_1:  # go to the east
                    x_max = x_1 + self.d_line + 1
                    x_min = x_0 - self.d_line
                else:  # go to the west
                    x_max = x_0 + self.d_line + 1
                    x_min = x_1 - self.d_line
                y_max = y_0 + self.d_line + 1
                y_min = y_0 - self.d_line
                z_max = z_0 + 1
                z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            if str(hide_pos) in self.occupied_coord:
                                self.occupied_coord[str(hide_pos)] += 1000
                            else:
                                self.occupied_coord[str(hide_pos)] = 1000  # Large Enough
            else:  # go diagonally
                if direct[0] > 0:
                    delta_x = 1
                    if direct[1] > 0:  # go to the north-east
                        delta_y = 1
                    else:  # go to the south-east
                        delta_y = -1
                else:
                    delta_x = -1
                    if direct[1] > 0:  # go to the north-west
                        delta_y = 1
                    else:  # go to the south-west
                        delta_y = -1
                while x_0 != x_1 + delta_x:
                    x_max = x_0 + self.d_line + 1
                    x_min = x_0 - self.d_line
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line

                    if x_min < 0:
                        x_min = 0
                    if x_max > self.grid_size[0]:
                        x_max = self.grid_size[0]
                    if y_min < 0:
                        y_min = 0
                    if y_max > self.grid_size[1]:
                        y_max = self.grid_size[1]
                    # Set the cost of the routed path Large Enough
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_0]
                            if str(hide_pos) in self.occupied_coord:
                                self.occupied_coord[str(hide_pos)] += 1000
                            else:
                                self.occupied_coord[str(hide_pos)] = 1000  # Large Enough

                    x_0 += delta_x
                    y_0 += delta_y

    def add_route_occupied(self, route):
        for i in range(len(route) - 1):
            x_0, y_0, z_0 = route[i]
            x_1, y_1, z_1 = route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                x_max = x_0 + self.d_line + 1
                x_min = x_0 - self.d_line
                if direct[1] == 0:  # go through a via
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + self.d_line + 1
                        y_min = y_0 - self.d_line
                    else:  # go to the south
                        y_max = y_0 + self.d_line + 1
                        y_min = y_1 - self.d_line
                    z_max = z_0 + 1
                    z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]
                # Set the cost of the routed path Large Enough
                self.grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] += 1000
            elif direct[1] == 0:  # go to the east or west
                if x_0 < x_1:  # go to the east
                    x_max = x_1 + self.d_line + 1
                    x_min = x_0 - self.d_line
                else:  # go to the west
                    x_max = x_0 + self.d_line + 1
                    x_min = x_1 - self.d_line
                y_max = y_0 + self.d_line + 1
                y_min = y_0 - self.d_line
                z_max = z_0 + 1
                z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]
                # Set the cost of the routed path Large Enough
                self.grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] += 1000
            else:  # go diagonally
                if direct[0] > 0:
                    delta_x = 1
                    if direct[1] > 0:  # go to the north-east
                        delta_y = 1
                    else:  # go to the south-east
                        delta_y = -1
                else:
                    delta_x = -1
                    if direct[1] > 0:  # go to the north-west
                        delta_y = 1
                    else:  # go to the south-west
                        delta_y = -1
                while x_0 != x_1 + delta_x:
                    x_max = x_0 + self.d_line + 1
                    x_min = x_0 - self.d_line
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line

                    if x_min < 0:
                        x_min = 0
                    if x_max > self.grid_size[0]:
                        x_max = self.grid_size[0]
                    if y_min < 0:
                        y_min = 0
                    if y_max > self.grid_size[1]:
                        y_max = self.grid_size[1]
                    # Set the cost of the routed path Large Enough
                    self.grid_graph[x_min:x_max, y_min:y_max, z_0] += 1000

                    x_0 += delta_x
                    y_0 += delta_y

    def del_occupied_coord(self, old_route):
        for i in range(len(old_route) - 1):
            x_0, y_0, z_0 = old_route[i]
            x_1, y_1, z_1 = old_route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                x_max = x_0 + self.d_line + 1
                x_min = x_0 - self.d_line
                if direct[1] == 0:  # go through a via
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + self.d_line + 1
                        y_min = y_0 - self.d_line
                    else:  # go to the south
                        y_max = y_0 + self.d_line + 1
                        y_min = y_1 - self.d_line
                    z_max = z_0 + 1
                    z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            self.occupied_coord[str(hide_pos)] -= 1000
                            if self.occupied_coord[str(hide_pos)] <= 0:
                                del self.occupied_coord[str(hide_pos)]
            elif direct[1] == 0:  # go to the east or west
                if x_0 < x_1:  # go to the east
                    x_max = x_1 + self.d_line + 1
                    x_min = x_0 - self.d_line
                else:  # go to the west
                    x_max = x_0 + self.d_line + 1
                    x_min = x_1 - self.d_line
                y_max = y_0 + self.d_line + 1
                y_min = y_0 - self.d_line

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]

                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        hide_pos = [x_min + x_i, y_min + y_i, z_0]
                        self.occupied_coord[str(hide_pos)] -= 1000
                        if self.occupied_coord[str(hide_pos)] <= 0:
                            del self.occupied_coord[str(hide_pos)]
            else:  # go diagonally
                if direct[0] > 0:
                    delta_x = 1
                    if direct[1] > 0:  # go to the north-east
                        delta_y = 1
                    else:  # go to the south-east
                        delta_y = -1
                else:
                    delta_x = -1
                    if direct[1] > 0:  # go to the north-west
                        delta_y = 1
                    else:  # go to the south-west
                        delta_y = -1
                while x_0 != x_1 + delta_x:
                    x_max = x_0 + self.d_line + 1
                    x_min = x_0 - self.d_line
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line

                    if x_min < 0:
                        x_min = 0
                    if x_max > self.grid_size[0]:
                        x_max = self.grid_size[0]
                    if y_min < 0:
                        y_min = 0
                    if y_max > self.grid_size[1]:
                        y_max = self.grid_size[1]
                    # Set the cost of the routed path Large Enough
                    for x_i in range(x_max - x_min):
                        for y_i in range(y_max - y_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_0]
                            self.occupied_coord[str(hide_pos)] -= 1000
                            if self.occupied_coord[str(hide_pos)] <= 0:
                                del self.occupied_coord[str(hide_pos)]

                    x_0 += delta_x
                    y_0 += delta_y

    def del_route_occupied(self, old_route):
        for i in range(len(old_route) - 1):
            x_0, y_0, z_0 = old_route[i]
            x_1, y_1, z_1 = old_route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                x_max = x_0 + self.d_line + 1
                x_min = x_0 - self.d_line
                if direct[1] == 0:  # go through a via
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + self.d_line + 1
                        y_min = y_0 - self.d_line
                    else:  # go to the south
                        y_max = y_0 + self.d_line + 1
                        y_min = y_1 - self.d_line
                    z_max = z_0 + 1
                    z_min = z_0

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                if z_min < 0:
                    z_min = 0
                if z_max > self.grid_size[2]:
                    z_max = self.grid_size[2]
                # Set the cost of the routed path Large Enough
                self.grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] -= 1000
            elif direct[1] == 0:  # go to the east or west
                if x_0 < x_1:  # go to the east
                    x_max = x_1 + self.d_line + 1
                    x_min = x_0 - self.d_line
                else:  # go to the west
                    x_max = x_0 + self.d_line + 1
                    x_min = x_1 - self.d_line
                y_max = y_0 + self.d_line + 1
                y_min = y_0 - self.d_line

                if x_min < 0:
                    x_min = 0
                if x_max > self.grid_size[0]:
                    x_max = self.grid_size[0]
                if y_min < 0:
                    y_min = 0
                if y_max > self.grid_size[1]:
                    y_max = self.grid_size[1]
                # Set the cost of the routed path Large Enough
                self.grid_graph[x_min:x_max, y_min:y_max, z_0] -= 1000
            else:  # go diagonally
                if direct[0] > 0:
                    delta_x = 1
                    if direct[1] > 0:  # go to the north-east
                        delta_y = 1
                    else:  # go to the south-east
                        delta_y = -1
                else:
                    delta_x = -1
                    if direct[1] > 0:  # go to the north-west
                        delta_y = 1
                    else:  # go to the south-west
                        delta_y = -1
                while x_0 != x_1 + delta_x:
                    x_max = x_0 + self.d_line + 1
                    x_min = x_0 - self.d_line
                    y_max = y_0 + self.d_line + 1
                    y_min = y_0 - self.d_line

                    if x_min < 0:
                        x_min = 0
                    if x_max > self.grid_size[0]:
                        x_max = self.grid_size[0]
                    if y_min < 0:
                        y_min = 0
                    if y_max > self.grid_size[1]:
                        y_max = self.grid_size[1]
                    # Set the cost of the routed path Large Enough
                    self.grid_graph[x_min:x_max, y_min:y_max, z_0] -= 1000

                    x_0 += delta_x
                    y_0 += delta_y

    def update(self, single_route, single_cost):
        self.set_route(single_route)
        self.cost += single_cost

        for pos in single_route:
            self.netPinSet.add(str(pos))

        self.netPinRoute.append(self.route)

        if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i] - 1:
            if self.episode > 0:
                if self.cost > self.route_cost[self.multiPinNet_i]:
                    self.netPinRoute = self.old_netPinRoute
                    self.cost = self.route_cost[self.multiPinNet_i]
                self.route_combo[self.multiPinNet_i] = self.netPinRoute
                self.route_cost[self.multiPinNet_i] = self.cost
            else:
                self.route_combo.append(self.netPinRoute)
                self.route_cost.append(self.cost)

            for pos in self.netlist[self.multiPinNet_i]:
                x_pos = pos[0]
                y_pos = pos[1]
                z_pos = pos[2]
                self.add_pin_effect(x_pos, y_pos, z_pos)

            for route in self.netPinRoute:
                self.add_occupied_coord(route)

        self.twoPinNet_i += 1

        # if self.episode > 0:
        #     if self.cost > self.route_cost[self.twoPinNet_i - 1]:
        #         self.route = self.old_route
        #         self.cost = self.route_cost[self.twoPinNet_i - 1]
        #     self.route_combo[self.twoPinNet_i - 1] = self.route
        #     self.route_cost[self.twoPinNet_i - 1] = self.cost
        # else:
        #     self.route_combo.append(self.route)
        #     self.route_cost.append(self.cost)
        #
        # self.add_occupied_coord()

    def update_v1(self, single_route, single_cost):
        self.set_route(single_route)
        self.cost += single_cost

        for pos in single_route:
            self.netPinSet.add(str(pos))

        self.netPinRoute.append(self.route)

        if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i] - 1:
            if self.episode > 0:
                if self.cost > self.route_cost[self.multiPinNet_i]:
                    self.netPinRoute = self.old_netPinRoute
                    self.cost = self.route_cost[self.multiPinNet_i]
                self.route_combo[self.multiPinNet_i] = self.netPinRoute
                self.route_cost[self.multiPinNet_i] = self.cost
            else:
                self.route_combo.append(self.netPinRoute)
                self.route_cost.append(self.cost)

            for pos in self.netlist[self.multiPinNet_i]:
                x_pos = pos[0]
                y_pos = pos[1]
                z_pos = pos[2]
                self.add_pin_effect_v1(x_pos, y_pos, z_pos)

            for route in self.netPinRoute:
                self.add_route_occupied(route)

        self.twoPinNet_i += 1

        # if self.episode > 0:
        #     if self.cost > self.route_cost[self.twoPinNet_i - 1]:
        #         self.route = self.old_route
        #         self.cost = self.route_cost[self.twoPinNet_i - 1]
        #     self.route_combo[self.twoPinNet_i - 1] = self.route
        #     self.route_cost[self.twoPinNet_i - 1] = self.cost
        # else:
        #     self.route_combo.append(self.route)
        #     self.route_cost.append(self.cost)
        #
        # self.add_route_occupied()

        # x_init = self.init_pos[0]
        # y_init = self.init_pos[1]
        # z_init = self.init_pos[2]
        # self.add_pin_effect(x_init, y_init, z_init)

        # x_goal = self.goal_pos[0]
        # y_goal = self.goal_pos[1]
        # z_goal = self.goal_pos[2]
        # self.add_pin_effect(x_goal, y_goal, z_goal)

    def breakup(self):
        # breakup the routing
        if self.episode > 0 and self.twoPinNet_i == 0:
            self.old_netPinRoute = self.route_combo[self.multiPinNet_i]
            self.route_combo[self.multiPinNet_i] = []
            for old_route in self.old_netPinRoute:
                self.del_occupied_coord(old_route)

    def breakup_v1(self):
        # breakup the routing
        if self.episode > 0 and self.twoPinNet_i == 0:
            self.old_netPinRoute = self.route_combo[self.multiPinNet_i]
            self.route_combo[self.multiPinNet_i] = []
            for old_route in self.old_netPinRoute:
                self.del_route_occupied(old_route)

    def reset(self):
        self.route = []

        # initialize state of gridEnv by the two pin net(self.twoPinNet_i)
        if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i]:
            self.cost = 0
            self.netPinSet = set([])
            self.netPinRoute = []
            self.twoPinNet_i = 0
            self.multiPinNet_i += 1

        if self.multiPinNet_i >= len(self.twoPinNetCombo):
            # all the two pin nets are routed
            self.episode_cost.append(sum(self.route_cost))
            self.episode += 1
            self.multiPinNet_i = 0

        self.init_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][0]
        self.goal_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][1]

        if self.twoPinNet_i == 0:
            for pos in self.netlist[self.multiPinNet_i]:
                x_pos = pos[0]
                y_pos = pos[1]
                z_pos = pos[2]
                self.eliminate_pin_effect(x_pos, y_pos, z_pos)

    def reset_v1(self):
        self.route = []

        # initialize state of gridEnv by the two pin net(self.twoPinNet_i)
        if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i]:
            self.cost = 0
            self.netPinSet = set([])
            self.netPinRoute = []
            self.twoPinNet_i = 0
            self.multiPinNet_i += 1

        if self.multiPinNet_i >= len(self.twoPinNetCombo):
            # all the two pin nets are routed
            self.episode_cost.append(sum(self.route_cost))
            self.episode += 1
            self.multiPinNet_i = 0
        # print("[{}][{}]".format(self.multiPinNet_i, self.twoPinNet_i))
        self.init_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][0]
        self.goal_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][1]

        if self.twoPinNet_i == 0:
            for pos in self.netlist[self.multiPinNet_i]:
                x_pos = pos[0]
                y_pos = pos[1]
                z_pos = pos[2]
                self.eliminate_pin_effect_v1(x_pos, y_pos, z_pos)


# def a_star_solver_sim(start_pos, end_pos, i):
#     result1_table = [([[1, 2, 0], [2, 3, 0], [2, 4, 0]], 2),
#                      ([[2, 2, 0], [3, 3, 0], [2, 3, 0], [3, 4, 0]], 3),
#                      ([[3, 2, 0], [4, 3, 0], [4, 4, 0]], 2),
#                      ([[4, 2, 0], [5, 3, 0], [5, 4, 0]], 2),
#                      ([[5, 2, 0], [6, 3, 0], [6, 4, 0]], 2)]
#     result2_table = [([[1, 2, 0], [1, 3, 0], [2, 4, 0]], 2),
#                      ([[2, 2, 0], [3, 3, 0], [3, 4, 0]], 2),
#                      ([[3, 2, 0], [4, 3, 0], [4, 4, 0]], 2),
#                      ([[4, 2, 0], [5, 3, 0], [5, 4, 0]], 2),
#                      ([[5, 2, 0], [6, 3, 0], [6, 4, 0]], 2)]
#     if i < 5:
#         result = result1_table[i]
#     else:
#         result = result2_table[i-5]
#     return result


# if __name__ == '__main__':
#     benchmark_dir = 'benchmark'
#     for benchmark_file in os.listdir(benchmark_dir):
#         benchmark_file = benchmark_dir + '/' + benchmark_file
#         gridParameters = grid_parameters(benchmark_file)
#         gridEnv = GridEnv(gridParameters)
#         k = 0
#         while gridEnv.episode < 10:
#             gridEnv.reset()
#             if gridEnv.episode == 10:
#                 break
#             gridEnv.breakup()
#             route, cost = a_star_solver_sim(gridEnv.init_pos, gridEnv.goal_pos, k)
#             gridEnv.route = route
#             gridEnv.cost = cost
#             gridEnv.update()
#             k += 1
#         print(gridEnv)
