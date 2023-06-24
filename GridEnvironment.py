import copy
import os
import random

import numpy as np

from utils import diagonal_distance


class GridEnv:
    def __init__(self, grid_parameter, type_=0):
        self.grid_parameter = grid_parameter
        self.grid_size = grid_parameter.gridSize
        self.num_net = grid_parameter.netNum

        self.electric_width = 3
        self.wire_max_width = None

        self.pin_hsize_array = []
        self.pin_grid_set = {}

        self.twoPinNetCombo, self.twoPinNetNums, self.netlist, self.widthList = self.generate_two_pin_net()

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
        self.wire_max_width = 0
        single_net_pins = []
        netlist = []
        net_order = []
        width_list = []
        for i in range(self.num_net):
            net_info = self.grid_parameter.netList[i + 1]
            for j in range(len(net_info.padList)):
                single_net_pins.append(net_info.padList[j].position)
            netlist.append(single_net_pins)
            if self.grid_parameter.netClass[net_info.netClass].track_width > self.wire_max_width:
                self.wire_max_width = self.grid_parameter.netClass[net_info.netClass].track_width
            width_list.append((self.grid_parameter.netClass[net_info.netClass].track_width,
                               self.grid_parameter.netClass[net_info.netClass].microvia_diameter))
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
            connected_component = []
            for j in range(len(two_pin_set)):
                is_loop = True
                if not connected_component:
                    fetched_pins = [two_pin_set[j][0][0], two_pin_set[j][0][1]]
                    connected_component.append(fetched_pins)
                    is_loop = False
                else:
                    static_c_i = [None, None]
                    for c_i in range(len(connected_component)):
                        if two_pin_set[j][0][0] not in connected_component[c_i] \
                                and two_pin_set[j][0][1] not in connected_component[c_i]:
                            if c_i == len(connected_component) - 1:
                                if static_c_i[0] is None:
                                    fetched_pins = [two_pin_set[j][0][0], two_pin_set[j][0][1]]
                                    connected_component.append(fetched_pins)
                                    is_loop = False
                                    break
                                else:
                                    connected_component[static_c_i[0]].append(two_pin_set[j][0][static_c_i[1]])
                                    is_loop = False
                                    break
                            else:
                                continue
                        else:
                            if two_pin_set[j][0][0] in connected_component[c_i] \
                                    and two_pin_set[j][0][1] not in connected_component[c_i]:
                                if static_c_i[0] is not None:
                                    for pin in connected_component[c_i]:
                                        connected_component[static_c_i[0]].append(pin)
                                    connected_component.pop(c_i)
                                    is_loop = False
                                    break
                                else:
                                    if c_i == len(connected_component) - 1:
                                        connected_component[c_i].append(two_pin_set[j][0][1])
                                        is_loop = False
                                        break
                                    else:
                                        static_c_i[0] = c_i
                                        static_c_i[1] = 1
                                        continue
                            elif two_pin_set[j][0][0] not in connected_component[c_i] \
                                    and two_pin_set[j][0][1] in connected_component[c_i]:
                                if static_c_i[0] is not None:
                                    for pin in connected_component[c_i]:
                                        connected_component[static_c_i[0]].append(pin)
                                    connected_component.pop(c_i)
                                    is_loop = False
                                    break
                                else:
                                    if c_i == len(connected_component) - 1:
                                        connected_component[c_i].append(two_pin_set[j][0][0])
                                        is_loop = False
                                        break
                                    else:
                                        static_c_i[0] = c_i
                                        static_c_i[1] = 0
                                        continue
                            else:
                                is_loop = True
                                break
                if not is_loop:
                    two_pin_nets.append(two_pin_set[j][0])
                if len(two_pin_nets) >= netlist_len - 1:
                    break
                # not_loop = False
                # if two_pin_set[j][0][0] not in fetched_pins:
                #     not_loop = True
                #     fetched_pins.append(two_pin_set[j][0][0])
                # if two_pin_set[j][0][1] not in fetched_pins:
                #     not_loop = True
                #     fetched_pins.append(two_pin_set[j][0][1])
                # if len(fetched_pins) >= netlist_len:
                #     not_loop = True
                # if not_loop:
                #     two_pin_nets.append(two_pin_set[j][0])
                # if len(two_pin_nets) >= netlist_len - 1:
                #     break
            two_pin_nets_combo.append(two_pin_nets)
            two_pin_nets = []
            two_pin_set = []

        return two_pin_nets_combo, two_pin_net_nums, netlist, width_list

    def generate_occupied_coord(self):
        occupied_dict = {}
        # Set grid graph refer to boundary list
        # TODO
        for i in range(self.num_net):
            net_info = self.grid_parameter.netList[i + 1]
            pin_hsize_list = []
            for j in range(len(net_info.padList)):
                pad = net_info.padList[j]
                x_coord = pad.position[0]
                y_coord = pad.position[1]
                z_coord = pad.position[2]

                pin_hsize_x = int(pad.size[0] / 2)
                pin_hsize_y = int(pad.size[1] / 2)
                pin_hsize_list.append([pin_hsize_x, pin_hsize_y, pad.type])

                if pin_hsize_x == pin_hsize_y:
                    size_x_min = x_coord - int(pin_hsize_x * 0.707)
                    size_x_max = x_coord + int(pin_hsize_x * 0.707) + 1
                    size_y_min = y_coord - int(pin_hsize_y * 0.707)
                    size_y_max = y_coord + int(pin_hsize_y * 0.707) + 1
                else:
                    size_x_min = x_coord - pin_hsize_x
                    size_x_max = x_coord + pin_hsize_x + 1
                    size_y_min = y_coord - pin_hsize_y
                    size_y_max = y_coord + pin_hsize_y + 1

                pin_hsize_x += self.electric_width + self.wire_max_width
                pin_hsize_y += self.electric_width + self.wire_max_width

                x_min = x_coord - pin_hsize_x
                x_max = x_coord + pin_hsize_x + 1
                y_min = y_coord - pin_hsize_y
                y_max = y_coord + pin_hsize_y + 1
                if pad.type == 'thru_hole':
                    z_min = 0
                    z_max = self.grid_size[2]
                else:
                    # pad.type == 'smd'
                    z_min = z_coord
                    z_max = z_coord + 1

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
                # Set the cost of the pin
                for x_i in range(x_max - x_min):
                    for y_i in range(y_max - y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                            if str(hide_pos) in occupied_dict:
                                occupied_dict[str(hide_pos)] += 1000
                            else:
                                occupied_dict[str(hide_pos)] = 1000  # Large Enough

                pin_set = set([])
                for x_i in range(size_x_max - size_x_min):
                    for y_i in range(size_y_max - size_y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [size_x_min + x_i, size_y_min + y_i, z_min + z_i]
                            pin_set.add(str(hide_pos))
                self.pin_grid_set[str(pad.position)] = pin_set

                # occupied_dict[str(net_info[str(j + 1)])] = 1000  # Large Enough
            self.pin_hsize_array.append(pin_hsize_list)

        # Set obstacle
        for obs_pad in self.grid_parameter.pad_obstacles:
            x_coord = obs_pad.position[0]
            y_coord = obs_pad.position[1]
            z_coord = obs_pad.position[2]

            pin_hsize_x = int(obs_pad.size[0] / 2) + self.electric_width + self.wire_max_width
            pin_hsize_y = int(obs_pad.size[1] / 2) + self.electric_width + self.wire_max_width

            x_min = x_coord - pin_hsize_x
            x_max = x_coord + pin_hsize_x + 1
            y_min = y_coord - pin_hsize_y
            y_max = y_coord + pin_hsize_y + 1
            if obs_pad.type == 'np_thru_hole' or obs_pad.type == 'thru_hole':
                z_min = 0
                z_max = self.grid_size[2]
            else:
                # pad.type == 'smd'
                z_min = z_coord
                z_max = z_coord + 1

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
            # Set the cost of the pin
            for x_i in range(x_max - x_min):
                for y_i in range(y_max - y_min):
                    for z_i in range(z_max - z_min):
                        hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                        if str(hide_pos) in occupied_dict:
                            occupied_dict[str(hide_pos)] += 1000
                        else:
                            occupied_dict[str(hide_pos)] = 1000  # Large Enough

        return occupied_dict

    def add_pin_effect(self, pin_x, pin_y, pin_z, pin_hsize):
        pin_hsize_x, pin_hsize_y, pin_type = pin_hsize
        pin_hsize_x += self.electric_width + self.wire_max_width
        pin_hsize_y += self.electric_width + self.wire_max_width
        x_min = pin_x - pin_hsize_x
        x_max = pin_x + pin_hsize_x + 1
        y_min = pin_y - pin_hsize_y
        y_max = pin_y + pin_hsize_y + 1
        if pin_type == 'thru_hole':
            z_min = 0
            z_max = self.grid_size[2]
        else:
            # pad.type == 'smd'
            z_min = pin_z
            z_max = pin_z + 1

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
        # Add the cost of the specified pin
        for x_i in range(x_max - x_min):
            for y_i in range(y_max - y_min):
                for z_i in range(z_max - z_min):
                    hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                    if str(hide_pos) in self.occupied_coord:
                        self.occupied_coord[str(hide_pos)] += 1000
                    else:
                        self.occupied_coord[str(hide_pos)] = 1000  # Large Enough

    def add_pin_effect_v1(self, pin_x, pin_y, pin_z, pin_hsize):
        pin_hsize_x, pin_hsize_y, pin_type = pin_hsize
        pin_hsize_x += self.electric_width + self.wire_max_width
        pin_hsize_y += self.electric_width + self.wire_max_width
        x_min = pin_x - pin_hsize_x
        x_max = pin_x + pin_hsize_x + 1
        y_min = pin_y - pin_hsize_y
        y_max = pin_y + pin_hsize_y + 1
        if pin_type == 'thru_hole':
            z_min = 0
            z_max = self.grid_size[2]
        else:
            # pad.type == 'smd'
            z_min = pin_z
            z_max = pin_z + 1

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
        # Add the cost of the specified pin
        self.grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] += 1000

    def eliminate_pin_effect(self, pin_x, pin_y, pin_z, pin_hsize):
        pin_hsize_x, pin_hsize_y, pin_type = pin_hsize
        pin_hsize_x += self.electric_width + self.wire_max_width
        pin_hsize_y += self.electric_width + self.wire_max_width
        x_min = pin_x - pin_hsize_x
        x_max = pin_x + pin_hsize_x + 1
        y_min = pin_y - pin_hsize_y
        y_max = pin_y + pin_hsize_y + 1
        if pin_type == 'thru_hole':
            z_min = 0
            z_max = self.grid_size[2]
        else:
            # pad.type == 'smd'
            z_min = pin_z
            z_max = pin_z + 1

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
        # Del the cost of the specified pin
        for x_i in range(x_max - x_min):
            for y_i in range(y_max - y_min):
                for z_i in range(z_max - z_min):
                    hide_pos = [x_min + x_i, y_min + y_i, z_min + z_i]
                    self.occupied_coord[str(hide_pos)] -= 1000
                    if self.occupied_coord[str(hide_pos)] <= 0:
                        del self.occupied_coord[str(hide_pos)]

    def eliminate_pin_effect_v1(self, pin_x, pin_y, pin_z, pin_hsize):
        pin_hsize_x, pin_hsize_y, pin_type = pin_hsize
        pin_hsize_x += self.electric_width + self.wire_max_width
        pin_hsize_y += self.electric_width + self.wire_max_width
        x_min = pin_x - pin_hsize_x
        x_max = pin_x + pin_hsize_x + 1
        y_min = pin_y - pin_hsize_y
        y_max = pin_y + pin_hsize_y + 1
        if pin_type == 'thru_hole':
            z_min = 0
            z_max = self.grid_size[2]
        else:
            # pad.type == 'smd'
            z_min = pin_z
            z_max = pin_z + 1

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
        # Del the cost of the specified pin
        self.grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] -= 1000

    def generate_grid_graph(self):
        grid_graph = np.zeros(self.grid_size)
        # Set grid graph refer to boundary list
        # TODO
        for i in range(self.num_net):
            net_info = self.grid_parameter.netList[i + 1]
            pin_hsize_list = []
            for j in range(len(net_info.padList)):
                pad = net_info.padList[j]
                x_coord = pad.position[0]
                y_coord = pad.position[1]
                z_coord = pad.position[2]

                pin_hsize_x = int(pad.size[0] / 2)
                pin_hsize_y = int(pad.size[1] / 2)
                pin_hsize_list.append([pin_hsize_x, pin_hsize_y, pad.type])

                if pin_hsize_x == pin_hsize_y:
                    size_x_min = x_coord - int(pin_hsize_x * 0.707)
                    size_x_max = x_coord + int(pin_hsize_x * 0.707) + 1
                    size_y_min = y_coord - int(pin_hsize_y * 0.707)
                    size_y_max = y_coord + int(pin_hsize_y * 0.707) + 1
                else:
                    size_x_min = x_coord - pin_hsize_x
                    size_x_max = x_coord + pin_hsize_x + 1
                    size_y_min = y_coord - pin_hsize_y
                    size_y_max = y_coord + pin_hsize_y + 1

                pin_hsize_x += self.electric_width + self.wire_max_width
                pin_hsize_y += self.electric_width + self.wire_max_width

                x_min = x_coord - pin_hsize_x
                x_max = x_coord + pin_hsize_x + 1
                y_min = y_coord - pin_hsize_y
                y_max = y_coord + pin_hsize_y + 1
                if pad.type == 'thru_hole':
                    z_min = 0
                    z_max = self.grid_size[2]
                else:
                    # pad.type == 'smd'
                    z_min = z_coord
                    z_max = z_coord + 1

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
                # Set the cost of the pin
                grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] += 1000

                pin_set = set([])
                for x_i in range(size_x_max - size_x_min):
                    for y_i in range(size_y_max - size_y_min):
                        for z_i in range(z_max - z_min):
                            hide_pos = [size_x_min + x_i, size_y_min + y_i, z_min + z_i]
                            pin_set.add(str(hide_pos))
                self.pin_grid_set[str(pad.position)] = pin_set

            self.pin_hsize_array.append(pin_hsize_list)
        # Set obstacle
        for obs_pad in self.grid_parameter.pad_obstacles:
            x_coord = obs_pad.position[0]
            y_coord = obs_pad.position[1]
            z_coord = obs_pad.position[2]

            pin_hsize_x = int(obs_pad.size[0] / 2) + self.electric_width + self.wire_max_width
            pin_hsize_y = int(obs_pad.size[1] / 2) + self.electric_width + self.wire_max_width

            x_min = x_coord - pin_hsize_x
            x_max = x_coord + pin_hsize_x + 1
            y_min = y_coord - pin_hsize_y
            y_max = y_coord + pin_hsize_y + 1
            if obs_pad.type == 'np_thru_hole' or obs_pad.type == 'thru_hole':
                z_min = 0
                z_max = self.grid_size[2]
            else:
                # pad.type == 'smd'
                z_min = z_coord
                z_max = z_coord + 1

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
            # Set the cost of the pin
            grid_graph[x_min:x_max, y_min:y_max, z_min:z_max] += 1000

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

    def add_occupied_coord(self, route, distance):
        d_line = distance[0] + self.electric_width
        via_d = distance[1] + self.electric_width
        for i in range(len(route) - 1):
            x_0, y_0, z_0 = route[i]
            x_1, y_1, z_1 = route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                if direct[1] == 0:  # go through a via
                    x_max = x_0 + via_d + 1
                    x_min = x_0 - via_d
                    y_max = y_0 + via_d + 1
                    y_min = y_0 - via_d
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + d_line + 1
                        y_min = y_0 - d_line
                    else:  # go to the south
                        y_max = y_0 + d_line + 1
                        y_min = y_1 - d_line
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
                    x_max = x_1 + d_line + 1
                    x_min = x_0 - d_line
                else:  # go to the west
                    x_max = x_0 + d_line + 1
                    x_min = x_1 - d_line
                y_max = y_0 + d_line + 1
                y_min = y_0 - d_line
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
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    y_max = y_0 + d_line + 1
                    y_min = y_0 - d_line

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

    def add_route_occupied(self, route, distance):
        d_line = distance[0] + self.electric_width
        via_d = distance[1] + self.electric_width
        for i in range(len(route) - 1):
            x_0, y_0, z_0 = route[i]
            x_1, y_1, z_1 = route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                if direct[1] == 0:  # go through a via
                    x_max = x_0 + via_d + 1
                    x_min = x_0 - via_d
                    y_max = y_0 + via_d + 1
                    y_min = y_0 - via_d
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + d_line + 1
                        y_min = y_0 - d_line
                    else:  # go to the south
                        y_max = y_0 + d_line + 1
                        y_min = y_1 - d_line
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
                    x_max = x_1 + d_line + 1
                    x_min = x_0 - d_line
                else:  # go to the west
                    x_max = x_0 + d_line + 1
                    x_min = x_1 - d_line
                y_max = y_0 + d_line + 1
                y_min = y_0 - d_line
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
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    y_max = y_0 + d_line + 1
                    y_min = y_0 - d_line

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

    def del_occupied_coord(self, old_route, distance):
        d_line = distance[0] + self.electric_width
        via_d = distance[1] + self.electric_width
        for i in range(len(old_route) - 1):
            x_0, y_0, z_0 = old_route[i]
            x_1, y_1, z_1 = old_route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                if direct[1] == 0:  # go through a via
                    x_max = x_0 + via_d + 1
                    x_min = x_0 - via_d
                    y_max = y_0 + via_d + 1
                    y_min = y_0 - via_d
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + d_line + 1
                        y_min = y_0 - d_line
                    else:  # go to the south
                        y_max = y_0 + d_line + 1
                        y_min = y_1 - d_line
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
                    x_max = x_1 + d_line + 1
                    x_min = x_0 - d_line
                else:  # go to the west
                    x_max = x_0 + d_line + 1
                    x_min = x_1 - d_line
                y_max = y_0 + d_line + 1
                y_min = y_0 - d_line

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
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    y_max = y_0 + d_line + 1
                    y_min = y_0 - d_line

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

    def del_route_occupied(self, old_route, distance):
        d_line = distance[0] + self.electric_width
        via_d = distance[1] + self.electric_width
        for i in range(len(old_route) - 1):
            x_0, y_0, z_0 = old_route[i]
            x_1, y_1, z_1 = old_route[i + 1]
            direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
            if direct[0] == 0:
                if direct[1] == 0:  # go through a via
                    x_max = x_0 + via_d + 1
                    x_min = x_0 - via_d
                    y_max = y_0 + via_d + 1
                    y_min = y_0 - via_d
                    if z_0 < z_1:
                        z_max = z_1 + 1
                        z_min = z_0
                    else:
                        z_max = z_0 + 1
                        z_min = z_1
                else:  # go to the north or south
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    if y_0 < y_1:  # go to the north
                        y_max = y_1 + d_line + 1
                        y_min = y_0 - d_line
                    else:  # go to the south
                        y_max = y_0 + d_line + 1
                        y_min = y_1 - d_line
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
                    x_max = x_1 + d_line + 1
                    x_min = x_0 - d_line
                else:  # go to the west
                    x_max = x_0 + d_line + 1
                    x_min = x_1 - d_line
                y_max = y_0 + d_line + 1
                y_min = y_0 - d_line

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
                    x_max = x_0 + d_line + 1
                    x_min = x_0 - d_line
                    y_max = y_0 + d_line + 1
                    y_min = y_0 - d_line

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
        if single_route:
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

                i = 0
                for pos in self.netlist[self.multiPinNet_i]:
                    x_pos = pos[0]
                    y_pos = pos[1]
                    z_pos = pos[2]
                    self.add_pin_effect(x_pos, y_pos, z_pos, self.pin_hsize_array[self.multiPinNet_i][i])
                    i += 1

                for route in self.netPinRoute:
                    distance = self.widthList[self.multiPinNet_i]
                    self.add_occupied_coord(route, distance)

            self.twoPinNet_i += 1
        else:
            if self.episode > 0:
                self.route_combo[self.multiPinNet_i] = single_route
                self.route_cost[self.multiPinNet_i] = single_cost
            self.route_combo.append(single_route)
            self.route_cost.append(single_cost)

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
        if single_route:
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

                i = 0
                for pos in self.netlist[self.multiPinNet_i]:
                    x_pos = pos[0]
                    y_pos = pos[1]
                    z_pos = pos[2]
                    self.add_pin_effect_v1(x_pos, y_pos, z_pos, self.pin_hsize_array[self.multiPinNet_i][i])
                    i += 1

                for route in self.netPinRoute:
                    distance = self.widthList[self.multiPinNet_i]
                    self.add_route_occupied(route, distance)

            self.twoPinNet_i += 1
        else:
            if self.episode > 0:
                self.route_combo[self.multiPinNet_i] = single_route
                self.route_cost[self.multiPinNet_i] = single_cost
            self.route_combo.append(single_route)
            self.route_cost.append(single_cost)

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
                distance = self.widthList[self.multiPinNet_i]
                self.del_occupied_coord(old_route, distance)

    def breakup_v1(self):
        # breakup the routing
        if self.episode > 0 and self.twoPinNet_i == 0:
            self.old_netPinRoute = self.route_combo[self.multiPinNet_i]
            self.route_combo[self.multiPinNet_i] = []
            for old_route in self.old_netPinRoute:
                distance = self.widthList[self.multiPinNet_i]
                self.del_route_occupied(old_route, distance)

    def reset(self):
        self.route = []

        # initialize state of gridEnv by the two pin net(self.twoPinNet_i)
        if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i]:
            if self.twoPinNetNums[self.multiPinNet_i] == 0:
                # the net has only one pin
                self.cost = 0
                self.netPinSet = set([])
                self.netPinRoute = []
                self.twoPinNet_i = 0
                self.multiPinNet_i += 1
                self.init_pos = None
                self.goal_pos = None
                return
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

        if self.twoPinNetNums[self.multiPinNet_i] == 0:
            # the net has only one pin
            self.cost = 0
            self.netPinSet = set([])
            self.netPinRoute = []
            self.twoPinNet_i = 0
            self.multiPinNet_i += 1
            self.init_pos = None
            self.goal_pos = None
            return

        print("[{}][{}]".format(self.multiPinNet_i, self.twoPinNet_i))
        self.init_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][0]
        self.goal_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][1]

        if self.twoPinNet_i == 0:
            i = 0
            for pos in self.netlist[self.multiPinNet_i]:
                x_pos = pos[0]
                y_pos = pos[1]
                z_pos = pos[2]
                self.eliminate_pin_effect(x_pos, y_pos, z_pos, self.pin_hsize_array[self.multiPinNet_i][i])
                i += 1

    def reset_v1(self):
        self.route = []

        # initialize state of gridEnv by the two pin net(self.twoPinNet_i)
        if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i]:
            if self.twoPinNetNums[self.multiPinNet_i] == 0:
                # the net has only one pin
                self.cost = 0
                self.netPinSet = set([])
                self.netPinRoute = []
                self.twoPinNet_i = 0
                self.multiPinNet_i += 1
                self.init_pos = None
                self.goal_pos = None
                return
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

        if self.twoPinNetNums[self.multiPinNet_i] == 0:
            # the net has only one pin
            self.cost = 0
            self.netPinSet = set([])
            self.netPinRoute = []
            self.twoPinNet_i = 0
            self.multiPinNet_i += 1
            self.init_pos = None
            self.goal_pos = None
            return

        print("[{}][{}]".format(self.multiPinNet_i, self.twoPinNet_i))
        self.init_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][0]
        self.goal_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][1]

        if self.twoPinNet_i == 0:
            i = 0
            for pos in self.netlist[self.multiPinNet_i]:
                x_pos = pos[0]
                y_pos = pos[1]
                z_pos = pos[2]
                self.eliminate_pin_effect_v1(x_pos, y_pos, z_pos, self.pin_hsize_array[self.multiPinNet_i][i])
                i += 1

    def merge_route(self):
        merge_route_combo = []
        for route_list in self.route_combo:
            merge_route_list = []
            for route in route_list:
                for i in range(len(route) - 1):
                    start = route[i]
                    end = route[i + 1]
                    merge_route_list.append([start, end])
            merge_route_combo.append(merge_route_list)

        return merge_route_combo

