from GridParameters import GridParameters
from utils import diagonal_distance_3d, circle_space_set, rect_space_set


class GridEnv:
    def __init__(self, grid_parameter: GridParameters):
        """
        Create the grid environment based on grid_parameter

        :param grid_parameter: the data structure of the grid parameters
        """
        self.grid_parameter = grid_parameter

        self.grid_size = grid_parameter.gridSize
        """ the size of the grid graph """
        self.num_net = grid_parameter.netNum
        """ the number of nets """

        self.electric_width = []
        """ 
            It is a list of the electrical width, each electrical width is also a list and has two elements.
            The first element is calculated by track width and clearance.
            The second element is calculated by microvia width and clearance.
        """

        self.widthList = []
        """
        a list of the width of each net. The first element of the width is track_width, the second is microvia_width.
        """

        self.pad_size_array = []
        """
            It is a list of the coordinates of the pads.
            pad_size_array[i][j] is the j-st pad's coordinates of the i-st net
        """
        self.pin_hsize_array = []
        """
            It is a list of the pads' size.
            pin_hsize_array[i][j] is the j-st pad's size of the i-st net
            pin_hsize_array[i][j] = [x_hsize, y_hsize, shape]
        """
        self.pin_grid_set = {}
        """ It is a dict[str, set], which use string of the coordinate to index pad's grids. """

        self.twoPinNetCombo, self.twoPinNetNums, self.netlist = self.generate_two_pin_net()
        """
        twoPinNetCombo: a list of the two-pin nets from each net. The order of the two-pin nets is based on MST.
        twoPinNetNums: a list of the number of the two-pin nets form each net
        netlist: a list of the pad from each net
        """

        self.occupied_coord = self.generate_occupied_coord()
        """ a dict stores the occupied grids """

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

    def generate_two_pin_net(self) -> (list, list, list):
        """
        Create the list of two-pin nets from multi-pin nets based on MST

        :return: twoPinNetCombo, twoPinNetNums, netlist
        """
        single_net_pins = []
        netlist = []
        net_order = []
        for i in range(self.num_net):
            net_info = self.grid_parameter.netList[i + 1]

            # if i > 4:  # For Debug
            #     print(net_info.netName)
            #     netlist.append([])
            #     continue

            for j in range(len(net_info.padList)):
                single_net_pins.append(net_info.padList[j].position)
            netlist.append(single_net_pins)

            self.widthList.append((self.grid_parameter.netClass[net_info.netClass].track_width,
                                   self.grid_parameter.netClass[net_info.netClass].microvia_diameter))

            self.electric_width.append((self.grid_parameter.netClass[net_info.netClass].clearance_with_track,
                                        self.grid_parameter.netClass[net_info.netClass].clearance_with_microvia))

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

            # if i > 4:  # For Debug
            #     two_pin_net_nums.append(0)
            #     continue

            two_pin_net_nums.append(netlist_len - 1)

            for j in range(netlist_len - 1):
                for p in range(netlist_len - j - 1):
                    pin_start = netlist[i][j]
                    pin_end = netlist[i][p + j + 1]
                    distance = diagonal_distance_3d(pin_start, pin_end)
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
            # adjust the order to make the starting pad is belongs to connected component
            if len(two_pin_nets) > 0:
                p = 1
                q = p
                connected_nodes = {str(two_pin_nets[0][0]), str(two_pin_nets[0][1])}
                while p < len(two_pin_nets):
                    if str(two_pin_nets[q][0]) in connected_nodes:
                        connected_nodes.add(str(two_pin_nets[q][1]))
                        two_pin_nets[p], two_pin_nets[q] = two_pin_nets[q], two_pin_nets[p]
                        p += 1
                        q = p
                    elif str(two_pin_nets[q][1]) in connected_nodes:
                        connected_nodes.add(str(two_pin_nets[q][0]))
                        two_pin_nets[q][0], two_pin_nets[q][1] = two_pin_nets[q][1], two_pin_nets[q][0]
                        two_pin_nets[p], two_pin_nets[q] = two_pin_nets[q], two_pin_nets[p]
                        p += 1
                        q = p
                    else:
                        q += 1

            two_pin_nets_combo.append(two_pin_nets)
            two_pin_nets = []
            two_pin_set = []

        return two_pin_nets_combo, two_pin_net_nums, netlist

    def generate_occupied_coord(self) -> dict:
        """
        Create an initial dict which contain the obstacle grid

        :return: the dict of the occupied grids
        """
        occupied_dict = {}

        # Set grid graph refer to boundary list
        # TODO

        # Set the pads with a net
        for i in range(self.num_net):
            pin_hsize_list = []
            net_info = self.grid_parameter.netList[i + 1]
            pad_size_list = []
            for j in range(len(net_info.padList)):
                pad = net_info.padList[j]
                x_coord = pad.position[0]
                y_coord = pad.position[1]
                z_coord = pad.position[2]

                pad_size_set = set([])
                pin_set = set([])

                pin_hsize_x = int(pad.size[0] / 2)
                pin_hsize_y = int(pad.size[1] / 2)
                pin_hsize_list.append([pin_hsize_x, pin_hsize_y, pad.shape])

                if pad.type == 'thru_hole':
                    z_min = 0
                    z_max = self.grid_size[2]
                else:
                    # pad.type == 'smd'
                    z_min = z_coord
                    z_max = z_coord + 1

                if pad.shape == 'circle':  # ? boundary detection ?
                    for z_i in range(z_max - z_min):
                        pad_size_set = pad_size_set | circle_space_set([x_coord, y_coord, z_i], pin_hsize_x)
                        pin_set = pin_set | circle_space_set([x_coord, y_coord, z_i], pin_hsize_x - 1)
                else:
                    for z_i in range(z_max - z_min):
                        pad_size_set = pad_size_set | rect_space_set([x_coord, y_coord, z_i], pin_hsize_x, pin_hsize_y)
                        pin_set = pin_set | rect_space_set([x_coord, y_coord, z_i], pin_hsize_x - 1, pin_hsize_y - 1)

                pad_size_list.append(pad_size_set)

                for hide_pos in pad_size_set:
                    if hide_pos in occupied_dict:
                        occupied_dict[hide_pos] += 1000
                    else:
                        occupied_dict[hide_pos] = 1000  # Large Enough

                self.pin_grid_set[str(pad.position)] = pin_set

            self.pin_hsize_array.append(pin_hsize_list)
            self.pad_size_array.append(pad_size_list)

        # Set obstacle pad
        for obs_pad in self.grid_parameter.pad_obstacles:
            x_coord = obs_pad.position[0]
            y_coord = obs_pad.position[1]
            z_coord = obs_pad.position[2]

            pad_size_set = set([])
            pin_hsize_x = int(obs_pad.size[0] / 2)
            pin_hsize_y = int(obs_pad.size[1] / 2)

            if obs_pad.type == 'np_thru_hole' or obs_pad.type == 'thru_hole':
                z_min = 0
                z_max = self.grid_size[2]
            else:
                # pad.type == 'smd'
                z_min = z_coord
                z_max = z_coord + 1

            if obs_pad.shape == 'circle':  # ? boundary detection ?
                for z_i in range(z_max - z_min):
                    pad_size_set = pad_size_set | circle_space_set([x_coord, y_coord, z_i], pin_hsize_x)
            else:
                for z_i in range(z_max - z_min):
                    pad_size_set = pad_size_set | rect_space_set([x_coord, y_coord, z_i], pin_hsize_x, pin_hsize_y)

            for hide_pos in pad_size_set:
                if hide_pos in occupied_dict:
                    occupied_dict[hide_pos] += 1000
                else:
                    occupied_dict[hide_pos] = 1000  # Large Enough

        return occupied_dict

    def add_pin_effect(self, pad_size_set: set):
        """
        Add the base cost of the grids of pad_size_set to the occupied grids

        :param pad_size_set: the set of the coordinates of the pad
        :return:
        """
        for hide_pos in pad_size_set:
            if hide_pos in self.occupied_coord:
                self.occupied_coord[hide_pos] += 1000
            else:
                self.occupied_coord[hide_pos] = 1000  # Large Enough

    def eliminate_pin_effect(self, pad_size_set: set):
        """
        Delete the base cost of the grids of pad_size_set to the occupied grids

        :param pad_size_set: the set of the coordinates of the pad
        :return:
        """
        for hide_pos in pad_size_set:
            self.occupied_coord[hide_pos] -= 1000
            if self.occupied_coord[hide_pos] <= 0:
                del self.occupied_coord[str(hide_pos)]

    def set_route(self, origin_route: list):
        """
        store the inflection points of the path as the route result

        :param origin_route: the routing path with all the points
        :return:
        """
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

    def add_occupied_coord(self, route: list, distance: list):
        """
        Add the base cost of the routing path to the occupied grid

        :param route: the routing path
        :param distance: the first element is track_width, the second is microvia_width
        :return:
        """
        d_line = distance[0]
        via_d = distance[1]

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

    def del_occupied_coord(self, old_route: list, distance: list):
        """
        Add the base cost of the routing path to the occupied grid

        :param old_route: the routing path
        :param distance: the first element is track_width, the second is microvia_width
        :return:
        """
        d_line = distance[0]
        via_d = distance[1]

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

    def update(self, single_route: list, single_cost: float):
        """
        Update the route and cost in the routing environment

        :param single_route: the routing path
        :param single_cost: the routing cost
        :return:
        """
        if single_route:
            self.set_route(single_route)
            self.cost += single_cost

            # add the routed net and pad to the netPinSet
            for pos in single_route:
                self.netPinSet.add(str(pos))
            self.netPinSet = self.netPinSet | self.pin_grid_set[str(self.goal_pos)]

            self.netPinRoute.append(self.route)

            if self.twoPinNet_i >= self.twoPinNetNums[self.multiPinNet_i] - 1:
                # the multi-pin net is routed completely
                if self.episode > 0:
                    if self.cost > self.route_cost[self.multiPinNet_i]:
                        self.netPinRoute = self.old_netPinRoute
                        self.cost = self.route_cost[self.multiPinNet_i]
                    self.route_combo[self.multiPinNet_i] = self.netPinRoute
                    self.route_cost[self.multiPinNet_i] = self.cost
                else:
                    self.route_combo.append(self.netPinRoute)
                    self.route_cost.append(self.cost)

                for i in range(len(self.netlist[self.multiPinNet_i])):
                    self.add_pin_effect(self.pad_size_array[self.multiPinNet_i][i])

                for route in self.netPinRoute:
                    distance = self.widthList[self.multiPinNet_i]
                    self.add_occupied_coord(route, distance)

            self.twoPinNet_i += 1
        else:
            # the multi-pin net is still routing
            if self.episode > 0:
                self.route_combo[self.multiPinNet_i] = single_route
                self.route_cost[self.multiPinNet_i] = single_cost

            self.route_combo.append(single_route)
            self.route_cost.append(single_cost)

    def breakup(self):
        # breakup the routing
        if self.episode > 0 and self.twoPinNet_i == 0:
            self.old_netPinRoute = self.route_combo[self.multiPinNet_i]
            self.route_combo[self.multiPinNet_i] = []
            for old_route in self.old_netPinRoute:
                distance = self.widthList[self.multiPinNet_i]
                self.del_occupied_coord(old_route, distance)

    def reset(self):
        """
        Reset the routing environment and prepare the next routing

        :return:
        """
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
            if self.multiPinNet_i >= len(self.twoPinNetCombo):
                # all the two pin nets are routed
                self.episode_cost.append(sum(self.route_cost))
                self.episode += 1
                self.multiPinNet_i = 0
            return

        print("NetCombo[{}][{}]".format(self.multiPinNet_i, self.twoPinNet_i))
        init_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][0]
        self.init_pos = init_pos
        # self.init_pos = self.pin_grid_set[str(init_pos)]
        self.goal_pos = self.twoPinNetCombo[self.multiPinNet_i][self.twoPinNet_i][1]

        if self.twoPinNet_i == 0:
            for i in range(len(self.netlist[self.multiPinNet_i])):
                self.eliminate_pin_effect(self.pad_size_array[self.multiPinNet_i][i])
            self.netPinSet = self.netPinSet | self.pin_grid_set[str(init_pos)]

    def merge_route(self) -> list:
        """
        Generate the routing paths to segments
        :return: the list of route segments
        """
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
