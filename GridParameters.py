import math
from kiutils.board import Board
from kiutils.items.brditems import Segment, Via
from kiutils.items.common import Position

from kiutils_pro import KiCadPro


class Pad:
    def __init__(self, pos, layer, size, pad_type, net_id):
        self.position = pos
        self.layer = layer
        self.size = size
        self.type = pad_type
        self.netID = net_id


class Net:
    def __init__(self, net_id, net_name, net_class):
        self.netID = net_id
        self.netName = net_name
        self.netClass = net_class
        self.padList = []


class NetClass:
    def __init__(self, track_width, microvia_diameter, microvia_drill):
        self.track_width = to_grid_coord(track_width)
        self.microvia_diameter = to_grid_coord(microvia_diameter)
        self.microvia_drill = to_grid_coord(microvia_drill)


def to_grid_coord(coord_i):
    return int(coord_i * 3 / 0.25)


class GridParameters:
    def __init__(self, kicad_pcb, kicad_pro):
        self.filename = kicad_pcb
        board = Board().from_file(kicad_pcb)

        project = KiCadPro().from_file(kicad_pro)

        boundary_list = board.graphicItems
        self.dia_pos_0 = [boundary_list[0].start.X, boundary_list[0].start.Y]
        self.dia_pos_1 = [boundary_list[0].start.X, boundary_list[0].start.Y]
        pos_set = set([])
        for gr_line in boundary_list:
            start_pos = [gr_line.start.X, gr_line.start.Y]
            end_pos = [gr_line.end.X, gr_line.end.Y]
            if str(start_pos) not in pos_set:
                pos_set.add(str(start_pos))
                if self.dia_pos_0[0] > start_pos[0]:
                    self.dia_pos_0[0] = start_pos[0]
                if self.dia_pos_0[1] > start_pos[1]:
                    self.dia_pos_0[1] = start_pos[1]
                if self.dia_pos_1[0] < start_pos[0]:
                    self.dia_pos_1[0] = start_pos[0]
                if self.dia_pos_1[1] < start_pos[1]:
                    self.dia_pos_1[1] = start_pos[1]

            if str(end_pos) not in pos_set:
                pos_set.add(str(end_pos))
                if self.dia_pos_0[0] > end_pos[0]:
                    self.dia_pos_0[0] = end_pos[0]
                if self.dia_pos_0[1] > end_pos[1]:
                    self.dia_pos_0[1] = end_pos[1]
                if self.dia_pos_1[0] < end_pos[0]:
                    self.dia_pos_1[0] = end_pos[0]
                if self.dia_pos_1[1] < end_pos[1]:
                    self.dia_pos_1[1] = end_pos[1]

        layers = {}
        layers_ = {}
        i = 0
        for layer in board.layers:
            if layer.type == 'signal':
                layers[layer.name] = i
                layers_[i] = layer.name
            i += 1

        net_list = []
        for net in board.nets:
            # parse net_class class
            if net.name != '':
                board_net = Net(net.number, net.name, project.netSetting.netClassPatterns[net.name])
                net_list.append(board_net)
            else:
                board_net = Net(net.number, net.name, None)
                net_list.append(board_net)
        # net_list.pop(0)
        self.pad_obstacles = []
        for footprint in board.footprints:
            for pad in footprint.pads:
                if footprint.position.angle is None:
                    theta = 0
                else:
                    theta = footprint.position.angle * math.pi / 180
                dx = pad.position.X * math.cos(theta) + pad.position.Y * math.sin(theta)
                dy = pad.position.Y * math.cos(theta) - pad.position.X * math.sin(theta)
                x = footprint.position.X + dx
                y = footprint.position.Y + dy
                pad_pos = [to_grid_coord(x - self.dia_pos_0[0]),
                           to_grid_coord(y - self.dia_pos_0[1]),
                           layers[footprint.layer]]
                if pad.position.angle is None:
                    alpha = 0
                else:
                    alpha = pad.position.angle * math.pi / 180
                size_x = pad.size.X * math.cos(alpha) + pad.size.Y * math.sin(alpha)
                size_y = pad.size.Y * math.cos(alpha) - pad.size.X * math.sin(alpha)
                pad_size = [to_grid_coord(abs(size_x)), to_grid_coord(abs(size_y))]
                if pad.net:
                    board_pad = Pad(pad_pos, footprint.layer, pad_size, pad.type, pad.net.number)
                    net_list[pad.net.number].padList.append(board_pad)
                else:
                    board_pad = Pad(pad_pos, footprint.layer, pad_size, pad.type, None)
                    self.pad_obstacles.append(board_pad)

        self.gridSize = [to_grid_coord(self.dia_pos_1[0] - self.dia_pos_0[0]),
                         to_grid_coord(self.dia_pos_1[1] - self.dia_pos_0[1]),
                         len(layers)]  # grid size
        self.layers = layers_
        self.grLines = boundary_list
        self.netNum = len(net_list) - 1
        self.netClassReal = project.netSetting.classes
        self.netClass = {}
        for net_class in self.netClassReal:
            self.netClass[net_class] = NetClass(self.netClassReal[net_class].track_width,
                                                self.netClassReal[net_class].microvia_diameter,
                                                self.netClassReal[net_class].microvia_drill)
        self.netList = net_list

        board.to_file()

    def to_real_coord(self, grid_coord):
        grid_x = grid_coord[0] / 3 * 0.25
        grid_y = grid_coord[1] / 3 * 0.25
        x = self.dia_pos_0[0] + grid_x
        y = self.dia_pos_0[1] + grid_y
        layer = self.layers[grid_coord[2]]
        return [x, y, layer]

    def store_route(self, merge_route_combo):
        board = Board().from_file(self.filename)
        i = 1
        item_id = 0
        for net in merge_route_combo:
            for segment in net:
                start = self.to_real_coord(segment[0])
                end = self.to_real_coord(segment[1])
                start_pos = Position(start[0], start[1])
                end_pos = Position(end[0], end[1])
                if start[2] == end[2]:
                    width = self.netClassReal[self.netList[i].netClass].track_width
                    layer = start[2]
                    item = Segment(start_pos, end_pos, width, layer, False, i, str(item_id))
                    board.traceItems.append(item)
                else:
                    size = self.netClassReal[self.netList[i].netClass].microvia_diameter
                    drill = self.netClassReal[self.netList[i].netClass].microvia_drill
                    layers = [self.layers[0], self.layers[1]]
                    item = Via('micro', False, start_pos, size, drill, layers, False, False, False, i, str(item_id))
                    board.traceItems.append(item)
                item_id += 1
            i += 1
        board.to_file()
