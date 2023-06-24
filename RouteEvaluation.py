def route_length(route_combo, via_cost=0):
    route_len = 0
    for j in range(len(route_combo)):
        for k in range(len(route_combo[j])):
            for i in range(len(route_combo[j][k]) - 1):
                x_0, y_0, z_0 = route_combo[j][k][i]
                x_1, y_1, z_1 = route_combo[j][k][i + 1]
                direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
                if direct[0] == 0:
                    if direct[1] == 0:  # go through a via
                        route_len += abs(direct[2]) * via_cost
                    else:  # go to the north or south
                        route_len += abs(direct[1])
                elif direct[1] == 0:  # go to the east or west
                    route_len += abs(direct[0])
                else:  # go diagonally
                    route_len += abs(direct[0]) * 1.414

    return route_len


def min_via_amount(netlist):
    min_via = 0
    layers = []
    for net in netlist:
        for i in range(len(net)):
            if net[i][2] not in layers:
                layers.append(net[i][2])
        min_via += len(layers) - 1
        layers = []

    return min_via


def via_amount(route_combo):
    via_num = 0
    for j in range(len(route_combo)):
        for k in range(len(route_combo[j])):
            for i in range(len(route_combo[j][k]) - 1):
                x_0, y_0, z_0 = route_combo[j][k][i]
                x_1, y_1, z_1 = route_combo[j][k][i + 1]
                direct = [x_1 - x_0, y_1 - y_0, z_1 - z_0]
                if direct[2] != 0:  # go through a via
                    via_num += 1

    return via_num
