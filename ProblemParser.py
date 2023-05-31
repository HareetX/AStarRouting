import os


def read(gr_file):
    file = open(gr_file, 'r')
    grid_info = {}
    i = 0
    for line in file:
        if not line.strip():
            continue
        else:
            grid_info[i] = line.split()
        i += 1
    file.close()
    return grid_info


def grid_parameters(grid_info):
    grid_parameter = {}
    net_parameters_store = []
    for lineNum in range(len(grid_info)):
        if 'grid' in grid_info[lineNum]:
            grid_parameter['gridSize'] = \
                [int(grid_info[lineNum][1]), int(grid_info[lineNum][2]), int(grid_info[lineNum][3])]
        elif 'num' in grid_info[lineNum]:
            grid_parameter['numNet'] = int(grid_info[lineNum][2])
        elif 'A' in grid_info[lineNum][0]:
            net_parameters = {'netName': grid_info[lineNum][0], 'netID': int(grid_info[lineNum][1]),
                              'numPins': int(grid_info[lineNum][2])}
            for i in range(int(grid_info[lineNum][2])):
                net_parameters[str(i + 1)] = \
                    [int(grid_info[lineNum + i + 1][0]), int(grid_info[lineNum + i + 1][1]),
                     int(grid_info[lineNum + i + 1][2])]

            net_parameters['route'] = []

            net_parameters_store.append(net_parameters)
        # Parse Obstacle information
        # TODO
        else:
            continue
    grid_parameter['netInfo'] = net_parameters_store

    return grid_parameter


if __name__ == '__main__':
    benchmark_dir = 'benchmark'
    for benchmark_file in os.listdir(benchmark_dir):
        benchmark_file = benchmark_dir + '/' + benchmark_file
        benchmark_info = read(benchmark_file)
        gp = grid_parameters(benchmark_info)
