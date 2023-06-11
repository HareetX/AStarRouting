import os

from utils import read


def grid_parameters(grid_file):
    grid_info = read(grid_file)
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
        gp = grid_parameters(benchmark_file)
