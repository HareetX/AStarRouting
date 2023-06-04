import numpy as np
from matplotlib import pyplot as plt


def draw_cost_plot(cost_list, benchmark_i):
    n = np.linspace(1, len(cost_list), len(cost_list))
    plt.figure()
    plt.plot(n, cost_list)
    plt.xlabel('episodes')
    plt.ylabel('cost')
    plt.savefig('benchmark_{dumpBench}.AStarCostPlot.jpg'.format(dumpBench=benchmark_i + 1))
    plt.close()


def draw_origin_grid_plot(grid_parameter, benchmark_i=0):
    grid_size = grid_parameter['gridSize']
    layer_num = grid_size[2]
    pins = []

    for i in range(grid_parameter['numNet']):
        net_info = grid_parameter['netInfo'][i]
        for j in range(net_info['numPins']):
            pins.append(net_info[str(j + 1)])

    for layer in range(layer_num):
        plt.figure()
        o_i = 0
        # draw the start & end
        for i in range(len(pins)):
            if pins[i][2] == layer:
                if o_i == 0:
                    plt.plot(pins[i][0], pins[i][1], 'o', color='blue', label='Pin')
                    o_i += 1
                else:
                    plt.plot(pins[i][0], pins[i][1], 'o', color='blue')
        plt.xlim([-1, grid_size[0] + 1])
        plt.ylim([-1, grid_size[1] + 1])
        plt.title('layer{}'.format(layer))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        # plt.show()
        plt.savefig('OriginVisualize_benchmark2d_layer{layer}_{dumpBench}.png'
                    .format(layer=layer, dumpBench=benchmark_i + 1))
        plt.close()


def draw_grid_plot(grid_env, benchmark_i=0):
    best_route = grid_env.route_combo
    grid_parameter = grid_env.grid_parameter
    grid_size = grid_parameter['gridSize']
    layer_num = grid_size[2]
    start = []
    end = []

    for layer in range(layer_num):
        plt.figure()
        o_i = 0
        v_i = 0
        for route in best_route:
            start.append(route[0])
            end.append(route[-1])
            for i in range(len(route) - 1):
                pair_x = [route[i][0], route[i + 1][0]]
                pair_y = [route[i][1], route[i + 1][1]]
                pair_z = [route[i][2], route[i + 1][2]]
                if pair_z[0] == pair_z[1] and pair_z[0] == layer:
                    # same layer
                    plt.plot(pair_x, pair_y, color='blue', linewidth=2.5)
                elif pair_z[0] != pair_z[1]:
                    # exchange layer
                    if v_i == 0:
                        plt.plot(pair_x[0], pair_y[0], 'v', color='red', label='Via')
                        v_i += 1
                    else:
                        plt.plot(pair_x[0], pair_y[0], 'v', color='red')
        # draw the start & end
        for i in range(len(start)):
            if start[i][2] == layer:
                if o_i == 0:
                    plt.plot(start[i][0], start[i][1], 'o', color='blue', label='Pin')
                    o_i += 1
                else:
                    plt.plot(start[i][0], start[i][1], 'o', color='blue')
            if end[i][2] == layer:
                if o_i == 0:
                    plt.plot(end[i][0], end[i][1], 'o', color='blue', label='Pin')
                    o_i += 1
                else:
                    plt.plot(end[i][0], end[i][1], 'o', color='blue')
        plt.xlim([-1, grid_size[0] + 1])
        plt.ylim([-1, grid_size[1] + 1])
        plt.title('layer{}'.format(layer))
        plt.xlabel('X')
        plt.ylabel('Y')
        # plt.scatter(marker="o", color='b', label="x**2")
        # plt.scatter(marker="v", color='r', label="2*x+1")
        plt.legend()
        # plt.show()
        plt.savefig('AStarRoutingVisualize_benchmark2d_layer{layer}_{dumpBench}.png'
                    .format(layer=layer, dumpBench=benchmark_i + 1))
        plt.close()
