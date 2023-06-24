import os

import numpy as np

from matplotlib import pyplot as plt

import matplotlib
matplotlib.use('TkAgg')

result_dir = 'result_plot'


def draw_cost_plot(cost_list, benchmark_i=0):
    n = np.linspace(1, len(cost_list), len(cost_list))
    plt.figure()
    plt.plot(n, cost_list)
    plt.xlabel('episodes')
    plt.ylabel('cost')

    if not os.path.exists(result_dir):
        os.mkdir(result_dir)

    plt.savefig('{result_dir}/benchmark_{dumpBench}.AStarCostPlot.jpg'.format(result_dir=result_dir,
                                                                              dumpBench=benchmark_i + 1))
    plt.close()


def draw_origin_grid_plot(grid_env, benchmark_i=0):
    grid_size = grid_env.grid_size
    # grid_size = grid_parameter['gridSize']
    layer_num = grid_size[2]
    pins = []

    for net in grid_env.netlist:
        for pos in net:
            pins.append(pos)

    # for i in range(grid_parameter['numNet']):
    #     net_info = grid_parameter['netInfo'][i]
    #     for j in range(net_info['numPins']):
    #         pins.append(net_info[str(j + 1)])

    plt.figure(figsize=(6*(grid_size[0]/grid_size[1]) * layer_num + layer_num - 1, 6))
    for layer in range(layer_num):
        plt.subplot(1, layer_num, layer + 1)
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

    if not os.path.exists(result_dir):
        os.mkdir(result_dir)

    plt.savefig('{result_dir}/OriginVisualize_benchmark2d_{dumpBench}.png'.format(result_dir=result_dir,
                                                                                  dumpBench=benchmark_i + 1))
    plt.close()


def draw_grid_plot(grid_env, benchmark_i=0):
    best_route = []
    for i in range(len(grid_env.route_combo)):
        for j in range(len(grid_env.route_combo[i])):
            best_route.append(grid_env.route_combo[i][j])
    # best_route = grid_env.route_combo
    # grid_parameter = grid_env.grid_parameter
    # grid_size = grid_parameter['gridSize']
    grid_size = grid_env.grid_size
    layer_num = grid_size[2]
    start = []
    end = []

    plt.figure(figsize=(6*(grid_size[0]/grid_size[1]) * layer_num + layer_num - 1, 6))
    for layer in range(layer_num):
        plt.subplot(1, layer_num, layer + 1)
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
                elif pair_z[0] != pair_z[1] and (pair_z[0] == layer or pair_z[1] == layer):
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
        plt.legend()
    # plt.show()

    if not os.path.exists(result_dir):
        os.mkdir(result_dir)

    plt.savefig('{result_dir}/AStarRoutingVisualize_benchmark2d_{dumpBench}.png'.format(result_dir=result_dir,
                                                                                        dumpBench=benchmark_i + 1))
    plt.close()


def draw_line_chat_plot(data_combo, label_combo, num, y_label):
    plt.figure()
    for i in range(num):
        n = np.linspace(1, len(data_combo[i]), len(data_combo[i]))
        plt.plot(n, data_combo[i], label=label_combo[i])
    plt.xlabel('episodes')
    plt.ylabel(y_label)
    plt.legend()

    if not os.path.exists(result_dir):
        os.mkdir(result_dir)

    plt.savefig('{result_dir}/{label}Plot.jpg'.format(result_dir=result_dir, label=y_label))
    plt.close()


def draw_scatter_plot(data_combo, label_combo, num, y_label):
    plt.figure()
    for i in range(num):
        n = np.linspace(1, len(data_combo[i]), len(data_combo[i]))
        plt.scatter(n, data_combo[i], label=label_combo[i])
    plt.xlabel('benchmark')
    plt.ylabel(y_label)
    plt.legend()

    if not os.path.exists(result_dir):
        os.mkdir(result_dir)

    plt.savefig('{result_dir}/{label}Plot.jpg'.format(result_dir=result_dir, label=y_label))
    plt.close()
