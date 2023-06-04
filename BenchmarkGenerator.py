import argparse
import os
import shutil

import numpy as np

np.random.seed(1)


def generator(benchmark_name, grid_size, layer_num, net_num, max_pin_num):
    file = open('%s' % benchmark_name, 'w+')

    # Write general information
    file.write('grid {gridSizeX} {gridSizeY} {layerNum}\n'.format(gridSizeX=grid_size[0], gridSizeY=grid_size[1], layerNum=layer_num))
    file.write('net num {netNum}\n'.format(netNum=net_num))

    # Write nets information
    pin_num = np.random.randint(2, max_pin_num + 1, net_num)  # Generate PinNum randomly
    for j in range(net_num):
        specific_pin_num = pin_num[j]
        file.write('A{netInd} {netInd} {pin}\n'.format(netInd=j + 1, pin=specific_pin_num))
        x_array = np.random.randint(0, grid_size[0], specific_pin_num)
        y_array = np.random.randint(0, grid_size[1], specific_pin_num)
        z_array = np.random.randint(0, layer_num, specific_pin_num)
        for k in range(specific_pin_num):
            file.write('{x}  {y}  {z}\n'.format(x=x_array[k], y=y_array[k], z=z_array[k]))

    # Write obstacles information
    # TODO
    file.close()
    return


def parse_arguments():
    parser = argparse.ArgumentParser('Benchmark Generator Parser')
    parser.add_argument('--benchNumber', type=int, dest='benchmarkNumber', default=5)
    parser.add_argument('--gridSize', type=int, dest='gridSize', default=128)
    parser.add_argument('--layerNum', type=int, dest='layerNum', default=2)
    parser.add_argument('--netNum', type=int, dest='netNum', default=5)
    parser.add_argument('--maxPinNum', type=int, dest='maxPinNum', default=3)

    return parser.parse_args()


if __name__ == '__main__':
    benchmark_dir = 'benchmark'
    if os.path.exists(benchmark_dir):
        shutil.rmtree(benchmark_dir)

    os.mkdir(benchmark_dir)

    args = parse_arguments()

    for i in range(args.benchmarkNumber):
        filename = benchmark_dir + '/benchmark{}.gr'.format(i + 1)
        generator(filename, [args.gridSize, args.gridSize], args.layerNum, args.netNum, args.maxPinNum)
