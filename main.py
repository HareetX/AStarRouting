import os
import shutil
import time
import argparse

from BenchmarkGenerator import generator


def parse_arguments():
    parser = argparse.ArgumentParser('Benchmark Generator Parser')
    parser.add_argument('--benchNumber', type=int, dest='benchmarkNumber', default=10)
    parser.add_argument('--gridSize', type=int, dest='gridSize', default=4)
    parser.add_argument('--layerNum', type=int, dest='layerNum', default=1)
    parser.add_argument('--netNum', type=int, dest='netNum', default=3)
    parser.add_argument('--maxPinNum', type=int, dest='maxPinNum', default=2)

    return parser.parse_args()


if __name__ == '__main__':
    start_time = time.time()  # Record starting time

    benchmark_dir = 'benchmark'
    solution_dqn_dir = 'solutionsAStar'
    if os.path.exists(benchmark_dir):
        shutil.rmtree(benchmark_dir)
    if os.path.exists(solution_dqn_dir):
        shutil.rmtree(solution_dqn_dir)

    print('**************')
    print('Problem Generating Module')
    print('**************')

    os.mkdir(benchmark_dir)

    args = parse_arguments()

    # Generate Benchmark
    for i in range(args.benchmarkNumber):
        filename = benchmark_dir + '/benchmark{}.gr'.format(i+1)
        generator(filename, args.gridSize, args.layerNum, args.netNum, args.maxPinNum)

    end_time = time.time()  # Record ending time
    print("time = {} s".format(end_time - start_time))

