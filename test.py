import os

import numpy as np

from PlotDraw import draw_line_chat_plot, draw_scatter_plot
from utils import read


def get_log_parameters(log_name, log_parameter):
    log_info = read(log_name)
    for line in log_info:
        if len(log_info[line]) == 1:
            if log_info[line][0] not in log_parameter:
                log_parameter[log_info[line][0]] = {}
        else:
            if log_info[line][0] in log_parameter:
                key = ""
                data_i = 0
                for i in range(len(log_info[line]) - 1):
                    if log_info[line][i + 1] == '=':
                        data_i = i + 2
                        break
                    key += log_info[line][i + 1] + ' '
                if '.' in log_info[line][data_i]:
                    data = float(log_info[line][data_i])
                else:
                    data = int(log_info[line][data_i])
                if key in log_parameter[log_info[line][0]]:
                    log_parameter[log_info[line][0]][key].append(data)
                else:
                    log_parameter[log_info[line][0]][key] = [data]


if __name__ == '__main__':
    log_file_dict = "log_dict.txt"
    log_file_ndarray = "log_ndarray.txt"
    log_parameters_dict = {}
    log_parameters_ndarray = {}

    for i in range(10):
        os.system("python AStarSolver.py --type={t} --episode={e} --log={l}".format(t=0, e=5, l='log_dict.txt'))
        get_log_parameters(log_file_dict, log_parameters_dict)
        os.system("python AStarSolver.py --type={t} --episode={e} --log={l}".format(t=1, e=5, l='log_ndarray.txt'))
        get_log_parameters(log_file_ndarray, log_parameters_ndarray)

    dict_time_list = None
    ndarray_time_list = None
    dict_route_time_list = None
    ndarray_route_time_list = None

    dict_route_length_list = []
    ndarray_route_length_list = []

    dict_via_amount_list = []
    ndarray_via_amount_list = []

    for i in range(len(log_parameters_dict)):
        if i == 0:
            dict_time_list = np.array(log_parameters_dict["benchmark{}".format(i + 1)]['time '])
            ndarray_time_list = np.array(log_parameters_ndarray["benchmark{}".format(i + 1)]['time '])
            dict_route_time_list = np.array(log_parameters_dict["benchmark{}".format(i + 1)]['route time '])
            ndarray_route_time_list = np.array(log_parameters_ndarray["benchmark{}".format(i + 1)]['route time '])
        else:
            dict_time_list += np.array(log_parameters_dict["benchmark{}".format(i + 1)]['time '])
            ndarray_time_list += np.array(log_parameters_ndarray["benchmark{}".format(i + 1)]['time '])
            dict_route_time_list += np.array(log_parameters_dict["benchmark{}".format(i + 1)]['route time '])
            ndarray_route_time_list += np.array(log_parameters_ndarray["benchmark{}".format(i + 1)]['route time '])

        dict_route_length_list.append(log_parameters_dict["benchmark{}".format(i + 1)]['route length '][0])
        ndarray_route_length_list.append(log_parameters_ndarray["benchmark{}".format(i + 1)]['route length '][0])

        dict_via_amount_list.append(log_parameters_dict["benchmark{}".format(i + 1)]['via amount '][0])
        ndarray_via_amount_list.append(log_parameters_ndarray["benchmark{}".format(i + 1)]['via amount '][0])

    dict_time_list /= len(log_parameters_dict)
    ndarray_time_list /= len(log_parameters_dict)
    dict_route_time_list /= len(log_parameters_dict)
    ndarray_route_time_list /= len(log_parameters_dict)

    route_length_list = np.array(dict_route_length_list) / np.array(ndarray_route_length_list)

    via_amount_list = np.array(dict_via_amount_list) / np.array(ndarray_via_amount_list)

    draw_line_chat_plot([dict_time_list, ndarray_time_list], ['dict', 'ndarray'], 2, 'total_time')
    draw_line_chat_plot([dict_route_time_list, ndarray_route_time_list], ['dict', 'ndarray'], 2, 'route_time')

    draw_scatter_plot([route_length_list], ['dict_WL/ndarray_WL'], 1, 'route_length_rate')
    draw_scatter_plot([via_amount_list], ['dict_Via/ndarray_Via'], 1, 'via_amount_rate')
