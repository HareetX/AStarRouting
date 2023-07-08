import math

import numpy as np


def diagonal_distance_3d(start, end):
    x_distance = abs(start[0] - end[0])
    y_distance = abs(start[1] - end[1])
    z_distance = abs(start[2] - end[2])

    if x_distance > y_distance:
        max_distance = x_distance
        min_distance = y_distance
    else:
        max_distance = y_distance
        min_distance = x_distance

    return max_distance - min_distance + 1.414 * min_distance + z_distance * 10


def diagonal_distance_2d(start, end):
    x_distance = abs(start[0] - end[0])
    y_distance = abs(start[1] - end[1])

    if x_distance > y_distance:
        max_distance = x_distance
        min_distance = y_distance
    else:
        max_distance = y_distance
        min_distance = x_distance

    return max_distance - min_distance + 1.414 * min_distance


# def is_include_segment(segment_ab, segment_cd):
#     is_inside = [False, False]
#     ac = np.subtract(segment_cd[0], segment_ab[0])
#     bc = np.subtract(segment_cd[0], segment_ab[1])
#     ad = np.subtract(segment_cd[1], segment_ab[0])
#     bd = np.subtract(segment_cd[1], segment_ab[1])
#     is_collinear = not np.cross(ac, bc).all() and not np.cross(ad, bd).all()
#     is_inside[0] = np.dot(ac, bc) < 0
#     is_inside[1] = np.dot(ad, bd) < 0
#     if is_collinear:
#         if is_inside[0] or is_inside[1]:
#             return False
#         else:
#             return True
#     else:
#         return False


def read(filename):
    file = open(filename, 'r')
    file_info = {}
    i = 0
    for line in file:
        if not line.strip():
            continue
        else:
            file_info[i] = line.split()
        i += 1
    file.close()
    return file_info


def circle_space(pos, radius) -> list:
    pos_list = [pos]
    for x_i in range(int(radius) + 1):
        for y_i in range(int(radius) + 1):
            if x_i == 0 and y_i == 0:
                continue
            if x_i ** 2 + y_i ** 2 <= radius ** 2:
                if x_i == 0:
                    pos_list.append([pos[0], pos[1] + y_i, pos[2]])
                    pos_list.append([pos[0], pos[1] - y_i, pos[2]])
                elif y_i == 0:
                    pos_list.append([pos[0] + x_i, pos[1], pos[2]])
                    pos_list.append([pos[0] - x_i, pos[1], pos[2]])
                else:
                    pos_list.append([pos[0] + x_i, pos[1] + y_i, pos[2]])
                    pos_list.append([pos[0] - x_i, pos[1] + y_i, pos[2]])
                    pos_list.append([pos[0] - x_i, pos[1] - y_i, pos[2]])
                    pos_list.append([pos[0] + x_i, pos[1] - y_i, pos[2]])

    return pos_list


def circle_space_set(pos, radius) -> set:
    pos_set = {str(pos)}
    for x_i in range(int(radius) + 1):
        for y_i in range(int(radius) + 1):
            if x_i == 0 and y_i == 0:
                continue
            if x_i ** 2 + y_i ** 2 <= radius ** 2:
                if x_i == 0:
                    pos_set.add(str([pos[0], pos[1] + y_i, pos[2]]))
                    pos_set.add(str([pos[0], pos[1] - y_i, pos[2]]))
                elif y_i == 0:
                    pos_set.add(str([pos[0] + x_i, pos[1], pos[2]]))
                    pos_set.add(str([pos[0] - x_i, pos[1], pos[2]]))
                else:
                    pos_set.add(str([pos[0] + x_i, pos[1] + y_i, pos[2]]))
                    pos_set.add(str([pos[0] - x_i, pos[1] + y_i, pos[2]]))
                    pos_set.add(str([pos[0] - x_i, pos[1] - y_i, pos[2]]))
                    pos_set.add(str([pos[0] + x_i, pos[1] - y_i, pos[2]]))

    return pos_set


def rect_space(pos, a, b) -> list:
    x, y, z = pos
    a = int(a)
    b = int(b)
    pos_list = []
    for x_i in range(2 * a + 1):
        for y_i in range(2 * b + 1):
            pos_list.append([x - a + x_i, y - b + y_i, z])

    return pos_list


def rect_space_set(pos, a, b) -> set:
    x, y, z = pos
    a = int(a)
    b = int(b)
    pos_set = set([])
    for x_i in range(2 * a + 1):
        for y_i in range(2 * b + 1):
            pos_set.add(str([x - a + x_i, y - b + y_i, z]))

    return pos_set


def via_cost_function_1(x, critical):
    if x > critical or x < -critical:
        return 0
    else:
        return 1000


def via_cost_function_2(x, alpha):
    if x == 0:
        return 1000
    return alpha * 1 / x


def via_cost_function_3(x, critical):
    if x > critical or x < -critical:
        return 1000 * critical**4 / abs(x**4)
    else:
        return 1000


if __name__ == '__main__':
    segment1 = [[1, 1, 1], [3, 3, 3]]
    segment2 = [[1, 1, 1], [3, 3, 3]]  # dont include
    segment3 = [[1, 1, 1], [4, 4, 4]]  # rewrite segment
    segment4 = [[1, 1, 1], [2, 2, 2]]  # dont include

    print(circle_space([0, 0, 0], 1.42))
    print(rect_space([0, 0, 0], 1.42))
