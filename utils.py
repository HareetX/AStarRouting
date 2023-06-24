import numpy as np


def diagonal_distance(start, end):
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


def is_include_segment(segment_ab, segment_cd):
    is_inside = [False, False]
    ac = np.subtract(segment_cd[0], segment_ab[0])
    bc = np.subtract(segment_cd[0], segment_ab[1])
    ad = np.subtract(segment_cd[1], segment_ab[0])
    bd = np.subtract(segment_cd[1], segment_ab[1])
    is_collinear = not np.cross(ac, bc).all() and not np.cross(ad, bd).all()
    is_inside[0] = np.dot(ac, bc) < 0
    is_inside[1] = np.dot(ad, bd) < 0
    if is_collinear:
        if is_inside[0] or is_inside[1]:
            return False
        else:
            return True
    else:
        return False


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


if __name__ == '__main__':
    segment1 = [[1, 1, 1], [3, 3, 3]]
    segment2 = [[1, 1, 1], [3, 3, 3]]  # dont include
    segment3 = [[1, 1, 1], [4, 4, 4]]  # rewrite segment
    segment4 = [[1, 1, 1], [2, 2, 2]]  # dont include
