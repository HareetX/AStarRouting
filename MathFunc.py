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
