import random
# import numpy


def random_locations(random_seed):
    random.seed(random_seed)
    _locations = []
    for i in range(25):
        if i <= 8:
            _locations.append((round(random.uniform(1, 3), 2)*5280, round(random.uniform(1, 3), 2)*5280))
        if 8 < i <= 16:
            _locations.append((round(random.uniform(3, 5), 2)*5280, round(random.uniform(2, 4), 2)*5280))
        if 16 < i <= 24:
            _locations.append((round(random.uniform(1, 3), 2)*5280, round(random.uniform(3, 5), 2)*5280))
        if i == 25:
            _locations.append((round(random.uniform(0, 5), 2)*5280, round(random.uniform(0, 5), 2)*5280))
    # print(_locations)
    # print(len(_locations))
    return _locations
