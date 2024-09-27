from math import sqrt


def map_from_to(val, in_low, in_high, out_low, out_high):
    y = (val - in_low) / (in_high - in_low) * (out_high - out_low) + out_low
    return y


def normalize_vec(x, y, z):
    length = sqrt(x ** 2 + y ** 2 + z ** 2)
    return x / length, y / length, z / length
