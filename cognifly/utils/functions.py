import numpy as np

def clip(x, min_x, max_x):
    """
    clips x between min_x and max_x
    """
    return max(min(x, max_x), min_x)


def get_angle_sequence_rad(angle):
    """
    cuts an angle bigger than pi into a sequence of angles <= pi/2
    """
    aa = abs(angle)
    if aa >= np.pi:  # equality is important because the direction is undefined
        chunks = int(aa // (np.pi / 2.0))
        last = aa % (np.pi / 2.0)
        res = [np.pi / 2.0, ] * chunks
        if last != 0:
            res.append(last)
    else:
        res = [aa, ]
    res = np.array(res) * np.sign(angle)
    return res.tolist()
