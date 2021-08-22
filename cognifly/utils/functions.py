def clip(x, min_x, max_x):
    """
    clips x between min_x and max_x
    """
    return max(min(x, max_x), min_x)
