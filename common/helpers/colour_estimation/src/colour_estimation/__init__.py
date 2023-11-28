from .rgb import RGB_COLOURS, RGB_HAIR_COLOURS

import numpy as np


def closest_colours(requested_colour, colours):
    '''
    Find the closest colours to the requested colour

    This returns the closest three matches
    '''

    distances = {color: np.linalg.norm(
        np.array(rgb_val) - requested_colour) for color, rgb_val in colours.items()}
    sorted_colors = sorted(distances.items(), key=lambda x: x[1])
    top_three_colors = sorted_colors[:3]
    formatted_colors = [(color_name, distance)
                        for color_name, distance in top_three_colors]

    return formatted_colors
