from geometry_msgs.msg import (
    Point,
    Pose,
)

import numpy as np
from itertools import permutations

from typing import Union, List


def euclidian_distance(p1: Point, p2: Point) -> float:
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


def min_hamiltonian_path(start: Pose, poses: List[Pose]) -> Union[None, List[Pose]]:
    best_order = None
    min_distance = np.inf

    for perm in permutations(poses):
        dist = euclidian_distance(start.position, perm[0].position)
        for i in range(len(poses) - 1):
            dist += euclidian_distance(perm[i].position, perm[i + 1].position)

        if dist < min_distance:
            min_distance = dist
            best_order = list(perm)

    return best_order
