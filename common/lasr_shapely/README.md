# lasr_shapely

Abstraction of some algorithms provided by shapely as services. Intended for Python 3.6 or older environments.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)
- message_generation (build)
- message_runtime (exec)

This packages requires Python 3.10 to be present.

This package has 2 Python dependencies:
- [shapely](https://pypi.org/project/shapely)None
- .. and 1 sub dependencies



## Usage

Start the service:

```bash
rosrun lasr_shapely service
```

Then consume the Python library:

```python
from lasr_shapely import LasrShapely

shapely = LasrShapely()

shapely.is_point_in_polygon_2d([[0, 0], [5,0], [10, 10], [0, 5]], 6, 6)
shapely.are_points_in_polygon_2d([[0, 0], [5,0], [10, 10], [0, 5]], [[1,2], [11, 11]])
```

## Example

See usage.

## Technical Overview

This package is just a service wrapper around a Python 3.7+ library.

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

This package has no messages.

### Services

#### `PointsInPolygon2D`

Request

| Field | Type | Description |
|:-:|:-:|---|
| polygon | float32[] | 1D-array of ordered pairs of points (connected by edge) that define the polygon |
| points | float32[] | 1D-array of pairs of points |

Response

| Field | Type | Description |
|:-:|:-:|---|
| inside | bool[] | Whether the points are inside the polygon |

#### `PointInPolygon2D`

Request

| Field | Type | Description |
|:-:|:-:|---|
| polygon | float32[] | 1D-array of ordered pairs of points (connected by edge) that define the polygon |
| x | float32 | X coordinate of point |
| y | float32 | Y coordinate of point |

Response

| Field | Type | Description |
|:-:|:-:|---|
| inside | bool | Whether the point is inside the polygon |


### Actions

This package has no actions.
