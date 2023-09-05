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
