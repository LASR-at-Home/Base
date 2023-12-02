Find the closest colours to a given colour:

```python
import numpy as np
from colour_estimation import closest_colours, RGB_COLOURS, RGB_HAIR_COLOURS

# find the closest colour from RGB_COLOURS dict
closest_colours(np.array([255, 0, 0]), RGB_COLOURS)

# find the closest colour from RGB_HAIR_COLOURS dict
closest_colours(np.array([200, 150, 0]), RGB_HAIR_COLOURS)
```
