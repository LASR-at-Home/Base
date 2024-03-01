Find the name of the median colour in an image:

```python
import numpy as np
from colour_estimation import closest_colours, RGB_COLOURS

# let `img` be a cv2 image / numpy array

closest_colours(np.median(img, axis=0), RGB_COLOURS)
```
