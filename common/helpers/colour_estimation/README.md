# colour_estimation

Python utilities for estimating the name of given colours.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)

Ensure numpy is available wherever this package is imported.

## Usage

Find the closest colours to a given colour:

```python
import numpy as np
from colour_estimation import closest_colours, RGB_COLOURS, RGB_HAIR_COLOURS

# find the closest colour from RGB_COLOURS dict
closest_colours(np.array([255, 0, 0]), RGB_COLOURS)

# find the closest colour from RGB_HAIR_COLOURS dict
closest_colours(np.array([200, 150, 0]), RGB_HAIR_COLOURS)
```

## Example

Find the name of the median colour in an image:

```python
import numpy as np
from colour_estimation import closest_colours, RGB_COLOURS

# let `img` be a cv2 image / numpy array

closest_colours(np.median(img, axis=0), RGB_COLOURS)
```

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
