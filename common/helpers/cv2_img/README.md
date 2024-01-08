# cv2_img

Helper methods for converting between sensor_msgs and cv2/Pillow format.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- sensor_msgs

Ensure that the following is readily available wherever you import this package:

- numpy
- Pillow

Despite the name of the package, cv2 is not actually a dependency.

## Usage

Convert image messages to cv2 or Pillow format:

```python
import cv2_img

# as Pillow Image
img = cv2_img.msg_to_pillow_img(request.image_raw)

# as numpy array (for cv2)
img = cv2_img.msg_to_cv2_img(request.image_raw)
```

Convert a cv2 image (numpy array) to an image message:

```python
import cv2_img

msg = cv2_img.cv2_img_to_msg(img)
```

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

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
