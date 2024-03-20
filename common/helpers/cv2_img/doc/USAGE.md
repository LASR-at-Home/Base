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
