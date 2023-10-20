There are currently two components to this package:

- The YOLO "server" which runs inference and uses IPC (Python `multiprocessing`) to communicate with a rospy node.
- The actual service node which uses IPC to communicate with the "server".

This is a temporary solution to workaround the minimum Python requirements for the `ultralytics` Python package which wraps around YOLOv8 and provides an interface for running interface and collecting results.

The actual YOLO detection routine works as follows:

- Decode the image using Pillow and numpy

  Pillow is used to decode the image and numpy is used to flip BGR to RGB if necessary.

  The following encodings are currently supported:

  - bgr8
  - 8UC3
  - rgb8

  > [!NOTE]  
  > This could be turned into a utility library.

- Load the appropriate YOLO model

  Models are loaded from the `models` folder. Standard v8 models are loaded on-demand and saved to the directory as well.

  > [!IMPORTANT]  
  > If you would like to train your own model, [a full guide is available here](https://github.com/insertish/yolov8_training_workspace).

  One or more models may be loaded at the same time, they are stored in an in-memory cache.

  > [!WARNING]  
  > Take care to keep track of how many different models you are loading.

- Run inference

  This is entirely handled by the Python package.

- Construct response

  Tensors containing masks are converted to Python arrays to be sent back to the client requesting the detection.

  > [!IMPORTANT]  
  > Tensors may not be stored in CPU memory, so they may have to be moved first using `.cpu()`.
