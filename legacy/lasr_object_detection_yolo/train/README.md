## Train YOLOv4

For generating darknet config files and training YOLOv4.

# Instructions

`docker build -t darknet .`

Mount the your data to /dataset/data:

`docker run -v data:/datasets/data darknet`

To run with GPU / CUDNN / OPENCV / OPENMP:

`docker run -e USE_GPU=1 -v data:/datasets/data darknet`

# Details

**build_darknet.sh** builds darknet from source, modify this to change compilation options.

**train.sh** for training YOLOv4. Usage is ./train.sh DATASET, where DATASET is a directory name in **datasets** (you must create this root directory).

If you encounter issues with darknet, you may want to alter the **batch_size** and **subdivisions**.

Run outside of ROS environment.