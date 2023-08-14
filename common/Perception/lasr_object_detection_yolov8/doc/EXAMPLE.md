1. Find a video to test on, or otherwise acquire an image topic.

   My test video is `https://www.youtube.com/watch?v=ng8Wivt52K0`, [download it using Cobalt](https://co.wukko.me/) then place it in a directory such as `~/test_video.mp4`.

2. Start ROS master if not started already.

   ```bash
   roscore &
   ```

3. Start a video stream using the example video. (skip if using another topic / source)

   ```bash
   roslaunch video_stream_opencv camera.launch video_stream_provider:=$HOME/test_video.mp4 loop_videofile:=true visualize:=true
   ```

4. Install the YOLO server if not already, see "Installing YOLOv8 Server" section.

5. Start the YOLO service.

   ```bash
   DEBUG=1 rosrun lasr_object_detection_yolov8 service
   ```

6. Launch image view to preview the debug output.

   ```bash
   rqt_image_view
   ```

7. Start the relay script to start processing images.

   ```bash
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw

   # pick a different model:
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw yolov8n-seg.pt
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw yolov8l.pt

   # example: re-create the mask on the client end:
   rosrun lasr_object_detection_yolov8 construct_mask /camera/image_raw
   ```
