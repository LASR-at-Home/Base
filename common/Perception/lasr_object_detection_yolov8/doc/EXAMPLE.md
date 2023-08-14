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

5. Start the YOLO server.

   ```bash
   cd src/lasr-base/common/Perception/lasr_object_detection_yolov8/yolo_server
   source venv/bin/activate.bash
   python3 server.py
   ```

6. Start the YOLO service.

   ```bash
   DEBUG=1 rosrun lasr_object_detection_yolov8 service
   ```

7. Launch image view to preview the debug output.

   ```bash
   rqt_image_view
   ```

8. Start the relay script to start processing images.

   ```bash
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw

   # pick a different model:
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw yolov8n-seg.pt
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw yolov8l.pt

   # example: re-create the mask on the client end:
   rosrun lasr_object_detection_yolov8 construct_mask /camera/image_raw
   ```
