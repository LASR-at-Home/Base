1. Find a video to test on, or otherwise acquire an image topic.

   My test video is `https://www.youtube.com/watch?v=ng8Wivt52K0`, [download it using Cobalt](https://co.wukko.me/) then place it in a directory such as `~/v.mp4`.

2. Then launch the demo:

   ```bash
   roslaunch lasr_vision_yolov8 demo.launch file:=$HOME/v.mp4

   # .. you can also try other models:
   roslaunch lasr_vision_yolov8 demo.launch model:=yolov8n.pt file:=$HOME/v.mp4
   ```
