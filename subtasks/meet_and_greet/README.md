# Meet and Great Package 

Some of teh files are currently under implementation
**@author: Nicole Lehchevska** -> refer to me if you have questions or ideas

> The Meet and greet package files:
1. Explore suroundings state -> robot wondering around (tbc) 
2. Look around state -> robot moving the head and searchign for people, to output known and unknown people (tbc) 
3. Meet and greet sm -> teh state machine implementing preemtive states (tbc) 
4. SImple meet and greet -> the demo file to represent the recognition


> How to run 
```python 
roslaunch lasr_perception_server perception_server.launch 
rosrun meet_and_greet simple_meet_and_greet.py 

```

Make sure you have either:
```python 
roslaunch custom_worlds wrs_receptionist.launch 
roslaunch usb_cam usb_cam-test.launch 
```