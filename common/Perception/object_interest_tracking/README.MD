## Under Development
#
This repo aims to provide a solution to who the robot should engage with in a human-like social environment in ROS. The package currently works only in Noetic. Due to mediapipe requirements of python > 3.8, the package won't run in melodic.

The current process takes an image frame, then detect people on the frame, then separate them to apply the scoring pipeline. Currently, the service will output the frame of the person [xywh] to engage with. It will always output a person, even if the person has a low engagement score.

# Services
## engagementScore
Input: Nothing

Output: A list that has the frame part of the person to engage with [xywh]

### Scoring

The current scoring takes into account emotions, head position, and availability of distractions. It will be extended to include more criteria.


# Future updates (todo list)

Add distance and the change in distance to the scoring pipeline

Add gesture detection

# References

A. M. Al-Nuimi and G. J. Mohammed, "Face Direction Estimation based on Mediapipe Landmarks," 2021 7th International Conference on Contemporary Information Technology and Mathematics (ICCITM), Mosul, Iraq, 2021, pp. 185-190, doi: 10.1109/ICCITM53167.2021.9677878.


#
## Tested only in simulation, yet to be tested in a robot
