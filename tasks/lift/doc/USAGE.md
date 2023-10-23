1. Setting up the points: 
    We need to set up all the points corresponding to all of the main areas in the map correctly. 

    The main points we need to know are: 
        - The four corners inside the lift (Should be a 2x2m square in the real world)
        - The center of inside of the lift
        - The four corners of the "lift waiting area" (should be a 2x2m square outside, but next to, the lift)
        - The center of the lift waiting area
        - The starting position 
        - The position we wait to view the lift waiting area 

    If running in the development room (given that a lift is set up correctly) the points should already be correctly in the demo.yaml file. It is worth checking that these points are correct. 

    To pick our own points, we pick the points from RVIZ. 

    All required points will be in the final_lift.yaml file. 
       

2. How to Run:
    To run the lift task, we need to run two launch files: 

    ```md
   1. Firstly, set up RASA, YOLO and other models/dependencies: 

      \`\`\`bash    
      rosrun lift setup.launch
      \`\`\`
   
   2. When the setup has finished, launch the demo: 

      \`\`\`bash
      rosrun lift demo.launch
      \`\`\`
   ```

    Note: Setup may take some time. Give it 2 or 3 minutes to do this, and it's finished when it's only outputing "Door status:" and nothing else. 


3. Common Issues: 
    1. RASA doesn't detect speech/breaks/doesn't work: 
        It may be that there isn't a RASA model for the lift task. 
        Please see common/speech/lasr_rasa/doc to train the RASA model for the lift. The required data should be in common/speech/lasr_rasa/assistants/lift/data
    2. There is no model for the keypoints: 
        The model is too big to be uploaded to git, please contact Zoe for the "lift_position" keypoint model. 
        Coming soon: Ideally there will be some shared folder with important models that are too big for git. Will share a link soon. 
    3. The microphone isn't working when attempting to do speech/whisper: 
        This may be a problem with the laptops microphone drivers. Please make sure no other application (including the settings UI) is using the microphone while running the episode. 
        Please make sure that the microphone set in the setup.launch file has the same index as the microphone you want to use. See common/speech/lasr_speech_recognition_whisper for general whisper and microphone usage. 