1. Setting up the points: 
    We need to set up all the points corresponding to all of the main areas in the map correctly. 

    The main points we need to know are: 
        - The four corners of the area we expect the guests to wait (where the door is)
        - Where the robot should wait for the guest 


    If running in the development room, the points should already be correctly in the receptionist_demo.yaml file. It is worth checking that these points are correct. 

    To pick our own points, we pick the points from RVIZ. 


2. Setting up the microphone: 
        - Currently, microphone may be set to "0" (laptops defautl microphone), "5" (Zoe's laptop+gomic) or "13" (development laptop+gomic) in the setup.launch file. 
        - Double check microphone index is the correct one for the microphone you want to use by doing: 
            \`\`\`bash    
            rosrun lasr_speech_recognition_whisper list_microphones.py
            \`\`\`
   


       

3. How to Run:
    To run the receptionist task, we need to run two launch files: 

    ```md
   1. Firstly, set up RASA, YOLO and other models/dependencies: 

      \`\`\`bash    
      roslaunch receptionist setup.launch
      \`\`\`
   
   2. When the setup has finished, launch the demo: 

      \`\`\`bash
      roslaunch receptionist demo.launch
      \`\`\`
   ```

    Note: Setup may take some time. Give it 2 or 3 minutes to do this, and it's finished when it's outputs "RASA is ready!". 


4. Common Issues: 
    1. RASA doesn't detect speech/breaks/doesn't work: 
        It may be that there isn't a RASA model for the receptionist task. 
        Please see common/speech/lasr_rasa/doc to train the RASA model for the receptionist. The required data should be in common/speech/lasr_rasa/assistants/lift/data (note: Currently, due to some issues with RASA dependencies, all models for receptionist task and all data to train receptionist task models are in the "assistants/lift" folder in RASA)
    2. The microphone isn't working when attempting to do speech/whisper: 
        This may be a problem with the laptops microphone drivers. Please make sure no other application (including the settings UI) is using the microphone while running the episode. 
        Please make sure that the microphone set in the setup.launch file has the same index as the microphone you want to use. See common/speech/lasr_speech_recognition_whisper for general whisper and microphone usage. 