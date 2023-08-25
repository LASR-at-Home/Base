1. Create a Rasa assistant.
    
    ```bash
    roscd lasr_rasa
    mkdir -p assistants/my_new_assistant
    cd assistants/my_new_assistant
    rosrun lasr_rasa rasa init
    ```

2. Create a NLU dataset, by defining training examples in `/lasr_rasa/assistants/my_new_assistant/data/nlu.yml`, for more information see the [Rasa docs](https://rasa.com/docs/rasa/nlu-training-data/).

3. Train the NLU assistant.
    ```bash
    rosrun lasr_rasa rasa train nlu
    ```
    This will produce a `.tar.gz` file in `lasr_rasa/assistants/my_new_assistant/models`.
    
4. Run Rasa as an API with our newly trained assistant.
    ```bash
    rosrun lasr_rasa rasa run --enable-api
    ```
    The API is run on `localhost:5005` by default, but you can change the port using the `-p` argument.
    The endpoint that we are interested in is `/model/parse`.

5. Test the API is up and running.
    ```bash
    curl 'localhost:5005/model/parse' -d '{"text" : "Hello"}'
    ```

6. Start ROS master if not started already.

   ```bash
   roscore &
   ```
   
7. Run the service
    ```bash
    rosrun lasr_rasa service [PORT]
    ```

8. Make a call to the service.
    ```bash
    rosservice call /lasr_rasa/parse "text: 'hello'"
    ```