# lasr_rasa

The lasr_rasa package

This package is maintained by:
- [Jared](mailto:j.w.swift@outlook.com)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- rospy (build)
- std_msgs (build)
- rospy (exec)
- std_msgs (exec)

Install Rasa

```bash
python3 -m pip install rasa==3.6.4
```

## Usage

This package provides the `/lasr_rasa/parse` service which uses the `Rasa` service definition from `lasr_rasa`.


```python
from lasr_rasa.srv import Rasa, RasaRequest

# create service proxy
rasa_service = rospy.ServiceProxy('/lasr_rasa/parse', Rasa)

# create request
request = RasaRequest("hello")

# send request
response = rasa_service(request)
# .. use request.json_response, if request.success
```

## Example

1. Create a Rasa assistant.
    
    ```bash
    mkdir lasr_rasa/assistants/my_new_assistant
    cd lasr_rasa/assistants/my_new_assistant
    rasa init
    ```

2. Create a NLU dataset, by defining training examples in `/lasr_rasa/assistants/my_new_assistant/data/nlu.yml`, for more information see the [Rasa docs](https://rasa.com/docs/rasa/nlu-training-data/).

3. Train the NLU assistant.
    ```bash
    rasa train nlu
    ```
    This will produce a `.tar.gz` file in `lasr_rasa/assistants/my_new_assistant/models`.
    
4. Run Rasa as an API with our newly trained assistant.
    ```bash
    rasa run --enable-api
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

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Messages

This package has no messages.

### Services

#### `Rasa`

Request

| Field | Type | Description |
|:-:|:-:|---|
| text | string |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| json_response | string |  |
| success | bool |  |


### Actions

This package has no actions.
