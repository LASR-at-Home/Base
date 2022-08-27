# LASR Web Server

## Running

The web server will be typically run alongside the entire Interaction stack (see the Interaction README).
However, if you wish to run in isolation, run the following:

```roslaunch lasr_web_server web_server.launch```

## Technologies

The web server is built using the following technologies:

- roslibjs
- rosbridge
- websocket
- Flask
- Bootstrap, HTML, CSS, JavaScript

## Requirements

- rosbridge_suite
- web_video_server

The following topics are maintained by the web server:

| Topic | Message Type | Functionality |
| ------ | ------ | ------ |
| /lasr_web_server/set_input_text | std_msgs/String | Publish to this topic to set the placeholder text of the input box. |
| /lasr_web_server/text_input | std_msgs/String | Publishes submitted user input. Subscribe to this topic to receive it.|
