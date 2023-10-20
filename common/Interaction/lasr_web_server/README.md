# lasr_web_server

The lasr_web_server package

This package is maintained by:
- [jared](mailto:jared@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)

- [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)
- [web_video_server](http://wiki.ros.org/web_video_server)

## Usage

The web server will be typically run alongside the entire Interaction stack (see the Interaction README).
However, if you wish to run in isolation, run the following:

`roslaunch lasr_web_server web_server.launch`

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

The following topics are maintained by the web server:

| Topic                           | Message Type    | Functionality                                                          |
| ------------------------------- | --------------- | ---------------------------------------------------------------------- |
| /lasr_web_server/set_input_text | std_msgs/String | Publish to this topic to set the placeholder text of the input box.    |
| /lasr_web_server/text_input     | std_msgs/String | Publishes submitted user input. Subscribe to this topic to receive it. |

The web server is built using the following technologies:

- [roslibjs](http://wiki.ros.org/roslibjs)
- [rosbridge](http://wiki.ros.org/rosbridge_suite)
- websocket
- [Flask](https://flask.palletsprojects.com/en/2.2.x/)
- Bootstrap, HTML, CSS, JavaScript

## ROS Definitions

### Launch Files

#### `web_server`

No description provided.


### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
