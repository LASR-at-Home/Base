# lasr_speech

The lasr_speech package

This package is maintained by:
- [Jared](mailto:j.w.swift@outlook.com)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- lasr_rasa (build)
- lasr_speech_recognition_msgs (build)
- lasr_speech_recognition_whisper (build)
- rospy (build)
- lasr_rasa (exec)
- lasr_speech_recognition_msgs (exec)
- lasr_speech_recognition_whisper (exec)
- rospy (exec)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `speech`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| matcher | by-index |  |
| device_param |  |  |
| rasa_model |  |  |



### Messages

This package has no messages.

### Services

#### `Speech`

Request

| Field | Type | Description |
|:-:|:-:|---|
| play_sound | bool |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| json_response | string |  |
| success | bool |  |


### Actions

This package has no actions.
