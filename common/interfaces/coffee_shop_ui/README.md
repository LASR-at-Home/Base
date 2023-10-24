# coffee_shop_ui

Provides a tablet user interface for ordering and confirming actions within the coffee shop task.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- message_generation (build)
- message_runtime (exec)

This package currently requires a rosbridge WebSocket server to run on port 9090 on the same machine running this node.

## Usage

Start a rosbridge server using:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

Build and run the coffee shop UI:

```bash
rosrun coffee_shop_ui build # don't need to run again except to update
rosrun coffee_shop_ui start

# .. or run a dev server:
rosrun coffee_shop_ui dev
```

You can now access the web server on $ROS_IP:3000.

You can now publish `std_msgs/String` to `/tablet/screen` with the string being one of:

- `home`
- `order`

  A `coffee_shop_ui/Order` message will be published to `/tablet/order` when the user confirms their order.

  The message contains a list of items in any order, items names are repeated for how many were ordered.

- `done`

  A `std_msgs/Empty` message will be published to `/tablet/done` when the user presses the button.

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

#### `Confirm`

| Field | Type | Description |
|:-:|:-:|---|
| value | bool | confirmation |

#### `Order`

| Field | Type | Description |
|:-:|:-:|---|
| products | string[] | list of products being ordered |


### Services

This package has no services.

### Actions

This package has no actions.
