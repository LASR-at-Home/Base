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
