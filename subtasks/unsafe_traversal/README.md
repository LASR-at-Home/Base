# Unsafe Traversal

Run the node:

```bash
rosrun unsafe_traversal unsafe_traversal
```

> ⚠️ Any interactions with this node should be done in a "stop the world" fashion whereby all other processes which have the potential to move the robot must cease in order to minimise the potential of a crash.

> ❗ This service does not currently tuck the arm.

## Basic Service Usage

Switch to unsafe mode:

```bash
rosservice call /unsafe_traversal/set_unsafe_traversal true
```

Switch back:

```bash
rosservice call /unsafe_traversal/set_unsafe_traversal false
```
