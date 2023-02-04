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

Or use the helper script:

```bash
rosrun unsafe_traversal test_service
```

## Action Usage

This node provides two actions:

- `/unsafe_traversal/move_to_goal` (`unsafe_traversal.msg.MoveToGoalAction`): move to start pose aligned to the end pose then move to the end pose
- `/unsafe_traversal/align_to_goal` (`unsafe_traversal.msg.AlignToGoalAction`): align from start pose to end pose

You can test these by editing `test_move_action` or `test_align_action` in the scripts folder.
