"""
spawn_state.py — YASMIN state that spawns the basket model in Gazebo
in front of the robot, then continues.

Spawns ONLY the basket. Pose is relative to reference_frame (default
base_footprint), so x is straight ahead of the robot — no world-frame math.
Re-runs delete the old entity first so repeated runs stay clean.

Gazebo Classic (gazebo_msgs/SpawnEntity, /spawn_entity service).
For Ignition/gz this needs the ros_gz create service instead.

Usage in your StateMachine:
    from spawn_state import Spawn
    self.add_state("SPAWN_BASKET", Spawn(model_path="/abs/path/basket.sdf"),
                   transitions={"succeeded": "DETECT_BASKET", "failed": "failed"})
"""

import math
import time

from geometry_msgs.msg import Pose, Point, Quaternion

try:
    from yasmin import State
except ImportError:
    class State:                             # fallback shim if yasmin is absent
        def __init__(self, outcomes):
            self._outcomes = outcomes


class Spawn(State):
    def __init__(self, model_path, name='basket', x=0.3, y=0.0, z=0.0, yaw=0.0,
                 reference_frame='base_footprint', delay=0.0, settle=0.0):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.model_path = model_path
        self.name = name
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.reference_frame = reference_frame
        self.delay = delay        # seconds to wait BEFORE spawning
        self.settle = settle      # seconds to wait AFTER spawning (let it settle)
        from yasmin_ros.yasmin_node import YasminNode
        from gazebo_msgs.srv import SpawnEntity, DeleteEntity
        self._SpawnEntity = SpawnEntity
        self._DeleteEntity = DeleteEntity
        self.node = YasminNode.get_instance()
        self.cli = self.node.create_client(SpawnEntity, '/spawn_entity')
        self.del_cli = self.node.create_client(DeleteEntity, '/delete_entity')

    def _wait(self, future, timeout=10.0):
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def execute(self, blackboard):
        if self.delay > 0.0:
            time.sleep(self.delay)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('[Spawn] /spawn_entity not available')
            return 'failed'
        try:
            with open(self.model_path) as f:
                xml = f.read()
        except Exception as e:
            self.node.get_logger().error(f'[Spawn] cannot read {self.model_path}: {e}')
            return 'failed'

        # best-effort delete so re-runs are clean (ignore result)
        if self.del_cli.wait_for_service(timeout_sec=1.0):
            dreq = self._DeleteEntity.Request()
            dreq.name = self.name
            self._wait(self.del_cli.call_async(dreq), timeout=3.0)

        req = self._SpawnEntity.Request()
        req.name = self.name
        req.xml = xml
        req.reference_frame = self.reference_frame
        req.initial_pose = Pose(
            position=Point(x=float(self.x), y=float(self.y), z=float(self.z)),
            orientation=Quaternion(x=0.0, y=0.0,
                                   z=math.sin(self.yaw / 2), w=math.cos(self.yaw / 2)))
        res = self._wait(self.cli.call_async(req))
        if res and res.success:
            if self.settle > 0.0:
                time.sleep(self.settle)
            self.node.get_logger().info(
                f'[Spawn] basket spawned at x={self.x} ({self.reference_frame})')
            return 'succeeded'
        self.node.get_logger().error(
            f'[Spawn] spawn failed: {getattr(res, "status_message", "no response")}')
        return 'failed'