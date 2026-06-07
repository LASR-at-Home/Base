#!/usr/bin/env python3
import sys
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from lasr_vision_interfaces.msg import Detection3DArray
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk, ImageDraw
import threading
import numpy as np
import time


class GraspGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LASR Grasp Control")
        self.root.geometry("1200x600")

        self.node = None
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_detections = Detection3DArray()
        self.command_pub = None
        self.head_pan = 0.0
        self.head_tilt = 0.0

        # Left frame: camera + detections
        self.left_frame = tk.Frame(root)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        tk.Label(self.left_frame, text="Camera Feed", font=("Arial", 12, "bold")).pack()
        self.image_label = tk.Label(self.left_frame, bg="gray")
        self.image_label.pack(fill=tk.BOTH, expand=True)

        # Right frame: controls
        self.right_frame = tk.Frame(root)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=5, pady=5)

        tk.Label(
            self.right_frame, text="Detected Objects", font=("Arial", 12, "bold")
        ).pack()

        # Listbox for objects
        self.object_frame = tk.Frame(self.right_frame)
        self.object_frame.pack(fill=tk.BOTH, expand=True, pady=(5, 10))

        scrollbar = tk.Scrollbar(self.object_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.object_listbox = tk.Listbox(
            self.object_frame, yscrollcommand=scrollbar.set, font=("Arial", 10)
        )
        self.object_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.object_listbox.yview)

        # Command buttons
        tk.Label(self.right_frame, text="Commands", font=("Arial", 12, "bold")).pack(
            pady=(10, 5)
        )

        tk.Label(
            self.right_frame,
            text="Detection queries (comma separated):",
            font=("Arial", 10),
        ).pack(anchor=tk.W)
        self.detect_entry = tk.Entry(self.right_frame, font=("Arial", 10))
        self.detect_entry.insert(0, "bottle,cup,object")
        self.detect_entry.pack(fill=tk.X, pady=(2, 5))

        self.button_frame = tk.Frame(self.right_frame)
        self.button_frame.pack(fill=tk.X, pady=5)

        tk.Button(
            self.button_frame,
            text="Detect",
            font=("Arial", 11),
            command=self.cmd_detect,
            bg="#9C27B0",
            fg="white",
        ).pack(fill=tk.X, pady=2)
        tk.Button(
            self.button_frame,
            text="Home",
            font=("Arial", 11),
            command=self.cmd_home,
            bg="#4CAF50",
            fg="white",
        ).pack(fill=tk.X, pady=2)
        tk.Button(
            self.button_frame,
            text="Init Grasp",
            font=("Arial", 11),
            command=self.cmd_init_grasp,
            bg="#2196F3",
            fg="white",
        ).pack(fill=tk.X, pady=2)

        tk.Label(
            self.right_frame, text="Grasp Object", font=("Arial", 11, "bold")
        ).pack(pady=(10, 5))
        self.grasp_button = tk.Button(
            self.right_frame,
            text="Grasp Selected",
            font=("Arial", 11),
            command=self.cmd_grasp,
            bg="#FF9800",
            fg="white",
            state=tk.DISABLED,
        )
        self.grasp_button.pack(fill=tk.X, pady=2)

        tk.Label(
            self.right_frame,
            text="Head Control (Use arrow keys)",
            font=("Arial", 11, "bold"),
        ).pack(pady=(10, 5))
        self.head_info = tk.Label(
            self.right_frame, text="Pan: 0.0  Tilt: 0.0", font=("Arial", 10)
        )
        self.head_info.pack(fill=tk.X, pady=5)

        tk.Label(self.right_frame, text="Status", font=("Arial", 11, "bold")).pack(
            pady=(10, 5)
        )
        self.status_label = tk.Label(
            self.right_frame,
            text="Initializing...",
            font=("Arial", 10),
            fg="blue",
            justify=tk.LEFT,
            wraplength=250,
        )
        self.status_label.pack(fill=tk.BOTH, expand=True)

        self.object_listbox.bind("<<ListboxSelect>>", self.on_object_select)

        # Bind arrow keys for head control
        self.root.bind("<Left>", lambda e: self.move_head_pan(-0.1))
        self.root.bind("<Right>", lambda e: self.move_head_pan(0.1))
        self.root.bind("<Up>", lambda e: self.move_head_tilt(0.1))
        self.root.bind("<Down>", lambda e: self.move_head_tilt(-0.1))

        # Start ROS2 node in background thread
        self.thread = threading.Thread(target=self.ros_thread, daemon=True)
        self.thread.start()

        # Update GUI every 30ms
        self.update_gui()

    def ros_thread(self):
        rclpy.init()
        self.node = Node("grasp_gui")
        self.command_pub = self.node.create_publisher(String, "/command", 10)
        self.detect_pub = self.node.create_publisher(String, "/detect", 10)
        self.head_traj_pub = self.node.create_publisher(
            JointTrajectory, "/head_controller/command", 10
        )
        self.head_action_client = ActionClient(
            self.node, FollowJointTrajectory, "/head_controller/follow_joint_trajectory"
        )
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(
            Image, "/head_front_camera/rgb/image_raw", self.on_rgb, camera_qos
        )
        self.node.create_subscription(
            Detection3DArray, "/object_centroids", self.on_detections, 10
        )
        self.node.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10
        )
        self.update_status("Ready")
        rclpy.spin(self.node)

    def on_rgb(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.update_status(f"RGB error: {e}")

    def on_detections(self, msg):
        self.latest_detections = msg

    def on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == "head_1_joint":
                self.head_pan = pos
            elif name == "head_2_joint":
                self.head_tilt = pos

    def update_gui(self):
        if self.latest_rgb is not None:
            self.display_image_with_detections()
            self.update_object_list()
        self.root.after(30, self.update_gui)

    def display_image_with_detections(self):
        img = self.latest_rgb.copy()
        h, w = img.shape[:2]

        # Convert BGR to RGB for PIL
        if len(img.shape) == 3 and img.shape[2] == 3:
            img = img[:, :, ::-1]
        pil_img = PILImage.fromarray(img)
        draw = ImageDraw.Draw(pil_img)

        for det in self.latest_detections.detections:
            x, y, bw, bh = det.xywh
            x1, y1 = int(x - bw / 2), int(y - bh / 2)
            x2, y2 = int(x + bw / 2), int(y + bh / 2)

            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            draw.rectangle([x1, y1, x2, y2], outline="lime", width=2)
            label = f"{det.name} ({det.confidence:.2f})"
            draw.text((x1, y1 - 10), label, fill="lime", font=None)

        # Resize for display
        pil_img.thumbnail((600, 600), PILImage.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(pil_img)
        self.image_label.config(image=photo)
        self.image_label.image = photo

    def update_object_list(self):
        current = self.object_listbox.get(0, tk.END)
        new_objects = [
            (det.name, f"{det.confidence:.2f}")
            for det in self.latest_detections.detections
        ]
        new_text = [f"{name} ({conf})" for name, conf in new_objects]

        if set(current) != set(new_text):
            self.object_listbox.delete(0, tk.END)
            for obj in new_text:
                self.object_listbox.insert(tk.END, obj)

            if len(new_text) > 0:
                self.grasp_button.config(state=tk.NORMAL)
            else:
                self.grasp_button.config(state=tk.DISABLED)

    def on_object_select(self, event):
        selection = self.object_listbox.curselection()
        if selection:
            self.grasp_button.config(state=tk.NORMAL)

    def cmd_detect(self):
        if self.detect_pub:
            queries = self.detect_entry.get().strip() or "bottle,cup,object"
            msg = String()
            msg.data = queries
            self.detect_pub.publish(msg)
            self.update_status(f"Detection triggered: {queries}")

    def cmd_home(self):
        self.send_command("home")

    def cmd_init_grasp(self):
        self.send_command("init_grasp")

    def cmd_grasp(self):
        selection = self.object_listbox.curselection()
        if selection:
            obj_text = self.object_listbox.get(selection[0])
            obj_name = obj_text.split(" (")[0]
            self.send_command(f"grasp {obj_name}")
            self.update_status(f"Grasping: {obj_name}")

    def send_command(self, cmd):
        if self.command_pub:
            msg = String()
            msg.data = cmd
            self.command_pub.publish(msg)
            self.update_status(f"Command sent: {cmd}")

    def update_status(self, msg):
        self.root.after(0, lambda: self.status_label.config(text=msg))

    def move_head_pan(self, delta):
        self.head_pan += delta
        self.head_pan = max(-1.57, min(1.57, self.head_pan))
        self.send_head_trajectory(self.head_pan, self.head_tilt)
        self.head_info.config(
            text=f"Pan: {self.head_pan:.2f}  Tilt: {self.head_tilt:.2f}"
        )

    def move_head_tilt(self, delta):
        self.head_tilt += delta
        self.head_tilt = max(-0.8, min(0.8, self.head_tilt))
        self.send_head_trajectory(self.head_pan, self.head_tilt)
        self.head_info.config(
            text=f"Pan: {self.head_pan:.2f}  Tilt: {self.head_tilt:.2f}"
        )

    def send_head_trajectory(self, pan, tilt):
        if not self.head_traj_pub or not self.head_action_client:
            return

        traj = JointTrajectory()
        traj.joint_names = ["head_1_joint", "head_2_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = Duration(sec=1)
        traj.points = [pt]

        # Publish to topic
        self.head_traj_pub.publish(traj)

        # Send via action
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        try:
            future = self.head_action_client.send_goal_async(goal)
        except Exception as e:
            self.update_status(f"Head action error: {e}")

    def on_closing(self):
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    gui = GraspGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
