"use client";

import { useEffect, useRef, useState } from "react";
import { Message, Ros, Topic } from "roslib";

import { Done } from "./screens/Done";
import { Ready } from "./screens/Ready";
import { YesNo } from "./screens/YesNo";

type Screen = "home" | "done" | "ready" | "yes_no" | "debug";
type ConfirmTopic = { value: boolean };

export default function App({ rosIp }: { rosIp: string }) {
  const [ros, setROS] = useState<Ros>();
  const [screen, setScreen] = useState<Screen>("home");

  const doneTopicRef = useRef<Topic>();
  const readyTopicRef = useRef<Topic>();
  const confirmTopicRef = useRef<Topic<ConfirmTopic>>();

  useEffect(() => {
    const ros = new Ros({
      url: rosIp ? `ws://${rosIp}:9090` : "ws://127.0.0.1:9090",
    });

    ros.on("connection", () => {
      setROS(ros);

      const screenTopic = new Topic<{ data: Screen }>({
        ros,
        name: "/tablet/screen",
        messageType: "std_msgs/msg/String",
      });
      screenTopic.subscribe((message) => setScreen(message.data));

      doneTopicRef.current = new Topic({
        ros,
        name: "/tablet/done",
        messageType: "std_msgs/msg/Empty",
      });
      doneTopicRef.current.advertise();

      readyTopicRef.current = new Topic({
        ros,
        name: "/tablet/ready",
        messageType: "std_msgs/msg/Empty",
      });
      readyTopicRef.current.advertise();

      confirmTopicRef.current = new Topic<ConfirmTopic>({
        ros,
        name: "/tablet/confirm",
        messageType: "robot_ui/msg/Confirm",
      });
      confirmTopicRef.current.advertise();
    });
  }, []);

  if (screen === "done") {
    return (
      <Done
        done={() => {
          doneTopicRef.current!.publish(new Message({}));
          setScreen("home");
        }}
      />
    );
  } else if (screen === "ready") {
    return (
      <Ready
        ready={() => {
          readyTopicRef.current!.publish(new Message({}));
          setScreen("home");
        }}
      />
    );
  } else if (screen === "yes_no") {
    return (
      <YesNo
        yes={() => {
          confirmTopicRef.current!.publish(new Message({ value: true }) as ConfirmTopic);
          setScreen("home");
        }}
        no={() => {
          confirmTopicRef.current!.publish(new Message({ value: false }) as ConfirmTopic);
          setScreen("home");
        }}
      />
    );
  } else if (screen === "debug") {
    return <>{ros ? "Connected to ROS =)" : "Connecting to ROS..."}</>;
  } else {
    return ros ? (
      <img
        src="/tiago.jpg"
        className="fixed top-0 left-0 w-full h-full object-cover"
      />
    ) : (
      <h1 className="text-9xl p-4">Connecting...</h1>
    );
  }
}
