"use client";

import { useEffect, useRef, useState } from "react";
import { CreateOrder } from "./screens/CreateOrder";

import { Message, Ros, Topic } from "roslib";
import { Done } from "./screens/Done";

type Screen = "home" | "order" | "done" | "debug";
type ProductTopic = { products: string[] };

export function App({ rosIp }: { rosIp: string }) {
  const [ros, setROS] = useState<Ros>();
  // const [topics, setTopics] = useState<string[]>([]);
  const [screen, setScreen] = useState<Screen>("home");

  const confirmTopicRef = useRef<Topic>();
  const orderTopicRef = useRef<Topic<ProductTopic>>();
  useEffect(() => {
    let ros = new Ros({
      url: rosIp ? `ws://${rosIp}:9090` : "ws://127.0.0.1:9090",
    });

    ros.on("connection", () => {
      // ros.getTopics((data) => setTopics(data.topics), alert);

      setROS(ros);

      const screenTopic = new Topic<{ data: Screen }>({
        ros,
        name: "/tablet/screen",
        messageType: "std_msgs/String",
      });

      screenTopic.subscribe((message) => {
        setScreen(message.data);
      });

      confirmTopicRef.current = new Topic({
        ros,
        name: "/tablet/done",
        messageType: "std_msgs/Empty",
      });

      confirmTopicRef.current.advertise();

      orderTopicRef.current = new Topic<ProductTopic>({
        ros,
        name: "/tablet/order",
        messageType: "coffee_shop_ui/Order",
      });

      orderTopicRef.current.advertise();
    });
  }, []);

  if (screen === "done") {
    return (
      <Done
        done={() => {
          let msg = new Message({});
          confirmTopicRef.current!.publish(msg);
          setScreen("home");
        }}
      />
    );
  } else if (screen === "order") {
    return (
      <CreateOrder
        finish={(products) => {
          let msg = new Message({ products });
          orderTopicRef.current!.publish(msg as ProductTopic);
          setScreen("home");
        }}
      />
    );
  } else if (screen === "debug") {
    return (
      <>
        {ros ? "Connected to ROS =)" : "Connecting to ROS..."}
        {/*topics.map((topic) => (
          <h1 key={topic}>{topic}</h1>
        ))*/}
      </>
    );
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
