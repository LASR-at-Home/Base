import { App } from "@/components/App";

export default function Home() {
  console.info("Advertising", process.env.ROS_IP, "to clients.");

  return (
    <main className="select-none">
      <App rosIp={process.env.ROS_IP as string} />
    </main>
  );
}
