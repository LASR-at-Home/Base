import dynamic from "next/dynamic";

const App = dynamic(() => import("@/components/App"), {
  ssr: false,
  loading: () => <h1 className="text-9xl p-4">Loading...</h1>,
});

export default function Home() {
  console.info("Advertising", process.env.ROS_IP, "to clients.");

  return (
    <main className="select-none">
      <App rosIp={process.env.ROS_IP as string} />
    </main>
  );
}
