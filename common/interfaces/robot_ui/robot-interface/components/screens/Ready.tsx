"use client";

export function Ready({ ready }: { ready: () => void }) {
  return (
    <div className="fixed w-full h-full top-0 left-0 grid place-items-center">
      <div
        onClick={ready}
        className="grid place-items-center w-[40vw] rounded-full aspect-square bg-blue-400 hover:bg-blue-300 text-white text-8xl"
      >
        Ready
      </div>
    </div>
  );
}
