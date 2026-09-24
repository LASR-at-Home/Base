"use client";

export function Ready({ ready }: { ready: () => void }) {
  return (
    <div className="fixed w-full h-full top-0 left-0 grid place-items-center">
      <div
        onClick={ready}
        className="grid place-items-center w-[60vw] rounded-full aspect-square bg-red-600 hover:bg-red-500 text-white text-6xl font-bold text-center"
      >
        Click here to start
      </div>
    </div>
  );
}
