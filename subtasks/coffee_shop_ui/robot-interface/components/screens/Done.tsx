"use client";

export function Done({ done }: { done: () => void }) {
  return (
    <div className="fixed w-full h-full top-0 left-0 grid place-items-center">
      <div
        onClick={done}
        className="grid place-items-center w-[40vw] rounded-full aspect-square bg-green-400 hover:bg-green-300 text-white text-8xl"
      >
        Done
      </div>
    </div>
  );
}
