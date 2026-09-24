"use client";

export function YesNo({ yes, no }: { yes: () => void; no: () => void }) {
  return (
    <div className="fixed w-full h-full top-0 left-0 flex flex-row">
      <div
        onClick={yes}
        className="grid place-items-center aspect-square bg-green-400 hover:bg-green-300 text-white text-8xl"
      >
        Yes
      </div>
      <div
        onClick={no}
        className="grid place-items-center aspect-square bg-red-400 hover:bg-red-300 text-white text-8xl"
      >
        No
      </div>
    </div>
  );
}
