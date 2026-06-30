"use client";

import { useState } from "react";

type State = "edit" | "confirm";
type Category = "drinks" | "fruits" | "snacks" | "food";

interface Item {
  id: string;
  name: string;
  category: Category;
  image: string;
}

const CATEGORY_EMOJI: Record<Category, string> = {
  drinks: "🥤",
  fruits: "🍎",
  snacks: "🍿",
  food: "🥫",
};

const RAW = "https://raw.githubusercontent.com/RoboCupAtHome/Incheon2026/main/objects/known_objects";

const ITEMS: Item[] = [
  // Drinks
  { id: "coke_can",          name: "Coca-Cola",          category: "drinks",  image: `${RAW}/drinks!drink/coke_can.jpeg` },
  { id: "coke_zero_can",     name: "Coke Zero",          category: "drinks",  image: `${RAW}/drinks!drink/coke_zero_can.jpeg` },
  { id: "milk",              name: "Milk",               category: "drinks",  image: `${RAW}/drinks!drink/milk.jpeg` },
  { id: "pepsi",             name: "Pepsi",              category: "drinks",  image: `${RAW}/drinks!drink/pepsi.jpeg` },
  { id: "red_bull",          name: "Red Bull",           category: "drinks",  image: `${RAW}/drinks!drink/red_bull.jpeg` },
  { id: "soju",              name: "Soju",               category: "drinks",  image: `${RAW}/drinks!drink/soju.jpeg` },
  // Fruits
  { id: "apple",             name: "Apple",              category: "fruits",  image: `${RAW}/fruits!fruit/apple.jpeg` },
  { id: "lemon",             name: "Lemon",              category: "fruits",  image: `${RAW}/fruits!fruit/lemon.jpeg` },
  { id: "mangostane",        name: "Mangosteen",         category: "fruits",  image: `${RAW}/fruits!fruit/mangostane.jpeg` },
  { id: "peach",             name: "Peach",              category: "fruits",  image: `${RAW}/fruits!fruit/peach.jpeg` },
  { id: "red_bellpepper",    name: "Red Bell Pepper",    category: "fruits",  image: `${RAW}/fruits!fruit/red_bellpepper.jpeg` },
  { id: "yellow_bellpepper", name: "Yellow Bell Pepper", category: "fruits",  image: `${RAW}/fruits!fruit/yellow_bellpepper.jpeg` },
  // Snacks
  { id: "pringles",          name: "Pringles",           category: "snacks",  image: `${RAW}/snacks!snack/pringles.jpeg` },
  { id: "seaweed",           name: "Seaweed",            category: "snacks",  image: `${RAW}/snacks!snack/seaweed.jpeg` },
  // Food
  { id: "cornflakes_2",      name: "Cornflakes",         category: "food",    image: `${RAW}/food/cornflakes_2.jpeg` },
  { id: "instant_noodles",   name: "Instant Noodles",    category: "food",    image: `${RAW}/food/instant_noodles.jpeg` },
];

const CATEGORIES: Category[] = ["drinks", "fruits", "snacks", "food"];

// Expand quantity map to a flat array of ids (e.g. {cola: 2} → ["cola", "cola"])
function expandOrder(qty: Record<string, number>): string[] {
  return Object.entries(qty).flatMap(([id, n]) => Array(n).fill(id));
}

export function CreateOrder({ finish }: { finish: (order: string[]) => void }) {
  const [state, setState] = useState<State>("edit");
  const [qty, setQty] = useState<Record<string, number>>({});

  const totalItems = Object.values(qty).reduce((a, b) => a + b, 0);

  return (
    <div className="fixed w-full h-full top-0 left-0 flex flex-col">
      {state === "edit" ? (
        <EditMode
          qty={qty}
          setQty={setQty}
          totalItems={totalItems}
          confirm={() => setState("confirm")}
        />
      ) : (
        <ConfirmMode
          qty={qty}
          finish={() => finish(expandOrder(qty))}
          cancel={() => setState("edit")}
        />
      )}
    </div>
  );
}

function EditMode({
  qty,
  setQty,
  totalItems,
  confirm,
}: {
  qty: Record<string, number>;
  setQty: React.Dispatch<React.SetStateAction<Record<string, number>>>;
  totalItems: number;
  confirm: () => void;
}) {
  const [activeCategory, setActiveCategory] = useState<Category>("drinks");

  const visible = ITEMS.filter((i) => i.category === activeCategory);

  const inc = (id: string) => setQty((prev) => ({ ...prev, [id]: (prev[id] ?? 0) + 1 }));
  const dec = (id: string) =>
    setQty((prev) => {
      const next = { ...prev };
      if ((next[id] ?? 0) <= 1) delete next[id];
      else next[id]--;
      return next;
    });

  return (
    <>
      {/* Category tabs */}
      <div className="flex flex-row bg-slate-700 shrink-0">
        {CATEGORIES.map((cat) => (
          <button
            key={cat}
            className={
              "flex-1 py-4 text-2xl font-bold capitalize transition-colors " +
              (activeCategory === cat ? "bg-slate-500 text-white" : "text-slate-300 hover:bg-slate-600")
            }
            onClick={() => setActiveCategory(cat)}
          >
            {`${CATEGORY_EMOJI[cat]} ${cat}`}
          </button>
        ))}
      </div>

      {/* Item grid */}
      <div className="flex-1 overflow-y-auto p-3 grid grid-cols-3 gap-3 content-start bg-slate-800">
        {visible.map((item) => {
          const count = qty[item.id] ?? 0;
          return (
            <div
              key={item.id}
              className={
                "rounded-xl flex flex-col overflow-hidden border-4 transition-all " +
                (count > 0 ? "border-white" : "border-transparent")
              }
            >
              {/* Image */}
              <div className="bg-white flex-1 min-h-0 relative">
                {/* eslint-disable-next-line @next/next/no-img-element */}
                <img
                  src={item.image}
                  alt={item.name}
                  className="w-full h-32 object-contain p-2"
                />
                {count > 0 && (
                  <span className="absolute top-1 right-1 bg-slate-700 text-white text-xl font-bold rounded-full w-8 h-8 flex items-center justify-center">
                    {count}
                  </span>
                )}
              </div>
              {/* Name */}
              <div className="bg-slate-600 px-2 py-1 text-white text-lg text-center leading-tight">
                {item.name}
              </div>
              {/* +/- controls */}
              <div className="flex flex-row bg-slate-700">
                <button
                  className="flex-1 py-3 text-3xl text-red-400 hover:bg-slate-600 transition-colors disabled:opacity-30"
                  onClick={() => dec(item.id)}
                  disabled={count === 0}
                >
                  −
                </button>
                <button
                  className="flex-1 py-3 text-3xl text-green-400 hover:bg-slate-600 transition-colors"
                  onClick={() => inc(item.id)}
                >
                  +
                </button>
              </div>
            </div>
          );
        })}
      </div>

      {/* Bottom bar */}
      <div className="flex flex-row shrink-0 h-24 gap-3 p-3 bg-slate-700">
        <div className="flex-1 bg-slate-600 rounded-xl flex items-center px-4 text-white text-2xl overflow-hidden">
          <span className="truncate">
            {totalItems === 0
              ? "No items selected"
              : Object.entries(qty)
                  .filter(([, n]) => n > 0)
                  .map(([id, n]) => `${n}× ${ITEMS.find((i) => i.id === id)!.name}`)
                  .join(", ")}
          </span>
        </div>
        <button
          className="w-32 rounded-xl text-white text-2xl font-bold bg-red-500 hover:bg-red-400 transition-colors"
          onClick={() => setQty({})}
        >
          Reset
        </button>
        <button
          disabled={totalItems === 0}
          onClick={confirm}
          className={
            "w-40 rounded-xl text-white text-2xl font-bold transition-colors " +
            (totalItems > 0 ? "bg-green-500 hover:bg-green-400" : "bg-green-800 opacity-50 cursor-not-allowed")
          }
        >
          Continue
        </button>
      </div>
    </>
  );
}

function ConfirmMode({
  qty,
  finish,
  cancel,
}: {
  qty: Record<string, number>;
  finish: () => void;
  cancel: () => void;
}) {
  const orderedItems = Object.entries(qty)
    .filter(([, n]) => n > 0)
    .map(([id, n]) => ({ item: ITEMS.find((i) => i.id === id)!, n }));

  return (
    <div className="flex flex-col h-full bg-slate-800 p-6 gap-6">
      <h1 className="text-5xl text-white font-bold">Confirm Order</h1>
      <div className="flex-1 flex flex-col gap-4 overflow-y-auto">
        {orderedItems.map(({ item, n }) => (
          <div key={item.id} className="flex flex-row items-center gap-4 bg-slate-700 rounded-xl p-4">
            {/* eslint-disable-next-line @next/next/no-img-element */}
            <img src={item.image} alt={item.name} className="w-16 h-16 object-contain bg-white rounded-lg p-1" />
            <div className="flex flex-col flex-1">
              <span className="text-white text-3xl">{item.name}</span>
              <span className="text-slate-400 text-xl capitalize">{item.category}</span>
            </div>
            <span className="text-white text-4xl font-bold">{n}×</span>
          </div>
        ))}
      </div>
      <div className="flex flex-row gap-4 h-24 shrink-0">
        <button
          className="flex-1 bg-green-500 hover:bg-green-400 rounded-xl text-white text-4xl font-bold transition-colors"
          onClick={finish}
        >
          Confirm
        </button>
        <button
          className="flex-1 bg-red-500 hover:bg-red-400 rounded-xl text-white text-4xl font-bold transition-colors"
          onClick={cancel}
        >
          Back
        </button>
      </div>
    </div>
  );
}
