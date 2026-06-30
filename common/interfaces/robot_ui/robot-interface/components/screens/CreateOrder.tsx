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
  // Drinks (.jpg, folder drinks!drink)
  { id: "juice_pack",       name: "Juice Pack",      category: "drinks", image: `${RAW}/drinks!drink/juice_pack.jpg` },
  { id: "cola",             name: "Cola",             category: "drinks", image: `${RAW}/drinks!drink/cola.jpg` },
  { id: "milk",             name: "Milk",             category: "drinks", image: `${RAW}/drinks!drink/milk.jpg` },
  { id: "orange_juice",     name: "Orange Juice",     category: "drinks", image: `${RAW}/drinks!drink/orange_juice.jpg` },
  { id: "tropical_juice",   name: "Tropical Juice",   category: "drinks", image: `${RAW}/drinks!drink/tropical_juice.jpg` },
  { id: "red_wine",         name: "Red Wine",         category: "drinks", image: `${RAW}/drinks!drink/red_wine.jpg` },
  { id: "iced_tea",         name: "Iced Tea",         category: "drinks", image: `${RAW}/drinks!drink/iced_tea.jpg` },
  // Fruits (.png, folder fruits!fruit)
  { id: "orange",           name: "Orange",           category: "fruits", image: `${RAW}/fruits!fruit/orange.png` },
  { id: "pear",             name: "Pear",             category: "fruits", image: `${RAW}/fruits!fruit/pear.png` },
  { id: "peach",            name: "Peach",            category: "fruits", image: `${RAW}/fruits!fruit/peach.png` },
  { id: "strawberry",       name: "Strawberry",       category: "fruits", image: `${RAW}/fruits!fruit/strawberry.png` },
  { id: "apple",            name: "Apple",            category: "fruits", image: `${RAW}/fruits!fruit/apple.png` },
  { id: "lemon",            name: "Lemon",            category: "fruits", image: `${RAW}/fruits!fruit/lemon.png` },
  { id: "banana",           name: "Banana",           category: "fruits", image: `${RAW}/fruits!fruit/banana.png` },
  { id: "plum",             name: "Plum",             category: "fruits", image: `${RAW}/fruits!fruit/plum.png` },
  // Snacks (mixed formats, folder snacks!snack)
  { id: "cornflakes",       name: "Cornflakes",       category: "snacks", image: `${RAW}/snacks!snack/cornflakes.jpg` },
  { id: "pringles",         name: "Pringles",         category: "snacks", image: `${RAW}/snacks!snack/pringles.png` },
  { id: "cheezit",          name: "Cheez-It",         category: "snacks", image: `${RAW}/snacks!snack/cheezit.png` },
  // Food (.png, folder food)
  { id: "chocolate_jello",  name: "Chocolate Jello",  category: "food",   image: `${RAW}/food/chocolate_jello.png` },
  { id: "coffee_grounds",   name: "Coffee Grounds",   category: "food",   image: `${RAW}/food/coffee_grounds.png` },
  { id: "mustard",          name: "Mustard",          category: "food",   image: `${RAW}/food/mustard.png` },
  { id: "tomato_soup",      name: "Tomato Soup",      category: "food",   image: `${RAW}/food/tomato_soup.png` },
  { id: "tuna",             name: "Tuna",             category: "food",   image: `${RAW}/food/tuna.png` },
  { id: "strawberry_jello", name: "Strawberry Jello", category: "food",   image: `${RAW}/food/strawberry_jello.png` },
  { id: "spam",             name: "Spam",             category: "food",   image: `${RAW}/food/spam.png` },
  { id: "sugar",            name: "Sugar",            category: "food",   image: `${RAW}/food/sugar.png` },
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
  const [activeCategory, setActiveCategory] = useState<Category | "all">("all");

  const visible = activeCategory === "all" ? ITEMS : ITEMS.filter((i) => i.category === activeCategory);

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
        {(["all", ...CATEGORIES] as const).map((cat) => (
          <button
            key={cat}
            className={
              "flex-1 py-4 text-2xl font-bold capitalize transition-colors " +
              (activeCategory === cat ? "bg-slate-500 text-white" : "text-slate-300 hover:bg-slate-600")
            }
            onClick={() => setActiveCategory(cat)}
          >
            {cat === "all" ? "All" : `${CATEGORY_EMOJI[cat]} ${cat}`}
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
