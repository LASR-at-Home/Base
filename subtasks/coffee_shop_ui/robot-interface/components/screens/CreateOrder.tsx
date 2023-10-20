"use client";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faPlusCircle, faMinusCircle } from "@fortawesome/free-solid-svg-icons";
import { SetStateAction, useState } from "react";

type State = "edit" | "confirm";

type Item =
  | "coffee"
  | "biscuits"
  | "sandwich"
  | "orange_juice"
  | "smoothie"
  | "granola";

const items: Item[] = [
  "coffee",
  "biscuits",
  "sandwich",
  "orange_juice",
  "smoothie",
  "granola",
];

const NAMES: { [key in Item]: string } = {
  coffee: 'Coffee',
  biscuits: 'Biscuits',
  sandwich: 'Sandwich',
  orange_juice: 'Juice',
  smoothie: 'Smoothie',
  granola: 'Granola'
}

export function CreateOrder({ finish }: { finish: (order: Item[]) => void }) {
  const [state, setState] = useState<State>("edit");
  const [order, setOrder] = useState<Item[]>([]);

  return (
    <div className="fixed w-full h-full top-0 left-0 flex flex-col p-4 gap-4">
      {state === "edit" ? (
        <EditMode
          order={order}
          setOrder={setOrder}
          confirm={() => setState("confirm")}
        />
      ) : (
        <ConfirmMode
          order={order}
          finish={() => finish(order)}
          cancel={() => setState("edit")}
        />
      )}
    </div>
  );
}

function ConfirmMode({
  order,
  finish,
  cancel,
}: {
  order: Item[];
  finish: () => void;
  cancel: () => void;
}) {
  return (
    <>
      <div className="flex flex-col gap-4 flex-[4]">
        <h1 className="text-5xl">Your Order</h1>
        <div className="flex flex-col flex-grow overflow-y-scroll min-h-0">
          {items
            .filter((item) => order.includes(item))
            .map((item) => (
              <div
                className="p-4 flex flex-row text-3xl items-center gap-4"
                key={item}
              >
                <span className="tracking-wide text-right w-[86px]">
                  {order.filter((x) => x === item).length}x
                </span>{" "}
                <img
                  className="h-[64px] w-[64px] object-contain"
                  src={`/objects/${item}.png`}
                />{" "}
                <span className="capitalize">{NAMES[item]}</span>
              </div>
            ))}
        </div>
      </div>
      <div className="flex flex-row gap-4 flex-1">
        <div
          className="flex-1 bg-green-400 grid place-items-center text-white text-5xl cursor-pointer"
          onClick={finish}
        >
          Confirm
        </div>
        <div
          className="flex-1 bg-red-400 grid place-items-center text-white text-5xl cursor-pointer"
          onClick={cancel}
        >
          Back
        </div>
      </div>
    </>
  );
}

function EditMode({
  order,
  setOrder,
  confirm,
}: {
  order: Item[];
  setOrder: React.Dispatch<SetStateAction<Item[]>>;
  confirm: () => void;
}) {
  return (
    <>
      <div className="flex flex-row gap-4 flex-[2] min-h-0">
        <Card item="coffee" order={order} setOrder={setOrder} />
        <Card item="sandwich" order={order} setOrder={setOrder} />
        <Card item="smoothie" order={order} setOrder={setOrder} />
      </div>
      <div className="flex flex-row gap-4 flex-[2] min-h-0">
        <Card item="orange_juice" order={order} setOrder={setOrder} />
        <Card item="granola" order={order} setOrder={setOrder} />
        <Card item="biscuits" order={order} setOrder={setOrder} />
      </div>
      <div className="flex flex-row gap-4 flex-1">
        <div
          className={
            "flex-1 bg-green-400 grid place-items-center text-white text-5xl transition-colors cursor-pointer" +
            (order.length ? "" : " bg-green-200 disabled")
          }
          onClick={order.length ? confirm : undefined}
        >
          Continue
        </div>
        <div
          className="flex-1 bg-red-400 grid place-items-center text-white text-5xl cursor-pointer"
          onClick={() => setOrder([])}
        >
          Reset
        </div>
      </div>
    </>
  );
}

const NEW_DESIGN = true;

function Card({
  item,
  order,
  setOrder,
}: {
  item: Item;
  order: Item[];
  setOrder: React.Dispatch<SetStateAction<Item[]>>;
}) {
  const count = order.filter((x) => x == item).length;

  const dec = () =>
    setOrder((order) => {
      const itemIndex = order.indexOf(item);
      if (itemIndex == -1) {
        return order;
      } else {
        const newOrder = [...order];
        newOrder.splice(itemIndex, 1);
        return newOrder;
      }
    });

  const inc = () => setOrder((order) => [...order, item]);

  if (NEW_DESIGN) {
    return (
      <div className="flex-1 flex flex-col bg-slate-400">
        <div className="flex-[6] min-h-0 grid">
          <img
            style={{
              gridArea: "1/1",
            }}
            className="p-4 w-full h-full object-contain"
            src={`/objects/${item}.png`}
          />
          <div
            style={{
              gridArea: "1/1",
            }}
            className="w-full h-full flex flex-row"
          >
            <div className="flex-1 grid place-items-center cursor-pointer" onClick={dec}>
              <div className="p-4 rounded-full opacity-75 pr-16 hover:opacity-100 transition-all">
                <FontAwesomeIcon icon={faMinusCircle} size={"5x"} color="#ed3e32" />
              </div>
            </div>
            <div className="flex-1 grid place-items-center cursor-pointer" onClick={inc}>
              <div className="p-4 rounded-full opacity-75 pl-16 hover:opacity-100 transition-all">
                <FontAwesomeIcon icon={faPlusCircle} size={"5x"} color="#2e632e" />
              </div>
            </div>
          </div>
        </div>
        <div className="flex-[2] flex flex-row bg-slate-500 text-5xl">
          <div className="flex-grow capitalize text-white flex flex-row items-center p-4 tracking-tighter">
            {NAMES[item]}
          </div>
          <div className="w-[64px] grid place-items-center text-white">
            {count}
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="flex-1 flex flex-col bg-slate-400">
      <div className="relative">
        <div className="absolute p-4 text-2xl text-white capitalize">
          {NAMES[item]}
        </div>
      </div>
      <div className="flex-[4] grid place-items-center min-h-0 p-4">
        <img className="h-full object-fill" src={`/objects/${item}.png`} />
      </div>
      <div className="flex-1 flex flex-row">
        <div
          className="flex-1 bg-slate-500 grid place-items-center text-white text-3xl hover:bg-slate-600 cursor-pointer transition-colors"
          onClick={dec}
        >
          <FontAwesomeIcon icon={faMinusCircle} height={32} />
        </div>
        <div className="flex-1 bg-slate-500 grid place-items-center text-white text-3xl">
          {count}
        </div>
        <div
          className="flex-1 bg-slate-500 grid place-items-center text-white text-3xl hover:bg-slate-600 cursor-pointer transition-colors"
          onClick={inc}
        >
          <FontAwesomeIcon icon={faPlusCircle} height={32} />
        </div>
      </div>
    </div>
  );
}
