#!/usr/bin/env python3
"""Interactive tool to label locations on the map by clicking.

Usage:
    python3 label_locations.py [--output locations.yaml]

Click on the map, type the name in the text box at the bottom, press Enter to confirm.
Press 'u' to undo last, 's' or 'q' to save and quit.
"""

import argparse
import os
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
import numpy as np
import yaml
from PIL import Image


MAP_DIR = Path(__file__).parent
MAP_IMAGE = MAP_DIR / "map.pgm"
MAP_YAML = MAP_DIR / "map.yaml"


def load_map_params():
    with open(MAP_YAML) as f:
        params = yaml.safe_load(f)
    return params["resolution"], params["origin"]


def pixel_to_world(px, py, img_height, resolution, origin):
    wx = origin[0] + px * resolution
    wy = origin[1] + (img_height - py) * resolution
    return wx, wy


def world_to_pixel(wx, wy, img_height, resolution, origin):
    px = (wx - origin[0]) / resolution
    py = img_height - (wy - origin[1]) / resolution
    return px, py


def save(output_path, locations):
    data = {"locations": locations}
    with open(output_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=True)
    print(f"Saved {len(locations)} locations to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Label locations on the map")
    parser.add_argument("--output", default=str(MAP_DIR / "locations.yaml"))
    args = parser.parse_args()

    resolution, origin = load_map_params()
    img = np.array(Image.open(MAP_IMAGE))
    img_height, img_width = img.shape[:2]

    locations = {}
    if os.path.exists(args.output):
        with open(args.output) as f:
            data = yaml.safe_load(f) or {}
        locations = data.get("locations", {})
        print(f"Loaded {len(locations)} existing locations")

    # pending click coordinates waiting for a name
    pending = {"px": None, "py": None}
    markers = {}

    fig = plt.figure(figsize=(10, 11))
    ax = fig.add_axes([0.05, 0.12, 0.9, 0.85])
    ax_textbox = fig.add_axes([0.15, 0.03, 0.5, 0.05])
    ax_confirm = fig.add_axes([0.67, 0.03, 0.13, 0.05])
    ax_undo = fig.add_axes([0.82, 0.03, 0.13, 0.05])

    ax.imshow(img, cmap="gray", origin="upper")
    ax.set_title("Click on map → type name below → Enter or Confirm\n's'=save  'u'=undo  'q'=quit+save", fontsize=10)

    textbox = TextBox(ax_textbox, "Name: ", initial="")
    btn_confirm = Button(ax_confirm, "Confirm")
    btn_undo = Button(ax_undo, "Undo")

    # pending marker (blue cross before name is confirmed)
    pending_marker = [None]

    def redraw_all():
        for name, artists in list(markers.items()):
            for a in artists:
                a.remove()
            markers.clear()
        for name, loc in locations.items():
            wx, wy = loc["position"]["x"], loc["position"]["y"]
            px, py = world_to_pixel(wx, wy, img_height, resolution, origin)
            dot = ax.plot(px, py, "ro", markersize=8)[0]
            txt = ax.text(px + 5, py - 5, name, color="red", fontsize=9, fontweight="bold")
            markers[name] = [dot, txt]
        fig.canvas.draw_idle()

    redraw_all()

    def confirm_location(name):
        name = name.strip()
        if not name or pending["px"] is None:
            return
        wx, wy = pixel_to_world(pending["px"], pending["py"], img_height, resolution, origin)
        locations[name] = {
            "position": {"x": round(float(wx), 3), "y": round(float(wy), 3), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        print(f"Added '{name}' at ({wx:.3f}, {wy:.3f})")
        pending["px"] = pending["py"] = None
        if pending_marker[0]:
            pending_marker[0].remove()
            pending_marker[0] = None
        textbox.set_val("")
        redraw_all()

    def on_click(event):
        if event.inaxes != ax or event.button != 1:
            return
        pending["px"] = event.xdata
        pending["py"] = event.ydata
        wx, wy = pixel_to_world(pending["px"], pending["py"], img_height, resolution, origin)
        print(f"Pending: ({wx:.3f}, {wy:.3f}) — type a name and press Enter")
        if pending_marker[0]:
            pending_marker[0].remove()
        pending_marker[0] = ax.plot(event.xdata, event.ydata, "b+", markersize=14, markeredgewidth=2)[0]
        fig.canvas.draw_idle()
        # focus the textbox
        textbox.begin_typing(None)

    def on_key(event):
        if event.key == "u":
            do_undo(None)
        elif event.key in ("s", "q"):
            save(args.output, locations)
            if event.key == "q":
                plt.close()

    def do_undo(_event):
        if locations:
            name, _ = locations.popitem()
            print(f"Undone: removed '{name}'")
            redraw_all()
        else:
            print("Nothing to undo.")

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    textbox.on_submit(confirm_location)
    btn_confirm.on_clicked(lambda _: confirm_location(textbox.text))
    btn_undo.on_clicked(do_undo)

    plt.show()
    save(args.output, locations)


if __name__ == "__main__":
    main()
