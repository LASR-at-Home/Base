# Example reference photos for few-shot mode

This folder shows the layout `yoloe_test.py --refs` expects: **one folder per
object, named after the object, containing a few cropped example photos.**

```
refs_example/
├── bottle/          1.jpg  2.jpg        <- tight crops of a bottle
├── crisps/          1.jpg  2.jpg
└── fruit_smoothie/  1.jpg  2.jpg
```

The folder name is the class name (underscores or spaces both work, e.g.
`fruit_smoothie` matches `--classes "fruit smoothie"`).

These specific images are just a demo from a food dataset. **Make your own folder
with your own objects** — see below.

## How to make your own reference shots

1. **Take 3–10 photos of each object.** Vary the angle, distance and lighting a
   bit — that variety is what makes few-shot work well. A phone camera is fine.
2. **Crop each photo tightly around the object** (little background). On a Mac:
   open the photo in Preview → drag a rectangle around the object → Tools ▸ Crop
   (⌘K) → save. Any image editor works. If you take close-up photos where the
   object already fills the frame, you can skip cropping.
3. **Put the crops in a folder named after the object**, e.g. `refs/cereal_box/`.
   Repeat for each object.
4. **Run with `--refs`:**

   ```bash
   python yoloe_test.py \
     --weights yoloe-v8m-seg.pt --device cuda \
     --classes "cereal box,mug,bottle" --refs ./refs \
     --source ./test_images --save-dir ./out
   ```

That's it — no annotation, no training. The photos themselves are the prompt.

Tip: more and more-varied examples per object generally raise accuracy; 1–2 plain
crops is the bare minimum.
