import os
import cv2
import torch
import numpy as np
from PIL import Image

# Import Grounding DINO
from groundingdino.util.inference import load_model, load_image, predict

# Import rembg
from rembg import remove

# ==========================================
# 1. PATH CONFIGURATION
# ==========================================
model_config_path = "/home/robocup/ewan/ros_ws/src/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
model_checkpoint_path = "/home/robocup/ewan/ros_ws/src/Base/tasks/doing_laundry/doing_laundry/weights/groundingdino_swint_ogc.pth"

image_path = "/home/hayeonglee/test_img/white.jpeg"
output_cropped_path = "result_1_cropped.png"  # Intermediate result (just the box)
output_final_path = "result_2_final_rembg.png"  # Final result (transparent)

# ==========================================
# 2. DEVICE SETUP
# ==========================================
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"[*] Using device: {device}")

try:
    # ==========================================
    # 3. LOAD MODELS
    # ==========================================
    print("[*] Loading Grounding DINO model...")
    model = load_model(model_config_path, model_checkpoint_path, device=device)

    print(f"[*] Loading image '{image_path}'...")
    image_source, image_tensor = load_image(image_path)

    # Get original image dimensions (Height, Width, Channels)
    # image_source is an OpenCV BGR numpy array
    h, w, _ = image_source.shape

    # ==========================================
    # 4. RUN GROUNDING DINO
    # ==========================================
    TEXT_PROMPT = "face"
    BOX_THRESHOLD = 0.35
    TEXT_THRESHOLD = 0.25

    print(f"[*] Searching for '{TEXT_PROMPT}'...")
    boxes, logits, phrases = predict(
        model=model,
        image=image_tensor,
        caption=TEXT_PROMPT,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD,
        device=device,
    )

    if len(boxes) == 0:
        print(f"[!] Could not find any '{TEXT_PROMPT}'. Exiting.")
        exit()

    # ==========================================
    # 5. FIND THE BEST BOX & CROP
    # ==========================================
    # If it finds multiple shirts, we take the one with the highest confidence score
    best_idx = torch.argmax(logits)
    best_box = boxes[best_idx]
    confidence = logits[best_idx]

    print(f"[*] Found target! Confidence: {confidence:.2f}")

    # Convert normalized (cx, cy, w, h) -> absolute pixel coordinates (xmin, ymin, xmax, ymax)
    cx, cy, bw, bh = best_box
    xmin = int((cx - bw / 2) * w)
    ymin = int((cy - bh / 2) * h)
    xmax = int((cx + bw / 2) * w)
    ymax = int((cy + bh / 2) * h)

    # Add a 5% margin (padding) around the box.
    # rembg works much better when it can see a little bit of the background.
    margin_x = int((xmax - xmin) * 0.1)
    margin_y = int((ymax - ymin) * 0.1)

    xmin = max(0, xmin - margin_x)
    ymin = max(0, ymin - margin_y)
    xmax = min(w, xmax + margin_x)
    ymax = min(h, ymax + margin_y)

    print(f"[*] Cropping image at coordinates: x({xmin}:{xmax}), y({ymin}:{ymax})")
    cropped_img_bgr = image_source[ymin:ymax, xmin:xmax]

    # Save the intermediate cropped image just so you can verify what DINO saw
    cv2.imwrite(output_cropped_path, cropped_img_bgr)

    # ==========================================
    # 6. RUN REMBG ON THE CROP
    # ==========================================
    print("[*] Passing the cropped image to rembg...")

    # Convert OpenCV BGR to PIL RGB format (rembg prefers PIL Images)
    cropped_img_rgb = cv2.cvtColor(cropped_img_bgr, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(cropped_img_rgb)

    # Remove background!
    final_image = remove(pil_image)

    # Save final output
    final_image.save(output_final_path)
    print(f"[*] SUCCESS! Pipeline complete.")
    print(f"    - Cropped box saved to : {output_cropped_path}")
    print(f"    - Final cutout saved to: {output_final_path}")

except Exception as e:
    print(f"\n[ERROR] An error occurred:\n{e}")
