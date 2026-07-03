import os
import cv2
import torch

# Import Grounding DINO modules
from groundingdino.util.inference import load_model, load_image, predict, annotate

# ==========================================
# 1. PATH CONFIGURATION
# ==========================================
# The config file already exists inside the cloned repository
model_config_path = "/home/hayeonglee/ros_ws/src/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"

# The 690MB weights file you downloaded manually
model_checkpoint_path = "/home/hayeonglee/ros_ws/src/Base/tasks/doing_laundry/doing_laundry/weights/groundingdino_swint_ogc.pth"

# Your input image and desired output file name
image_path = "/home/hayeonglee/test_img/basket.jpeg"  
output_path = "result_dino.png"

# ==========================================
# 2. DEVICE SETUP
# ==========================================
# Automatically use GPU if available in the Apptainer, otherwise fallback to CPU
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

try:
    # ==========================================
    # 3. LOAD MODEL & IMAGE
    # ==========================================
    print("Loading Grounding DINO model... (This might take a moment on CPU)")
    model = load_model(model_config_path, model_checkpoint_path, device=device)

    print(f"Loading image from '{image_path}'...")
    image_source, image = load_image(image_path)

    # ==========================================
    # 4. INFERENCE SETTINGS
    # ==========================================
    TEXT_PROMPT = "basket"
    BOX_THRESHOLD = 0.35  # Confidence threshold for the bounding box (0.0 to 1.0)
    TEXT_THRESHOLD = 0.25 # Confidence threshold for the text label

    print(f"Searching for '{TEXT_PROMPT}' in the image...")
    
    # Run the model to find the object
    boxes, logits, phrases = predict(
        model=model,
        image=image,
        caption=TEXT_PROMPT,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD,
        device=device
    )

    # ==========================================
    # 5. PROCESS & SAVE RESULTS
    # ==========================================
    print(f"Found {len(boxes)} object(s).")
    
    if len(boxes) > 0:
        # Draw the bounding boxes on the image
        print("Annotating image...")
        annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
        
        # Save the result
        cv2.imwrite(output_path, annotated_frame)
        print(f"Success! Result saved to '{output_path}'")
        
        # Print out the exact box coordinates for your robot arm logic
        # boxes are returned in normalized format: [center_x, center_y, width, height]
        print("\n--- Detection Details ---")
        for box, logit, phrase in zip(boxes, logits, phrases):
            print(f"Detected: {phrase}")
            print(f"Confidence: {logit:.2f}")
            print(f"Bounding Box (cx, cy, w, h): {box.tolist()}\n")
    else:
        print(f"Could not find any '{TEXT_PROMPT}' in the image with the current thresholds.")

except FileNotFoundError as e:
    print(f"\n[ERROR] File not found. Please double-check your paths:\n{e}")
except Exception as e:
    print(f"\n[ERROR] An unexpected error occurred:\n{e}")