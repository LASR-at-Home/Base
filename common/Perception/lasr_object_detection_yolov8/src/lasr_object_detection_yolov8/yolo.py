import numpy as np

from PIL import Image
from ultralytics import YOLO

# model cache
loaded_models = {}

def detect(dataset, min_confidence, debug, encoding, width, height, image_data):
    # decode the image
    print('Decoding')
    if encoding == 'bgr8':
        img = Image.frombytes('RGB', (width, height), image_data, 'raw')

        # BGR => RGB
        img = Image.fromarray(np.array(img)[:,:,::-1])
    else:
        raise Exception("Unsupported format.")
    
    # load model
    print('Loading model')
    model = None
    if dataset in loaded_models:
        model = loaded_models[dataset]
    else:
        model = YOLO(dataset)
        print('Loaded', dataset, 'model')

        loaded_models[dataset] = model

    # run inference
    print('Running inference')
    results = model(img)
    result = results[0]
    print('Inference complete')

    # construct response
    detected_objects = []
    object_count = result.boxes.cls.size(dim=0)
    has_segment_masks = result.masks is not None
    for i in range(0, object_count):
        confidence = float(result.boxes.conf.numpy()[i])
        if confidence >= min_confidence:
            detection = {
                'name': result.names[int(result.boxes.cls[i])],
                'confidence': confidence,
                'xywh': result.boxes.xywh[i].numpy().astype(int).tolist(),
            }

            # copy segmented mask if available
            if has_segment_masks:
                detection['xyseg'] = result.masks.xy[i].flatten().astype(int).tolist()

            detected_objects.append(detection)
    
    plotted = None
    if debug:
        plotted = result.plot().tobytes()
    
    return plotted, detected_objects
