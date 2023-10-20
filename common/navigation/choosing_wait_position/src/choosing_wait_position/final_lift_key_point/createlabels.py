import json 
import os 
import cv2 
import matplotlib.pyplot as plt 


def converter(file_labels,file_image):
    img = cv2.imread(file_image)
    img_w, img_h = img.shape[1], img.shape[0]

    with open(file_labels_example) as f: 
        lines_txt = f.readlines()
        lines = [] 
        for line in lines_txt:
            lines.append([int(line.split()[0])] + [round(float(el), 5) for el in line.split()[1:]])


    bboxes = [] 

    for line in lines:
        x_c, y_c, w, h = round(line[1] * img_w), round(line[2] * img_h), round(line[3] * img_w), round(line[4] * img_h)
        bboxes.append([round(x_c - w/2), round(y_c - h/2), round(x_c + w/2), round(y_c + h/2)])

    x1, y1, x2, y2, = bboxes[-1]

    keypoints = []
    keypoint = (round((x1 + x2)/2), round((y1+y2)/2),0)
    keypoints.append(keypoint)

                    
    return bboxes,keypoints



def dump2json(bboxes, keypoints, file_json):
    annotations = {}
    annotations['bboxes'], annotations['keypoints'] = bboxes, keypoints
    with open(file_json, "w") as f:
        json.dump(annotations, f)



#Example

file_image_example = 'data/train/images/cropped1ezgif-frame-039_jpg.rf.d24e6236c1b349c58e5152fa7079f151.jpg'


img = cv2.imread(file_image_example)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

plt.figure(figsize=(15,15))
plt.imshow(img)
#plt.show()

file_labels_example = 'data/train/labels/cropped1ezgif-frame-039_jpg.rf.d24e6236c1b349c58e5152fa7079f151.txt'

bboxes,keypoints = converter(file_labels_example, file_image_example)

print("Bboxes:", bboxes)

for bbox_idx, bbox in enumerate(bboxes):
    top_left_corner = tuple([bbox[0],bbox[1]])
    bottom_right_corner = tuple([bbox[2],bbox[3]])
    img = cv2.rectangle(img, top_left_corner,bottom_right_corner,(0,255,0),2)


    c1, c2, c3 = keypoints[bbox_idx]
    print("c1: ", c1, "c2: ", c2)
    img = cv2.circle(img, (round(c1),round(c2)), 2, (255,0,0),4)


plt.figure(figsize=(15,15))
plt.imshow(img)
plt.show()



#Actual Labelling 

IMAGES = 'data/test/images'
LABELS = 'data/test/labels'
ANNOTATIONS = 'data/test/annotations'

files_names = [file.split('.jpg')[0] for file in os.listdir(IMAGES)]


for file in files_names:
    file_labels = os.path.join(LABELS, file + ".txt")
    file_image = os.path.join(IMAGES, file + ".jpg")
    bboxes, keypoints = converter(file_labels, file_image)
    dump2json(bboxes,keypoints,os.path.join(ANNOTATIONS,file + '.json'))

IMAGES = 'data/valid/images'
LABELS = 'data/valid/labels'
ANNOTATIONS = 'data/valid/annotations'

files_names = [file.split('.jpg')[0] for file in os.listdir(IMAGES)]


for file in files_names:
    file_labels = os.path.join(LABELS, file + ".txt")
    file_image = os.path.join(IMAGES, file + ".jpg")
    bboxes, keypoints = converter(file_labels, file_image)
    dump2json(bboxes,keypoints,os.path.join(ANNOTATIONS,file + '.json'))

