import os
import sys
import random

# validate input
if len(sys.argv) < 2 or len(sys.argv) > 3:
    print('usage: python makeTestFile.py' + \
        '<name of image folder in getImage/datasets> [full path to lasr_darknet_config for use on different system]')
    sys.exit()
    
# get dataset name
dataset = sys.argv[1]

# lasr_darknet_config path

lasr_path = os.path.dirname(os.path.realpath(__file__)) if not os.path.exists("/.dockerenv") else ""

# folder paths
cfg_path = lasr_path + '/config/' + dataset
img_path = lasr_path + '/datasets/' + dataset
bkp_path = lasr_path + '/weights/' + dataset

# file paths
data_path = cfg_path + '/' + dataset + '.data'
train_path = cfg_path + '/train_' + dataset + '.txt'
valid_path = cfg_path + '/valid_' + dataset + '.txt'
names_path = img_path + '/classes.txt'
yv4_path = cfg_path + '/yolov4.cfg'
yv4tiny_path = cfg_path + '/yolov4-tiny.cfg'
yv4tiny_tplt_path = lasr_path + '/.template/yolov4-tiny.cfg'
yv4_tplt_path = lasr_path + '/.template/yolov4.cfg'

# validate paths
# write something here!!!!

# create dirs
if not os.path.exists(cfg_path):
    os.makedirs(cfg_path)
if not os.path.exists(bkp_path):
    os.makedirs(bkp_path)

# get number of classes
classes = 0
with open(names_path) as f:
    for line in f:
        classes += 1

# SET GEN PATHS
if len(sys.argv) == 3:
    gen_path = sys.argv[2]
else:
    gen_path = lasr_path

# set gen paths
cfg_gen_path = gen_path + '/config/' + dataset
img_gen_path = gen_path + '/datasets/' + dataset
bkp_gen_path = gen_path + '/weights/' + dataset
train_gen_path = cfg_gen_path + '/train_' + dataset + '.txt'
valid_gen_path = cfg_gen_path + '/valid_' + dataset + '.txt'
names_gen_path = img_gen_path + '/classes.txt'

# --- CREATE TRAIN AND VALID FILES --- #
train_file = open(train_path, 'w')
valid_file = open(valid_path, 'w')

# find jpgs in "datasets" folder and add all paths to train and valid lists
for root, dirs, files in os.walk(img_path):
    for filename in files:
        if filename.endswith('.jpg'):
            text_out = img_gen_path + '/' + filename + '\n'
            # one in ten files is a validation image
            if (random.randint(-1, 10)):
                train_file.write(text_out)
            else:
                valid_file.write(text_out)

train_file.close()
valid_file.close()



# --- CREATE DATA FILE --- #
data_file = open(data_path, 'w')
data_file.write('classes = ' + str(classes) + '\n')
data_file.write('train = ' + train_gen_path + '\n')
data_file.write('valid = ' + valid_gen_path + '\n')
data_file.write('names = ' + names_gen_path + '\n')
data_file.write('backup = ' + bkp_gen_path + '\n')



# --- CREATE YOLOV4 TINY AND YOLOV4 CFG FILES --- #
cfg_file = [open(yv4tiny_path, 'w'), open(yv4_path, 'w')]
templates = [yv4tiny_tplt_path, yv4_tplt_path]

max_batches = classes * 2000
filters = (classes + 5) * 3

for i in range(2):
    filters_lines = []
    filters_last = 0

    # first pass - find filters lines to be replaced
    with open(templates[i]) as f:
        for lineno, line in enumerate(f):
            if line.startswith('filters'):
                filters_last = lineno
            elif line.startswith('[yolo]'):
                filters_lines.append(filters_last)

    # second pass - copy lines to new cfg with replacement
    with open(templates[i]) as f:
        for lineno, line in enumerate(f):
            if lineno in filters_lines:
                cfg_file[i].write('filters=' + str(filters) + '\n')
            elif line.startswith('classes'):
                cfg_file[i].write('classes=' + str(classes) + '\n')
            elif line.startswith('max_batches'):
                cfg_file[i].write('max_batches=' + str(max_batches) + '\n')
            elif line.startswith('steps='):
                cfg_file[i].write('steps=' + str((max_batches / 10) * 8) + ',' + str((max_batches / 10) * 9) + '\n')
            else:
                cfg_file[i].write(line)
