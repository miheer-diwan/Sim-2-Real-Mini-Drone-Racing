
from PIL import Image 
import os
import cv2
import numpy as np
import shutil
import random

base_path = '/home/oliver/GateDataset/outputs'
rgba_path = base_path + '/RGBA'
bg_path = base_path + '/Background'
label_path = base_path + '/labels'
augmented_path = base_path + '/augmented'
 
# iterate over files in
# that directory

bg_imgs = []
for bg_filename in os.listdir(bg_path):
    bg_img_path = os.path.join(bg_path, bg_filename)

    # Load the background image using OpenCV
    bg_image = Image.open(bg_img_path)

    # Resize the background image to the desired size
    des_size = (720,720)
    bg_image = bg_image.resize(des_size)
    bg_imgs.append(bg_image)
    print(len(bg_imgs))

counter = 0
for rgba_filename in os.listdir(rgba_path):
    filename = rgba_filename.rsplit('.', 1)[0]
    txt_filename = filename + '.txt'
    txt_path = os.path.join(label_path, txt_filename)
    img_path = os.path.join(rgba_path, rgba_filename)
    fg_img = Image.open(img_path)
    sampled_bgs = random.sample(bg_imgs, 2)
    for bg_img in sampled_bgs:

        aug_img = bg_img.copy()
        aug_img.paste(fg_img, (0,0), mask = fg_img)

        aug_img.save(augmented_path + '/train/{}.png'.format(counter))

        shutil.copy(txt_path,augmented_path + '/train/{}.txt'.format(counter))
        print(counter)
        # if counter%10 == 0:
        #     cv2.imwrite(augmented_path + '/validate/{}.png'.format(counter), result)
        #     shutil.copy(txt_path,augmented_path + '/validate/{}.txt'.format(counter))
        #     print(counter)
        # elif counter%10 == 1:
        #     cv2.imwrite(augmented_path + '/test/{}.png'.format(counter), result)
        #     shutil.copy(txt_path,augmented_path + '/test/{}.txt'.format(counter))
        #     print(counter)
        # else:
        #     cv2.imwrite(augmented_path + '/train/{}.png'.format(counter), result)
        #     shutil.copy(txt_path,augmented_path + '/train/{}.txt'.format(counter))
        #     print(counter)

        counter += 1
        

