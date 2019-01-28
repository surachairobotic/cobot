import cv2
import os
import numpy as np

cat_paths = ['bw', 'green', 'red']
x_train = []
for path in cat_paths:
  files = os.listdir(path)
  imgs = []
  for file in files:
    img = cv2.imread(path + '\\' + file, cv2.IMREAD_GRAYSCALE)
    print(img.shape)
    #img = np.arange(img.shape[0], img.shape[1])#.reshape((img.shape[0], img.shape[1], 1))
    img = img.astype('float32')
    imgs.append(img)

  x_train.append(np.stack(imgs) / 255.0)

# x_train[0].shape = [?,60,110]

#https://towardsdatascience.com/image-classification-python-keras-tutorial-kaggle-challenge-45a6332a58b8
