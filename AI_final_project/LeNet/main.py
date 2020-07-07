from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
from lenet import LeNet


def normalize_image(images):
    for i in range(len(images)):
        image = images[i]
        mean = np.mean(image)
        std = np.std(image)
        images[i] = (images[i]-mean)/std
    return images

def one_hot_labels(labels):
    return np.eye(labels.max()+1)[labels]


def main():
    # image shape: N x H x W, pixel [0, 255]
    # label shape: N x 10
    with np.load('mnist.npz', allow_pickle=True) as f:
        x_train, y_train = f['x_train'], f['y_train']
        x_test, y_test = f['x_test'], f['y_test']
    print(x_train.shape, x_train[0].max(), x_train[0].min()) #(60000, 28, 28) 255 0 5
    print(x_test.shape, x_test[0].max(), x_test[0].min()) #(10000, 28, 28) 255 0 7
    print(y_train.shape)

    x_train = normalize_image(x_train)
    x_test = normalize_image(x_test)
    y_train = one_hot_labels(y_train)
    y_test = one_hot_labels(y_test)

    net = LeNet()
    averagetime,accuracies = net.fit(x_train, y_train, x_test, y_test, epoches=10, batch_size=16, lr=3e-3)
    

    accu = net.evaluate(x_test, labels=y_test)
    print("final accuracy {}".format(accu))
    print(averagetime,'s')

if __name__ == "__main__":
    main()


