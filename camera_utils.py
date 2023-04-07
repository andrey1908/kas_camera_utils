import numpy as np
import cv2
from time import time, sleep
import os.path as osp


def get_image_resolution(camera):
    image = camera()
    w = image.shape[1]
    h = image.shape[0]
    return w, h


def get_fps(camera, duration=5):
    start = time()
    now = start
    num_images = 0
    while now - start < duration:
        camera()
        num_images += 1
        now = time()
    fps = num_images / duration
    return fps


def stream(camera, callbacks=None, window_name=""):
    if callbacks is None:
        callbacks = list()
    elif callable(callbacks):
        callbacks = [callbacks]
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    while True:
        image = camera()
        if image is None or image.size == 0:
            print("Could not get camera image\n")
            sleep(0.5)
            continue
        for callback in callbacks:
            callback(image)
        cv2.imshow(window_name, image)
        key = cv2.waitKey(1)
        if key != -1:
            cv2.destroyAllWindows()
            break


class StreamCallbacks:
    def flip(image):
        cv2.flip(image, 1, dts=image)

    def get_save_callback(save_folder, save_every_k=1, extention='jpg'):
        def save(image):
            if save.counter % save_every_k == 0:
                cv2.imwrite(osp.join(save_folder, f'{save.counter:04}.{extention}'),
                    image)
            save.counter += 1
        save.counter = 0
        return save


def collect_data_for_calibration(camera, save_folder, save_every_k=1, extention='png'):
    stream(camera,
        [StreamCallbacks.get_save_callback(save_folder,
            save_every_k=save_every_k, extention=extention),
        StreamCallbacks.flip],
        window_name='collect data for calibration')
