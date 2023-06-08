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


def stream(camera, callbacks=None, window_name="stream"):
    if callbacks is None:
        callbacks = list()
    elif callable(callbacks):
        callbacks = [callbacks]
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, (600, 300))
    key = -1
    while True:
        camera_frames = camera()
        image = camera_frames.pop("image")
        if image is None or image.size == 0:
            print("Could not get camera image\n")
            sleep(0.5)
            continue

        continue_streaming = None
        for callback in callbacks:
            ret = callback(image, key, **camera_frames)
            if ret is not None:
                continue_streaming = ret

        cv2.imshow(window_name, image)
        key = cv2.waitKey(1)

        if continue_streaming is None:
            continue_streaming = (key == -1)
        if not continue_streaming:
            cv2.destroyAllWindows()
            break


class StreamCallbacks:
    def flip(image, key):
        cv2.flip(image, 1, dst=image)

    def get_save_every_k_callback(save_folder, save_every_k=1, extention='jpg'):
        def save(image, key, **kwargs):
            if save.counter % save_every_k == 0:
                image_name = f'{save.counter:04}.{extention}'
                saved = cv2.imwrite(osp.join(save_folder, image_name), image)
                if saved:
                    print(f"Saved image {image_name}")
                else:
                    print("Could not save image")
                save.counter += 1
        save.counter = 0
        return save

    def get_save_by_key_callback(save_folder, save_key=ord(' '), extention='jpg'):
        def save(image, key, **kwargs):
            if key == save_key:
                image_name = f'{save.counter:04}.{extention}'
                saved = cv2.imwrite(osp.join(save_folder, image_name), image)
                if saved:
                    print(f"Saved image {image_name}")
                else:
                    print("Could not save image")
                save.counter += 1
            return key in (-1, save_key)
        save.counter = 0
        return save


def collect_data_for_calibration(camera, save_folder, save_key=ord(' '), extention='png'):
    stream(camera,
        [StreamCallbacks.get_save_by_key_callback(save_folder,
            save_key=save_key, extention=extention),
        StreamCallbacks.flip],
        window_name='collect data for calibration')

