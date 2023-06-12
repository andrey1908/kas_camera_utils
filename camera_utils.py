import numpy as np
import cv2
from time import time, sleep
import os.path as osp
import glob


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
    continue_streaming = True
    while continue_streaming:
        camera_frames = camera()
        if isinstance(camera_frames, np.ndarray):  # backward compatibility
            camera_frames = {"image": camera_frames}
        image = camera_frames.pop("image")
        if image is None or image.size == 0:
            print("Could not get camera image\n")
            sleep(0.5)
            continue

        continue_streaming_is_set_by_callback = False
        for callback in callbacks:
            ret = callback(image, key, **camera_frames)
            if ret is not None:
                continue_streaming = (continue_streaming and ret)
                continue_streaming_is_set_by_callback = True

        cv2.imshow(window_name, image)
        key = cv2.waitKey(1)

        if not continue_streaming_is_set_by_callback:
            continue_streaming = (key == -1)

    cv2.destroyAllWindows()


class ImagesSavePathsGenerator:
    def __init__(self, save_folder, extention, continue_saving=False,
            start_from=0):
        self.save_folder = save_folder
        self.extention = extention
        self.continue_saving = continue_saving
        self.start_from = start_from

        if self.continue_saving:
            files = glob.glob(f"{save_folder}/????.{extention}")
            max_num = 0
            for file in files:
                num = osp.splitext(osp.basename(file))[0]
                if num.isdigit():
                    num = int(num)
                    max_num = max(max_num, num)
            self.counter = max_num + 1
        else:
            self.counter = self.start_from

    def __call__(self):
        next_image_save_path = f'{self.counter:04}.{self.extention}'
        next_image_save_path = osp.join(self.save_folder, next_image_save_path)
        self.counter += 1
        return next_image_save_path


class StreamCallbacks:
    def flip(image, key):
        cv2.flip(image, 1, dst=image)

    def get_save_every_k_callback(save_folder, save_every_k=1, extention='jpg',
            continue_saving=False, start_from=0):
        images_save_paths_generator = ImagesSavePathsGenerator(save_folder, extention,
            continue_saving=continue_saving, start_from=start_from)

        def save(image, key, **kwargs):
            if save.counter % save_every_k == 0:
                image_path = images_save_paths_generator()
                saved = cv2.imwrite(image_path, image)
                if saved:
                    print(f"Saved image {osp.basename(image_path)}")
                else:
                    print("Could not save image")
        save.counter = 0

        return save

    def get_save_by_key_callback(save_folder, save_key=ord(' '), extention='jpg',
            continue_saving=False, start_from=0):
        images_save_paths_generator = ImagesSavePathsGenerator(save_folder, extention,
            continue_saving=continue_saving, start_from=start_from)

        def save(image, key, **kwargs):
            if key == save_key:
                image_path = images_save_paths_generator()
                saved = cv2.imwrite(image_path, image)
                if saved:
                    print(f"Saved image {osp.basename(image_path)}")
                else:
                    print("Could not save image")
            continue_streaming = key in (-1, save_key)
            return continue_streaming

        return save


def collect_data_for_calibration(camera, save_folder, save_key=ord(' '), extention='png'):
    stream(camera,
        [StreamCallbacks.get_save_by_key_callback(save_folder,
            save_key=save_key, extention=extention),
        StreamCallbacks.flip],
        window_name='collect data for calibration')

