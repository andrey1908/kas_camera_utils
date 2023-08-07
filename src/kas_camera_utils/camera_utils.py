import numpy as np
import cv2
from time import time, sleep
import os.path as osp
import glob


def get_image_resolution(camera):
    image = camera.read()
    w = image.shape[1]
    h = image.shape[0]
    return w, h


def get_fps(camera, duration=5):
    start = time()
    now = start
    num_images = 0
    while now - start < duration:
        camera.read()
        num_images += 1
        now = time()
    fps = num_images / duration
    return fps


def stream(camera, callbacks=None, pause_key=ord('p'), step_key=ord('s'),
        window_name="stream"):
    if callbacks is None:
        callbacks = list()
    elif callable(callbacks):
        callbacks = [callbacks]
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, (600, 300))

    key = -1
    continue_streaming = True
    paused = False
    step = False
    while continue_streaming:
        if not paused or step:
            camera_frames = camera()
            if len(camera_frames) == 0:
                print(f"Could not read camera frames\n")
                sleep(0.5)
                continue
            for name, frame in camera_frames.items():
                if frame is None or frame.size == 0:
                    print(f"Could not read camera frame '{name}'\n")
                    sleep(0.5)
                    continue

            if "image" not in camera_frames:
                image_shape = next(iter(camera_frames.values())).shape[:2] + (3,)
                image = np.zeros(image_shape, dtype=np.uint8)
                camera_frames["image"] = image

            continue_streaming_is_set_by_callback = False
            for callback in callbacks:
                # camera_frames always contain "image" frame
                callback_continue_streaming = callback(key, **camera_frames)
                if callback_continue_streaming is not None:
                    continue_streaming = (continue_streaming and callback_continue_streaming)
                    continue_streaming_is_set_by_callback = True

            image = camera_frames["image"]
            cv2.imshow(window_name, image)
            step = False

        if not paused:
            key = cv2.waitKey(1)
        else:
            key = cv2.waitKey(10)  # to prevent busy wait when paused
        if not continue_streaming_is_set_by_callback:
            continue_streaming = (key == -1) or (key == pause_key) or (key == step_key)

        if key == pause_key:
            paused = not paused
        if key == step_key:
            step = True

    cv2.destroyWindow(window_name)


class ImagesSavePathsGenerator:
    def __init__(self, save_folder, extention='jpg', continue_saving=False,
            start_from=0):
        self.save_folder = osp.expanduser(save_folder)
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
    def flip(key, image, **kwargs):
        cv2.flip(image, 1, dst=image)

    def get_save_every_k_callback(images_save_paths_generator, save_every_k=1):
        def save(key, image, **kwargs):
            if save.counter % save_every_k == 0:
                image_path = images_save_paths_generator()
                saved = cv2.imwrite(image_path, image)
                if saved:
                    print(f"Saved image {osp.basename(image_path)}")
                else:
                    print("Could not save image")
            save.counter += 1
        save.counter = 0
        return save

    def get_save_by_key_callback(images_save_paths_generator, save_key=ord(' ')):
        def save(key, image, **kwargs):
            if key == save_key:
                image_path = images_save_paths_generator()
                saved = cv2.imwrite(image_path, image)
                if saved:
                    print(f"Saved image {osp.basename(image_path)}")
                else:
                    print("Could not save image")
            if key == save_key:
                continue_streaming = True
            else:
                continue_streaming = None
            return continue_streaming
        return save


def collect_data_for_calibration(camera, save_folder, save_key=ord(' '), extention='png'):
    stream(camera,
        [StreamCallbacks.get_save_by_key_callback(save_folder,
            save_key=save_key, extention=extention),
        StreamCallbacks.flip],
        window_name='collect data for calibration')

