import cv2


class CvCamera:
    def __init__(self, source, auto_repeat=False):
        self.source = source
        self.auto_repeat = auto_repeat

        self.video = cv2.VideoCapture()

        if isinstance(self.source, str):
            self.video.open(self.source)
            if not self.video.isOpened():
                raise FileNotFoundError(f"Video file not found: {self.source}")

    def is_active(self):
        return self.video.isOpened()

    def start(self):
        if not isinstance(self.source, str):
            self.video.open(self.source)
            if not self.video.isOpened():
                raise RuntimeError(f"Can't open video device {self.source}")

    def try_start(self):
        if not self.is_active():
            self.start()

    def stop(self):
        if not isinstance(self.source, str):
            self.video.release()

    def __call__(self):
        image = self.read()
        return {"image": image}

    def read(self):
        ret, image = self.video.read()
        if not ret:
            if isinstance(self.source, str):
                if self.auto_repeat:
                    self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, image = self.video.read()
                    if not ret:
                        raise RuntimeError("Could not restart video")
                else:
                    raise StopIteration()
            else:
                raise RuntimeError(f"Could not read from video device {self.source}")
        return image
