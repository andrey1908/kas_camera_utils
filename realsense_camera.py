import pyrealsense2 as rs
import numpy as np


class RealsenseCamera:
    def __init__(self):
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline = rs.pipeline()

    def start(self):
        self.pipeline.start(self.config)

    def stop(self):
        self.pipeline.stop()

    def __call__(self):
        frames = self.pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None
        image = np.array(color.get_data())
        return image

    def get_exposure(self):
        prof = self.pipeline.get_active_profile()
        s = prof.get_device().query_sensors()[1]
        exposure = s.get_option(rs.option.exposure)
        return exposure

    def set_exposure(self, exposure):
        prof = self.pipeline.get_active_profile()
        s = prof.get_device().query_sensors()[1]
        s.set_option(rs.option.exposure, exposure)
