import pyrealsense2 as rs
import numpy as np


class RealsenseCamera:
    def __init__(self, enable_image=True, enable_depth=False, align_depth2image=True):
        self.enable_image = enable_image
        self.enable_depth = enable_depth
        self.align_depth2image = align_depth2image

        self.config = rs.config()
        if self.enable_image:
            self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        if self.enable_depth:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = None
        if self.enable_depth and self.align_depth2image:
            self.align = rs.align(rs.stream.color)
        self.pipeline = rs.pipeline()

        self.start()
        profile = self.pipeline.get_active_profile()
        if self.enable_image:
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            self.K = np.eye(3)
            self.K[0, 0] = intrinsics.fx
            self.K[1, 1] = intrinsics.fy
            self.K[0, 2] = intrinsics.ppx
            self.K[1, 2] = intrinsics.ppy
            self.D = np.array(intrinsics.coeffs)
        if self.enable_depth:
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
        self.stop()

    def start(self):
        self.pipeline.start(self.config)

    def stop(self):
        self.pipeline.stop()

    def __call__(self):
        frames = dict()
        frames_list = self.read(read_image=True, read_depth=True)
        for frame, name in zip(frames_list, ("image", "depth")):
            if frame is not None:
                frames[name] = frame
        return frames

    def read(self, read_image=True, read_depth=False):
        frames = self.pipeline.wait_for_frames()
        if self.align:
            frames = self.align.process(frames)

        ret = list()
        if read_image:
            if self.enable_image:
                color_frame = frames.get_color_frame()
                image = np.array(color_frame.get_data())
                ret.append(image)
            else:
                ret.append(None)

        if read_depth:
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                depth = np.array(depth_frame.get_data())
                ret.append(depth)
            else:
                ret.append(None)

        if len(ret) == 1:
            return ret[0]
        return ret

    def get_exposure(self):
        profile = self.pipeline.get_active_profile()
        color_sensor = profile.get_device().first_color_sensor()
        exposure = color_sensor.get_option(rs.option.exposure)
        return exposure

    def set_exposure(self, exposure):
        profile = self.pipeline.get_active_profile()
        color_sensor = profile.get_device().first_color_sensor()
        color_sensor.set_option(rs.option.exposure, exposure)
