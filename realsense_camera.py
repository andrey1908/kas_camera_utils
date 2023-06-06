import pyrealsense2 as rs
import numpy as np


class RealsenseCamera:
    def __init__(self, enable_color=True, enable_depth=False, align_depth2color=True):
        self.enable_color = enable_color
        self.enable_depth = enable_depth
        self.align_depth2color = align_depth2color

        self.config = rs.config()
        if self.enable_color:
            self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        if self.enable_depth:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = None
        if self.enable_depth and self.align_depth2color:
            self.align = rs.align(rs.stream.color)
        self.pipeline = rs.pipeline()

    def start(self):
        self.pipeline.start(self.config)

    def stop(self):
        self.pipeline.stop()

    def __call__(self):
        image = self.read()
        return image

    def read(self, read_color=True, read_depth=False):
        frames = self.pipeline.wait_for_frames()
        if self.align:
            frames = self.align.process(frames)

        ret = list()
        if read_color:
            color_frame = frames.get_color_frame()
            if color_frame:
                image = np.array(color_frame.get_data())
                ret.append(image)
            else:
                ret.append(None)

        if read_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
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

    def get_depth_scale(self):
        profile = self.pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        return depth_scale

    def get_intrinsics(self):
        profile = self.pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        return intrinsics

    def get_K(self):
        intrinsics = self.get_intrinsics()
        K = np.eye(3)
        K[0, 0] = intrinsics.fx
        K[1, 1] = intrinsics.fy
        K[0, 2] = intrinsics.ppx
        K[1, 2] = intrinsics.ppy
        return K

    def get_D(self):
        intrinsics = self.get_intrinsics()
        D = np.array(intrinsics.coeffs)
        return D
