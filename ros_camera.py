import rospy
import rostopic
import rosbag
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class RosbagTopicReader:
    def __init__(self, bag, topic, auto_repeat=False):
        self.bag = bag
        self.topic = topic
        self.auto_repeat = auto_repeat

        self.reader = (msg for topic, msg, t in self.bag.read_messages(self.topic))
        self.next_msg = None

        self.repeated = False

    def __iter__(self):
        return self

    def __next__(self):
        if self.next_msg is not None:
            msg = self.next_msg
            self.next_msg = None
        else:
            self.repeated = False
            try:
                msg = next(self.reader)
            except StopIteration:
                if self.auto_repeat:
                    self.reset()
                    msg = next(self.reader)
                    self.repeated = True
                else:
                    raise
        return msg

    def reset(self):
        self.reader = (msg for topic, msg, t in self.bag.read_messages(self.topic))
        self.next_msg = None
        self.repeated = False

    def peek(self):
        if self.next_msg is None:
            self.next_msg = next(self)
        return self.next_msg


class RosCamera:
    def __init__(self, camera_info_topic=None, image_topic=None,
            depth_info_topic=None, depth_topic=None, rosbag_file=None,
            exact_sync=True):
        self.camera_info_topic = camera_info_topic
        self.image_topic = image_topic
        self.depth_info_topic = depth_info_topic
        self.depth_topic = depth_topic
        self.rosbag_file = rosbag_file
        self.exact_sync = exact_sync

        assert not (self.exact_sync and rosbag_file is None)

        self.enable_image = bool(self.camera_info_topic and self.image_topic)
        self.enable_depth = bool(self.depth_info_topic and self.depth_topic)

        if self.rosbag_file is not None:
            self.bag = rosbag.Bag(self.rosbag_file, 'r')
            if self.enable_image:
                camera_info_reader = RosbagTopicReader(self.bag, self.camera_info_topic)
                self.image_reader = RosbagTopicReader(
                    self.bag, self.image_topic, auto_repeat=True)
            if self.enable_depth:
                depth_info_reader = RosbagTopicReader(self.bag, self.depth_info_topic)
                self.depth_reader = RosbagTopicReader(
                    self.bag, self.depth_topic, auto_repeat=True)
        else:
            rospy.init_node("ros_camera", anonymous=True)
            if self.enable_image:
                camera_info_reader = self._get_topic_reader(self.camera_info_topic)
                self.image_reader = self._get_topic_reader(self.image_topic)
            if self.enable_depth:
                depth_info_reader = self._get_topic_reader(self.depth_info_topic)
                self.depth_reader = self._get_topic_reader(self.depth_topic)

        if self.enable_image:
            camera_info_msg = next(camera_info_reader)
            self.K = camera_info_msg.K
            self.D = camera_info_msg.D
            self.K = np.array(self.K).reshape(3, 3)
            self.D = np.array(self.D)
        if self.enable_depth:
            depth_info_msg = next(depth_info_reader)
            self.depth_K = depth_info_msg.K
            self.depth_D = depth_info_msg.D
            self.depth_K = np.array(self.depth_K).reshape(3, 3)
            self.depth_D = np.array(self.depth_D)
        self.bridge = CvBridge()

    def __del__(self):
        if hasattr(self, "bag"):
            self.bag.close()

    def _get_topic_reader(self, topic):
        topic_type, _, _ = rostopic.get_topic_class(topic)
        while True:
            msg = rospy.wait_for_message(topic, topic_type)
            yield msg

    def _sync_rosbag_topic_readers(self):
        if self.enable_image and self.enable_depth:
            next_image_stamp = self.image_reader.peek().header.stamp
            next_depth_stamp = self.depth_reader.peek().header.stamp
            while next_image_stamp != next_depth_stamp:
                if next_image_stamp < next_depth_stamp:
                    next(self.image_reader)
                    if self.image_reader.repeated:
                        self.depth_reader.reset()
                        next_depth_stamp = self.depth_reader.peek().header.stamp
                    next_image_stamp = self.image_reader.peek().header.stamp
                elif next_image_stamp > next_depth_stamp:
                    next(self.depth_reader)
                    if self.depth_reader.repeated:
                        self.image_reader.reset()
                        next_image_stamp = self.image_reader.peek().header.stamp
                    next_depth_stamp = self.depth_reader.peek().header.stamp

    def start(self):
        pass

    def stop(self):
        pass

    def __call__(self):
        frames = dict()
        frames_list = self.read(read_image=True, read_depth=True)
        for frame, name in zip(frames_list, ("image", "depth")):
            if frame is not None:
                frames[name] = frame
        return frames
    
    def read(self, read_image=True, read_depth=False):
        import time
        # time.sleep(0.1)
        self.last_image_stamp = None
        self.last_depth_stamp = None

        if self.exact_sync:
            self._sync_rosbag_topic_readers()

        ret = list()
        if read_image:
            if self.enable_image:
                image_msg = next(self.image_reader)
                if image_msg._type == "sensor_msgs/Image":
                    image = self.bridge.imgmsg_to_cv2(
                        image_msg, desired_encoding='passthrough')
                elif image_msg._type == "sensor_msgs/CompressedImage":
                    image = self.bridge.compressed_imgmsg_to_cv2(
                        image_msg, desired_encoding='passthrough')
                else:
                    raise RuntimeError
                self.last_image_stamp = image_msg.header.stamp
                ret.append(image)
            else:
                ret.append(None)

        if read_depth:
            if self.enable_depth:
                depth_msg = next(self.depth_reader)
                if depth_msg._type == "sensor_msgs/Image":
                    depth = self.bridge.imgmsg_to_cv2(
                        depth_msg, desired_encoding='passthrough')
                elif depth_msg._type == "sensor_msgs/CompressedImage":
                    depth = self.bridge.compressed_imgmsg_to_cv2(
                        depth_msg, desired_encoding='passthrough')
                else:
                    raise RuntimeError
                self.last_depth_stamp = depth_msg.header.stamp
                ret.append(depth)
            else:
                ret.append(None)
        
        if len(ret) == 1:
            return ret[0]
        return ret
