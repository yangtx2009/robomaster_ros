import threading
import queue
# import time

from cv_bridge import CvBridge
import numpy as np
import yaml

import rclpy
import robomaster.media
import robomaster_msgs.msg
import sensor_msgs.msg
import rcl_interfaces.msg
import rclpy.duration
import rclpy.time
# from rclpy.clock import ClockType

from typing import TYPE_CHECKING, Optional, Dict, Any
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


# taking 2^14 as maximal amplitude (verified), not 2^15 as could be expected for 16 bit audio
MAX_LEVEL = 10 * 2 * 14 * np.log10(2)


def sound_level(samples: np.ndarray) -> float:
    rms = np.mean(np.power(samples.astype(float), 2))
    return 10 * np.log10(rms) - MAX_LEVEL


def camera_info_from_calibration(calibration: Dict[str, Any], frame_id: str
                                 ) -> sensor_msgs.msg.CameraInfo:
    msg = sensor_msgs.msg.CameraInfo()
    msg.header.frame_id = frame_id
    msg.height = calibration['image_height']
    msg.width = calibration['image_width']
    msg.distortion_model = calibration['distortion_model']
    msg.d = calibration['distortion_coefficients']['data']
    msg.k = calibration['camera_matrix']['data']
    msg.r = calibration['rectification_matrix']['data']
    msg.p = calibration['projection_matrix']['data']
    return msg


class Camera(robomaster.media.LiveView, Module):  # type: ignore

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self._video_streaming = False
        robomaster.media.LiveView.__init__(self, robot)
        self.logger = node.get_logger()
        self.api = robot.camera
        self.api._liveview = self
        self.bridge = CvBridge()
        self._message_queue: "queue.Queue[sensor_msgs.msg.Image]" = queue.Queue(1)
        self.clock = node.get_clock()
        self.video: bool = node.declare_parameter("camera.video.enabled", True).value
        self.next_capture_time = self.clock.now()

        if self.video:
            self.publish_raw_video: bool = node.declare_parameter("camera.video.raw", True).value
            self.publish_h264_video: bool = node.declare_parameter("camera.video.h264", True).value
            protocol: str = node.declare_parameter("camera.video.protocol", "tcp").value
            height: int = node.declare_parameter("camera.video.resolution", 360).value
            rate: float = node.declare_parameter("camera.video.rate", -1.0).value
            self.min_video_period: Optional[rclpy.duration.Duration]
            if rate > 0:
                self.logger.info(f"[Camera] fps limited to {rate:.1f}")
                self.min_video_period = rclpy.duration.Duration(nanoseconds=1e9 / rate)
            else:
                self.min_video_period = None
            node.create_subscription(
                robomaster_msgs.msg.CameraConfig, 'camera/config',
                self.has_updated_camera_config, 1)
            if protocol in ('udp', 'tcp'):
                robomaster.config.ep_conf.video_stream_proto = protocol
            if height not in (360, 540, 720):
                node.get_logger().error("Resolution not supported. Should be 360, 540, or 720")
                return
            if self.publish_raw_video:
                self.raw_video_pub = node.create_publisher(
                    sensor_msgs.msg.Image, 'camera/image_color', 1)
            if self.publish_h264_video:
                self.h264_video_pub = node.create_publisher(
                    robomaster_msgs.msg.H264Packet, 'camera/image_h264', 1)
            self.frame_id = node.tf_frame('camera_optical_link')
            calibration_path = node.declare_parameter("camera.video.calibration_file", '').value
            self.publish_camera_info = False
            if calibration_path:
                try:
                    with open(calibration_path, 'r') as f:
                        calibration = yaml.load(f, yaml.SafeLoader)
                        if calibration:
                            self.camera_info_msg = camera_info_from_calibration(
                                calibration, self.frame_id)
                            self.camera_info_pub = node.create_publisher(
                                sensor_msgs.msg.CameraInfo, 'camera/camera_info', 1)
                            self.publish_camera_info = True
                except FileNotFoundError:
                    self.logger.warn(f"[Camera] Calibration file not found at {calibration_path}")
            self.logger.info(f"[Camera] Start video stream with resolution {height}p")
            self.api.start_video_stream(display=False, resolution=f"{height}p")
        self.audio: bool = node.declare_parameter("camera.audio.enabled", True).value
        if self.audio:
            self.publish_raw_audio: bool = node.declare_parameter("camera.audio.raw", True).value
            self.publish_opus_audio: bool = node.declare_parameter("camera.audio.opus", True).value
            if self.publish_raw_audio:
                self.audio_raw_pub = node.create_publisher(
                    robomaster_msgs.msg.AudioData, 'camera/audio_raw', 1)
            if self.publish_opus_audio:
                self.audio_opus_pub = node.create_publisher(
                    robomaster_msgs.msg.AudioOpus, 'camera/audio_opus', 1)
            self.audio_level_pub = node.create_publisher(
                robomaster_msgs.msg.AudioLevel, 'camera/audio_level', 1)
            self.api.start_audio_stream()

        # TODO(Jerome): disabled beacuse setting the resolution is not working ... why?
        # node.add_on_set_parameters_callback(self.set_params_cb)

    def set_params_cb(self, params: Any) -> rcl_interfaces.msg.SetParametersResult:
        success = True
        for param in params:
            if param.name == 'camera.video.resolution' and self.video:
                if param.value in (360, 540, 720):
                    self.logger.info(f"[Camera] Will set resolution to {param.value}")
                    self.api.stop_video_stream()
                    self.api.start_video_stream(display=False, resolution=f"{param.value}p")
                    self.logger.info(f"[Camera] Has set resolution to {param.value}")
                else:
                    success = False
                    self.logger.warning(
                        f"[Camera] Resolution {param.value} not supported. "
                        "Should be 360, 540, or 720")
        return rcl_interfaces.msg.SetParametersResult(successful=success)

    def start_video_stream(self, display: bool = False, addr: Optional[str] = None,
                           ip_proto: str = "tcp") -> None:
        super().start_video_stream(display=display, addr=addr, ip_proto=ip_proto)
        self._video_publisher_thread = threading.Thread(target=self._video_publisher_task)
        self._video_publisher_thread.start()

    def stop_video_stream(self) -> None:
        # self.logger.info("will stop_video_stream")
        super().stop_video_stream()
        self._message_queue.queue.clear()
        self._video_publisher_thread.join()
        # self.logger.info("has stopped_video_stream")

    # TODO(jerome) Check because we are overwriting a method here!
    # def stop(self) -> None:
    #     self.logger.info("[Camera] Will stop")
    #     if self.video:
    #         self.stop_video_stream()
    #         self.api.stop_video_stream()
    #     if self.audio:
    #         self.stop_audio_stream()
    #         self.api.stop_audio_stream()
    #     self.logger.info("[Camera] Has stopped")

    # TODO(jerome): maybe better a parameter??
    def has_updated_camera_config(self, msg: robomaster_msgs.msg.CameraConfig) -> None:
        self.api._set_zoom(msg.zoom)

    # DONE(jerome): make it work even if the publisher (and/or decoder are slow)
    # TODO(Jerome): why not switching back to the super class method?
    def _video_decoder_task(self) -> None:
        self._video_streaming = True
        seq = 0
        while self._video_streaming:
            # blocking
            data = self._video_stream_conn._sock_queue.get()
            while True:
                # empty the queue
                try:
                    additional_data = self._video_stream_conn._sock_queue.get(False)
                    if additional_data:
                        data += additional_data
                except queue.Empty:
                    break
            if data:
                capture_time = self.clock.now()
                if self.publish_h264_video:
                    msg = robomaster_msgs.msg.H264Packet()
                    msg.header.stamp = capture_time.to_msg()
                    msg.header.frame_id = self.frame_id
                    msg.data = data
                    msg.seq = seq
                    seq += 1
                    self.h264_video_pub.publish(msg)
                if self.publish_raw_video:
                    frames = self._h264_decode(data)
                    if not frames:
                        continue
                    self._video_frame_count += len(frames)
                    # self.logger.info(f"Got frames {len(frames)}")
                    if self.min_video_period:
                        if capture_time < self.next_capture_time:
                            continue
                    for frame in frames[-1:]:
                        # self.logger.info(f"SHAPE {frame.shape}")
                        i_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                        # + rclpy.duration.Duration(nanoseconds=3e8)
                        i_msg.header.stamp = capture_time.to_msg()
                        i_msg.header.frame_id = self.frame_id
                        try:
                            self._message_queue.put(i_msg, block=False)
                        except queue.Full:
                            pass
                        else:
                            if self.min_video_period:
                                self.next_capture_time += self.min_video_period

    def _video_publisher_task(self) -> None:
        while self._video_streaming:
            try:
                msg = self._message_queue.get(timeout=1)
            except queue.Empty:
                continue
            # self.logger.info("Got msg")
            try:
                self.raw_video_pub.publish(msg)
            except rclpy._rclpy_pybind11.RCLError:  # type: ignore
                return
            if self.publish_camera_info:
                self.camera_info_msg.header.stamp = msg.header.stamp
                self.camera_info_pub.publish(self.camera_info_msg)
            # self.logger.info("Sent msg")

    def start_audio_stream(self, addr: Optional[str] = None,
                           ip_proto: str = "tcp") -> None:
        super().start_audio_stream(addr=addr, ip_proto=ip_proto)
        self._audio_publisher_thread = threading.Thread(target=self._audio_publisher_task)
        self._audio_publisher_thread.start()

    def stop_audio_stream(self) -> None:
        super().stop_audio_stream()
        self._audio_publisher_thread = threading.Thread(target=self._audio_publisher_task)
        self._audio_publisher_thread.start()

    def _audio_decoder_task(self) -> None:
        self._audio_streaming = True
        seq = 0
        while self._audio_streaming:
            data = self._audio_stream_conn.read_buf()
            if data and self.publish_opus_audio:
                msg = robomaster_msgs.msg.AudioOpus(buffer=data)
                msg.header.stamp = self.clock.now().to_msg()
                msg.seq = seq
                seq += 1
                self.audio_opus_pub.publish(msg)
            if data:
                frame = self._audio_decoder.decode(data)
                if frame:
                    try:
                        self._audio_frame_count += 1
                        self._audio_frame_queue.put(frame, block=False)
                    except queue.Full:
                        pass

    def _audio_publisher_task(self) -> None:
        while self._audio_streaming:
            try:
                frame = self._audio_frame_queue.get(timeout=1)
            except queue.Empty:
                continue
            frame = np.frombuffer(frame, np.int16)
            if self.publish_raw_audio:
                msg = robomaster_msgs.msg.AudioData(data=frame)
                msg.header.stamp = self.clock.now().to_msg()
                self.audio_raw_pub.publish(msg)
            level_msg = robomaster_msgs.msg.AudioLevel(level=sound_level(frame))
            self.audio_level_pub.publish(level_msg)
