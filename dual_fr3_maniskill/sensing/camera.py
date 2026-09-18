"""Dedicated aligned RGB-D camera; no cable geometry/segmentation is exported."""
from dataclasses import dataclass
import time
import numpy as np


@dataclass
class CameraSettings:
    width: int = 1280
    height: int = 960
    fovy: float = 0.82
    near: float = 0.02
    far: float = 5.0
    eye: tuple = (0.55, 0.65, 1.45)
    target: tuple = (0.52, 0.65, 0.04)
    up: tuple = (0.0, 1.0, 0.0)
    frequency: float = 2.0
    frame: str = "cable_camera_optical_frame"

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or not 0 < self.fovy < np.pi:
            raise ValueError("Invalid perception camera resolution/fovy")
        if not 0 < self.near < self.far or not np.isfinite(self.frequency) or self.frequency <= 0:
            raise ValueError("Invalid perception camera clipping/frequency")
        eye, target, up = (np.asarray(v, dtype=float) for v in (self.eye, self.target, self.up))
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in (eye, target, up)):
            raise ValueError("camera eye/target/up must be finite three-vectors")
        if np.linalg.norm(np.cross(target-eye, up)) < 1e-8:
            raise ValueError("camera target/eye/up do not define a viewing orientation")


def decode_textures(rgba, position, model_matrix):
    """SAPIEN Position is OpenGL xyz + z-buffer, NOT radial range."""
    rgb = np.ascontiguousarray(np.clip(rgba[..., :3] * 255, 0, 255).astype(np.uint8))
    depth = np.array(-position[..., 2], dtype=np.float32)
    valid = (position[..., 3] < 1) & np.isfinite(position).all(axis=-1) & (depth > 0)
    depth[~valid] = np.nan
    # optical: right/down/forward; OpenGL: right/up/backward.
    world_from_optical = np.asarray(model_matrix, dtype=float) @ np.diag([1., -1., -1., 1.])
    return rgb, depth, world_from_optical


class PerceptionCamera:
    def __init__(self, env, settings):
        from mani_skill2.utils.sapien_utils import look_at
        self.env, self.settings = env, settings
        c = settings
        self.camera = env._scene.add_camera("cable_perception", c.width, c.height, c.fovy, c.near, c.far)
        self.camera.set_local_pose(look_at(c.eye, c.target, up=c.up))

    def capture(self):
        # Must call ENV hook: rope actors live in a separate physics scene and
        # their render poses are synchronized by UsbCableEnv.update_render().
        self.env.update_render()
        self.camera.take_picture()
        rgb, depth, transform = decode_textures(self.camera.get_float_texture("Color"),
            self.camera.get_float_texture("Position"), self.camera.get_model_matrix())
        k = self.camera.get_intrinsic_matrix().copy()
        # SAPIEN raster samples are (u+.5,v+.5); ROS/OpenCV integer coordinates
        # denote pixel centres. Express the same rays with that convention.
        k[0, 2] -= .5
        k[1, 2] -= .5
        return rgb, depth, k, transform


class CameraPublisher:
    def __init__(self, node):
        from sensor_msgs.msg import Image, CameraInfo
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from tf2_ros import StaticTransformBroadcaster
        self.node = node
        defaults = CameraSettings()
        values = {}
        for name, value in vars(defaults).items():
            values[name] = node.declare_parameter("camera_" + name, list(value) if isinstance(value, tuple) else value).value
        self.settings = CameraSettings(**values)
        self.camera = PerceptionCamera(node.sim.env, self.settings)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.rgb = node.create_publisher(Image, "/cable_camera/color/image_raw", qos)
        self.depth = node.create_publisher(Image, "/cable_camera/aligned_depth/image_raw", qos)
        self.info = node.create_publisher(CameraInfo, "/cable_camera/color/camera_info", qos)
        self.tf = StaticTransformBroadcaster(node)
        self.next_capture = 0.
        self.tf_sent = False

    def publish_if_due(self, stamp):
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import Header
        from geometry_msgs.msg import TransformStamped
        from transforms3d.quaternions import mat2quat
        now = time.monotonic()
        if now < self.next_capture:
            return
        self.next_capture = now + 1 / self.settings.frequency
        rgb, depth, k, transform = self.camera.capture()
        header = Header(stamp=stamp, frame_id=self.settings.frame)
        if not self.tf_sent:
            tf = TransformStamped(header=Header(stamp=stamp, frame_id="world"), child_frame_id=self.settings.frame)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = transform[:3, 3].tolist()
            q = mat2quat(transform[:3, :3])
            tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = q.tolist()
            self.tf.sendTransform(tf)
            self.tf_sent = True
        for data, encoding, publisher in ((rgb, "rgb8", self.rgb), (depth, "32FC1", self.depth)):
            msg = Image(header=header, height=data.shape[0], width=data.shape[1], encoding=encoding,
                        is_bigendian=False, step=data.strides[0], data=data.tobytes())
            publisher.publish(msg)
        info = CameraInfo(header=header, height=depth.shape[0], width=depth.shape[1],
                          distortion_model="plumb_bob", d=[0.]*5, k=k.ravel().tolist(),
                          r=np.eye(3).ravel().tolist(), p=np.column_stack((k, np.zeros(3))).ravel().tolist())
        self.info.publish(info)
