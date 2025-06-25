#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

import numpy as np
import struct
import depthai as dai
import cv2


class OakDPointCloudNode(Node):
    def __init__(self):
        super().__init__('oakd_pointcloud_node')
        self.bridge = CvBridge()

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/oakd/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oakd/depth/image_raw', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/oakd/pointcloud', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/oakd/rgb/camera_info', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Intrinsics (Replace with real calibration if available)
        self.fx = 870.0
        self.fy = 870.0
        self.cx = 640.0
        self.cy = 360.0
        self.width = 640
        self.height = 360

        self.pipeline = self._create_pipeline()
        self.device = dai.Device(self.pipeline)
        self.depth_q = self.device.getOutputQueue("depth", maxSize=4, blocking=False)
        self.rgb_q = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)

        # RGB + Depth @ 15 Hz
        self.timer_fast = self.create_timer(1.0 / 15.0, self.publish_rgb_depth)

        # PointCloud2 @ 2 Hz
        self.timer_slow = self.create_timer(0.5, self.publish_pointcloud)

        # Last received frames
        self.latest_rgb = None
        self.latest_depth = None

    def _create_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline

    def publish_rgb_depth(self):
        # Try get new frames
        if self.rgb_q.has():
            self.latest_rgb = self.rgb_q.get().getCvFrame()
            self.latest_rgb = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2RGB)
        if self.depth_q.has():
            self.latest_depth = self.depth_q.get().getFrame()

        if self.latest_rgb is None or self.latest_depth is None:
            return

        stamp = self.get_clock().now().to_msg()

        # Publish TF
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "oakd_link"
        tf.child_frame_id = "oakd_rgb_camera_frame"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

        # Publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(self.latest_rgb, encoding="rgb8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "oakd_rgb_camera_frame"
        self.rgb_pub.publish(rgb_msg)

        # Publish Depth image (mono16)
        depth_msg = self.bridge.cv2_to_imgmsg(self.latest_depth, encoding="mono16")
        depth_msg.header = rgb_msg.header
        self.depth_pub.publish(depth_msg)

        # CameraInfo
        cam_info = CameraInfo()
        cam_info.header = rgb_msg.header
        cam_info.width = self.width
        cam_info.height = self.height
        cam_info.k = [float(self.fx), 0.0, float(self.cx),
                      0.0, float(self.fy), float(self.cy),
                      0.0, 0.0, 1.0]
        cam_info.p = [float(self.fx), 0.0, float(self.cx), 0.0,
                      0.0, float(self.fy), float(self.cy), 0.0,
                      0.0, 0.0, 1.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        cam_info.d = [0.0] * 5
        self.rgb_info_pub.publish(cam_info)

    def publish_pointcloud(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        stamp = self.get_clock().now().to_msg()
        height, width = self.latest_depth.shape
        points = []

        stride = 4  # Every 4th pixel in x and y

        for v in range(0, height, stride):
            for u in range(0, width, stride):
                Z = self.latest_depth[v, u] / 1000.0
                if Z <= 0.1 or Z > 10.0:
                    continue
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy
                R, G, B = self.latest_rgb[v, u]
                rgb = (int(R) << 16) | (int(G) << 8) | int(B)
                points.append(struct.pack('fffI', X, Y, Z, rgb))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = stamp
        cloud_msg.header.frame_id = "oakd_rgb_camera_frame"
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = 16 * len(points)
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(points)

        self.pc_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakDPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
