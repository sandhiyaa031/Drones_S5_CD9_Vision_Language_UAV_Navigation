#!/usr/bin/env python3

import base64
import json
import os
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


class VLMDetector(Node):

    def __init__(self):
        super().__init__('vlm_detector')

        # Camera publisher (/camera/image_raw) is RELIABLE,
        # so the subscriber must use a compatible QoS profile.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )

        self.detection_pub = self.create_publisher(
            PoseStamped,
            '/survivor/detection',
            10
        )

        self.busy = False

        self.get_logger().info('========================================')
        self.get_logger().info(' VLM SURVIVOR DETECTOR')
        self.get_logger().info(' Camera QoS: RELIABLE')
        self.get_logger().info(' Waiting for camera frames')
        self.get_logger().info('========================================')

    def image_callback(self, msg):

        if self.busy:
            return

        self.busy = True

        try:
            self.get_logger().info(
                f'Camera frame received: '
                f'{msg.width}x{msg.height}, encoding={msg.encoding}',
                throttle_duration_sec=5.0
            )

            image = self.image_to_base64(msg)

            result = self.query_vlm(image)

            if result is not None:
                self.publish_detection(
                    result['x'],
                    result['y']
                )

        except Exception as e:
            self.get_logger().error(
                f'VLM error: {e}'
            )

        finally:
            self.busy = False

    def image_to_base64(self, msg):

        # Camera must provide RGB8 or BGR8 frames.
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise RuntimeError(
                f'Unsupported camera encoding: {msg.encoding}'
            )

        import cv2
        import numpy as np

        image = np.frombuffer(
            msg.data,
            dtype=np.uint8
        ).reshape(
            msg.height,
            msg.width,
            3
        )

        success, encoded = cv2.imencode(
            '.jpg',
            image
        )

        if not success:
            raise RuntimeError(
                'Could not encode camera image'
            )

        return base64.b64encode(
            encoded.tobytes()
        ).decode('utf-8')

    def query_vlm(self, image_b64):

        endpoint = os.environ.get('VLM_ENDPOINT')

        if not endpoint:
            self.get_logger().warn(
                'VLM_ENDPOINT is not configured.'
            )
            return None

        payload = {
            'image': image_b64,
            'prompt': (
                'You are a drone search-and-rescue vision system. '
                'Look for a human survivor in this image. '
                'Return JSON only. '
                'If a survivor is visible return: '
                '{"found":true,"x":NUMBER,"y":NUMBER}. '
                'If no survivor is visible return: '
                '{"found":false}. '
                'x and y are normalized image coordinates from '
                '0.0 to 1.0.'
            )
        }

        request = urllib.request.Request(
            endpoint,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Content-Type': 'application/json'
            },
            method='POST'
        )

        with urllib.request.urlopen(
            request,
            timeout=10
        ) as response:

            result = json.loads(
                response.read().decode('utf-8')
            )

        if not result.get('found', False):
            return None

        return {
            'x': float(result['x']),
            'y': float(result['y'])
        }

    def publish_detection(self, image_x, image_y):

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        # Temporary normalized camera coordinates.
        # The camera-to-local-frame transform will be added
        # once the simulated camera is available.
        msg.pose.position.x = image_x
        msg.pose.position.y = image_y
        msg.pose.position.z = -2.0

        msg.pose.orientation.w = 1.0

        self.detection_pub.publish(msg)

        self.get_logger().warn(
            f'VLM SURVIVOR DETECTED: '
            f'image=({image_x:.3f}, {image_y:.3f})'
        )


def main(args=None):

    rclpy.init(args=args)

    node = VLMDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
