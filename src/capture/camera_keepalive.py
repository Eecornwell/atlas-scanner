#!/usr/bin/env python3
# Camera stream keepalive — subscribes to camera topics to prevent the
# Insta360 SDK from closing the stream after ~40s of no consumers.
import sys
import rclpy
from sensor_msgs.msg import CompressedImage, Image

camera_mode = sys.argv[1] if len(sys.argv) > 1 else 'dual_fisheye'

ctx = rclpy.Context()
rclpy.init(context=ctx)
node = rclpy.create_node('camera_keepalive', context=ctx)
# Always subscribe to compressed (keeps the driver streaming)
node.create_subscription(CompressedImage, '/dual_fisheye/image/compressed', lambda m: None, 10)
# In single_fisheye mode also subscribe to decoded topic so the decoder stays active
if camera_mode == 'single_fisheye':
    node.create_subscription(Image, '/dual_fisheye/image', lambda m: None, 10)
executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
executor.add_node(node)
try:
    executor.spin()
except Exception:
    pass
finally:
    try:
        rclpy.shutdown(context=ctx)
    except Exception:
        pass
