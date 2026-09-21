#!/usr/bin/env python3
"""Mark CARLA ground-truth localization initialized once odometry is live."""

import rclpy
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CarlaLocalizationReady(Node):
    def __init__(self):
        super().__init__("carla_localization_ready")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            qos,
        )
        self.latest_stamp = None
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.on_odometry,
            10,
        )
        self.create_timer(1.0, self.publish_ready)

    def on_odometry(self, message):
        self.latest_stamp = message.header.stamp

    def publish_ready(self):
        if self.latest_stamp is None:
            return
        message = LocalizationInitializationState()
        message.stamp = self.latest_stamp
        message.state = LocalizationInitializationState.INITIALIZED
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = CarlaLocalizationReady()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
