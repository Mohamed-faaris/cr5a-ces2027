#!/usr/bin/env python3
"""Filter ros2_control's synthetic mimic-joint state before publishing to MoveIt/RSP."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMimicRelay(Node):
    def __init__(self):
        super().__init__("joint_state_mimic_relay")
        self.declare_parameter("input_topic", "/joint_state_broadcaster/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("robot_state_publisher_topic", "/rsp_joint_states")
        self.declare_parameter("drop_suffix", "_mimic")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        rsp_topic = self.get_parameter("robot_state_publisher_topic").value
        self.drop_suffix = self.get_parameter("drop_suffix").value
        self.output_pub = self.create_publisher(JointState, output_topic, 10)
        self.rsp_pub = self.create_publisher(JointState, rsp_topic, 10)
        self.subscription = self.create_subscription(JointState, input_topic, self._on_state, 10)
        self.get_logger().info(f"Filtering {input_topic}; publishing {output_topic} and {rsp_topic}")

    def _on_state(self, message: JointState):
        keep = [index for index, name in enumerate(message.name)
                if not (self.drop_suffix and name.endswith(self.drop_suffix))]
        filtered = JointState()
        filtered.header = message.header
        filtered.name = [message.name[index] for index in keep]
        for field in ("position", "velocity", "effort"):
            values = getattr(message, field)
            if values:
                setattr(filtered, field, [values[index] for index in keep if index < len(values)])
        self.output_pub.publish(filtered)
        self.rsp_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMimicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
