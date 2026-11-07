import os
import threading
from pathlib import Path

import rclpy
import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from .api import create_app
from .ros_backend import BridgeNode


def main():
    default_config = Path(
        get_package_share_directory("cr5a_ros_bridge")
    ) / "config" / "bridge.yaml"
    config_path = Path(os.environ.get("CR5A_BRIDGE_CONFIG", str(default_config)))
    if not config_path.is_file():
        raise FileNotFoundError(f"Bridge configuration not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    rclpy.init()
    node = BridgeNode(config)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name="cr5a-ros-executor", daemon=True)
    ros_thread.start()

    api_config = config.get("api", {})
    node.get_logger().info(
        f"CR5A Robot API listening on http://{api_config.get('host', '127.0.0.1')}:{api_config.get('port', 8000)}"
    )
    try:
        uvicorn.run(
            create_app(node, config),
            host=api_config.get("host", "127.0.0.1"),
            port=int(api_config.get("port", 8000)),
            log_level=api_config.get("log_level", "info"),
            access_log=False,
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
