#!/usr/bin/env python3
"""Run the OMNI UDP server (convenience script).

This mirrors the TCP run script but starts the UDP server.
"""

import logging
import time
from udp_server import OMNIUDPServer, PoseData


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    def on_pose(p: PoseData):
        logging.debug("Pose callback")

    def on_cmd(cmd_id: int):
        logging.info(f"CMD callback: {cmd_id}")

    # Use the server's default trajectory source (ROS2 /planned_path bridge).
    server = OMNIUDPServer(host="0.0.0.0", port=9000, on_pose_callback=on_pose, on_cmd_callback=on_cmd)

    try:
        server.start()
        logging.info("UDP server running. Ctrl+C to stop.")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("Shutting down UDP server")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
