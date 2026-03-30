#!/usr/bin/env python3
"""Run the OMNI UDP server (convenience script).

Convenience wrapper to start the UDP server.
"""

import logging
import sys
import time
from udp_server import OMNIUDPServer, PoseData


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    def on_pose(p: PoseData):
        pass

    def on_cmd(cmd_id: int):
        pass

    # Use the server's default trajectory source (ROS2 /planned_path bridge).
    server = OMNIUDPServer(host="0.0.0.0", port=9000, on_pose_callback=on_pose, on_cmd_callback=on_cmd)

    started = False

    try:
        server.start()
        started = True
        logging.info("UDP server running. Ctrl+C to stop.")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("Shutting down UDP server")
    except Exception as exc:
        logging.exception(f"UDP server failed: {exc}")
        sys.exit(1)
    finally:
        if started:
            server.stop()
        else:
            server.stop(stop_ros2_stack=False)


if __name__ == "__main__":
    main()
