#!/usr/bin/env python3
"""Setup script for OMNI Pi Communication Server."""
from setuptools import setup

setup(
    name="omni-pi-server",
    version="1.0.0",
    description="Production-ready UDP communication server for OMNI robot stack",
    author="OMNI Project",
    packages=["pi_comm_server"],
    py_modules=[
        "pi_comm_server.protocol",
        "pi_comm_server.planner_stub",
        "pi_comm_server.ros2_manager",
        "pi_comm_server.udp_server",
        "pi_comm_server.run_udp_server",
        "pi_comm_server.test_client",
    ],
    entry_points={
        "console_scripts": [
            "omni-udp-server=pi_comm_server.run_udp_server:main",
            "omni-test-client=pi_comm_server.run_test_client:main",
        ]
    },
    python_requires=">=3.8",
)
