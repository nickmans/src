#!/usr/bin/env python3
"""Wrapper script for OMNI test client that handles imports correctly."""
import os
import sys

# Add the script directory to sys.path so imports work
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Now import and run the test client
from test_client import main
import asyncio

if __name__ == "__main__":
    asyncio.run(main())
