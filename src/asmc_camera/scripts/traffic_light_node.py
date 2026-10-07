#!/usr/bin/env python3
"""Compatibility executable for the packaged ResNet18 runtime."""
from asmc_camera.nodes.traffic_light_node import main

if __name__ == "__main__":
    raise SystemExit(main())
