#!/usr/bin/env python3
"""Experimental shared worker entry point; camera.launch defaults to separate nodes.

Shared input measurements did not approve this as the default runtime. Mode,
generation and model-specific preprocessing remain independent.
"""
from asmc_camera.runtime.shared_runtime import main

if __name__ == '__main__':
    main()
