#!/usr/bin/env python3
"""Shared prepared handoff replaces the historical LOCAL-only selector grace."""
import subprocess
if __name__=='__main__':
    subprocess.run(['rostest','asmc_planner','jamming_handoff.test','strategy:=local'],check=True)
