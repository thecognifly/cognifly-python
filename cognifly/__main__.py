"""
Usage:
  python3 -m cognifly controller
"""

import sys
from cognifly.cognifly_controller.cognifly_controller import run_controller

_, cmd, *args = sys.argv

if cmd == "controller":
    run_controller()
else:
    raise AttributeError("Undefined command: " + cmd)
