"""
Usage:
  python3 -m cognifly controller
  or
  python3 -m cognifly controller-headless
"""

import sys
from cognifly.cognifly_controller.cognifly_controller import run_controller

_, cmd, *args = sys.argv

if cmd == "controller":
    run_controller()
elif cmd == "controller-headless":
    run_controller(print_screen=False)
else:
    raise AttributeError("Undefined command: " + cmd)
