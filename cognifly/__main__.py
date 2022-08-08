"""
Usage:
  python3 -m cognifly controller
  or
  python3 -m cognifly controller-headless
"""

import sys
from cognifly.cognifly_controller.cognifly_controller import run_controller

_, cmd, *args = sys.argv
trace_logs = "trace" in args or "--trace" in args

if cmd == "controller":
    run_controller(trace_logs=trace_logs)
elif cmd == "controller-headless":
    run_controller(print_screen=False, trace_logs=trace_logs)
else:
    raise AttributeError("Undefined command: " + cmd)
