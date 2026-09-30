"""Put controllers/rescue_robot on sys.path so tests import modules without Webots."""

import os
import sys

CONTROLLER_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "controllers", "rescue_robot"))
if CONTROLLER_DIR not in sys.path:
    sys.path.insert(0, CONTROLLER_DIR)
