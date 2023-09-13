#!/usr/bin/env python

from multiprocessing import Queue
from openpilot.tools.sim.bridge.carla import CarlaBridge  # pylint: disable = no-name-in-module
from openpilot.tools.sim.bridge.common import parse_args, SimulatorBridge  # pylint: disable = no-name-in-module

from typing import Any

if __name__ == "__main__":
  q: Any = Queue()
  args = parse_args()

  simulator_bridge: SimulatorBridge
  if args.simulator == "carla":
    simulator_bridge = CarlaBridge(args)
  else:
    raise AssertionError("simulator type not supported")
  p = simulator_bridge.run(q)

  if args.joystick:
    # start input poll for joystick
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread

    wheel_poll_thread(q)
  else:
    # start input poll for keyboard
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    keyboard_poll_thread(q)
  p.join()
