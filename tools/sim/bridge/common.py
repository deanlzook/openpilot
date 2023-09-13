import argparse
from functools import partial
import os
import signal
import threading
import time
from multiprocessing import Process, Queue
from abc import ABC, abstractmethod

import numpy as np
import pyopencl as cl
import pyopencl.array as cl_array

import cereal.messaging as messaging
from cereal import log
from cereal.visionipc import VisionIpcServer, VisionStreamType
from openpilot.common.basedir import BASEDIR
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params
from openpilot.common.realtime import DT_DMON, Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.lib.can import can_function
from openpilot.selfdrive.car.honda.values import CruiseButtons
from openpilot.tools.sim.lib.common import SimulatorState

W, H = 1928, 1208
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100

pm = messaging.PubMaster(['roadCameraState', 'wideRoadCameraState', 'accelerometer', 'gyroscope', 'can', "gpsLocationExternal"])
sm = messaging.SubMaster(['carControl', 'controlsState'])

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between CARLA and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  # TODO(jon-chuang): Multiplex the default based on detection of `CUDA` or `ROCm`
  parser.add_argument('--high_quality', action='store_true')
  # TODO(jon-chuang): Multiplex the default based on detection of `CUDA` or `ROCm`
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--town', type=str, default='Town04_Opt')
  parser.add_argument('--spawn_point', dest='num_selected_spawn_point', type=int, default=16)
  parser.add_argument('--host', dest='host', type=str, default='127.0.0.1')
  parser.add_argument('--port', dest='port', type=int, default=2000)
  parser.add_argument('--simulator', dest='simulator', type=str, default='carla')
  parser.add_argument('--ticks_per_frame', dest='ticks_per_frame', type=int, default=None)

  return parser.parse_args(add_args)

def steer_rate_limit(old, new):
  # Rate limiting to 0.5 degrees per step
  limit = 0.5
  if new > old + limit:
    return old + limit
  elif new < old - limit:
    return old - limit
  else:
    return new


class Camerad:
  def __init__(self, dual_camera):
    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    if dual_camera:
      self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 5, False, W, H)

    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, False, W, H)
    self.vipc_server.start_listener()

    # set up for pyopencl rgb to yuv conversion
    self.ctx = cl.create_some_context()
    self.queue = cl.CommandQueue(self.ctx)
    cl_arg = f" -DHEIGHT={H} -DWIDTH={W} -DRGB_STRIDE={W * 3} -DUV_WIDTH={W // 2} -DUV_HEIGHT={H // 2} -DRGB_SIZE={W * H} -DCL_DEBUG "

    kernel_fn = os.path.join(BASEDIR, "tools/sim/rgb_to_nv12.cl")
    with open(kernel_fn) as f:
      prg = cl.Program(self.ctx, f.read()).build(cl_arg)
      self.krnl = prg.rgb_to_nv12
    self.Wdiv4 = W // 4 if (W % 4 == 0) else (W + (4 - W % 4)) // 4
    self.Hdiv4 = H // 4 if (H % 4 == 0) else (H + (4 - H % 4)) // 4

  def cam_send_yuv_road(self, yuv):
    self._send_yuv(yuv, self.frame_road_id, 'roadCameraState', VisionStreamType.VISION_STREAM_ROAD)
    self.frame_road_id += 1

  def cam_send_yuv_wide_road(self, yuv):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD)
    self.frame_wide_id += 1

  # Returns: yuv bytes
  def rgb_to_yuv(self, rgb):
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8

    rgb_cl = cl_array.to_device(self.queue, rgb)
    yuv_cl = cl_array.empty_like(rgb_cl)
    self.krnl(self.queue, (self.Wdiv4, self.Hdiv4), None, rgb_cl.data, yuv_cl.data).wait()
    yuv = np.resize(yuv_cl.get(), rgb.size // 2)
    return yuv.data.tobytes()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    eof = int(frame_id * 0.05 * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

    dat = messaging.new_message(pub_type)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    pm.send(pub_type, dat)

def imu_callback(simulator_state):
  # send 5x since 'sensor_tick' doesn't seem to work. limited by the world tick?
  for _ in range(5):
    dat = messaging.new_message('accelerometer')
    dat.accelerometer.sensor = 4
    dat.accelerometer.type = 0x10
    dat.accelerometer.timestamp = dat.logMonoTime  # TODO: use the IMU timestamp
    dat.accelerometer.init('acceleration')
    dat.accelerometer.acceleration.v = [simulator_state.imu.accelerometer.x, simulator_state.imu.accelerometer.y, simulator_state.imu.accelerometer.z]
    pm.send('accelerometer', dat)

    # copied these numbers from locationd
    dat = messaging.new_message('gyroscope')
    dat.gyroscope.sensor = 5
    dat.gyroscope.type = 0x10
    dat.gyroscope.timestamp = dat.logMonoTime  # TODO: use the IMU timestamp
    dat.gyroscope.init('gyroUncalibrated')
    dat.gyroscope.gyroUncalibrated.v = [simulator_state.imu.gyroscope.x, simulator_state.imu.gyroscope.y, simulator_state.imu.gyroscope.z]
    pm.send('gyroscope', dat)
    time.sleep(0.01)


def panda_state_function(vs: SimulatorState, exit_event: threading.Event):
  pm = messaging.PubMaster(['pandaStates'])
  while not exit_event.is_set():
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': vs.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'hondaNidec',
    }
    pm.send('pandaStates', dat)
    time.sleep(0.5)


def peripheral_state_function(exit_event: threading.Event):
  pm = messaging.PubMaster(['peripheralState'])
  while not exit_event.is_set():
    dat = messaging.new_message('peripheralState')
    dat.valid = True
    # fake peripheral state data
    dat.peripheralState = {
      'pandaType': log.PandaState.PandaType.blackPanda,
      'voltage': 12000,
      'current': 5678,
      'fanSpeedRpm': 1000
    }
    Params().put_bool("ObdMultiplexingEnabled", False)
    pm.send('peripheralState', dat)
    time.sleep(0.5)


def gps_callback(simulator_state: SimulatorState):
  dat = messaging.new_message('gpsLocationExternal')

  if not simulator_state.valid:
    return

  # transform vel from carla to NED
  # north is -Y in CARLA
  velNED = [
    -simulator_state.velocity.y,  # north/south component of NED is negative when moving south
    simulator_state.velocity.x,  # positive when moving east, which is x in carla
    simulator_state.velocity.z,
  ]

  dat.gpsLocationExternal = {
    "unixTimestampMillis": int(time.time() * 1000),
    "flags": 1,  # valid fix
    "accuracy": 1.0,
    "verticalAccuracy": 1.0,
    "speedAccuracy": 0.1,
    "bearingAccuracyDeg": 0.1,
    "vNED": velNED,
    "bearingDeg": simulator_state.imu.bearing,
    "latitude": simulator_state.gps.latitude,
    "longitude": simulator_state.gps.longitude,
    "altitude": simulator_state.gps.altitude,
    "speed": simulator_state.speed,
    "source": log.GpsLocationData.SensorSource.ublox,
  }

  pm.send('gpsLocationExternal', dat)


def fake_driver_monitoring(exit_event: threading.Event):
  pm = messaging.PubMaster(['driverStateV2', 'driverMonitoringState'])
  while not exit_event.is_set():
    # dmonitoringmodeld output
    dat = messaging.new_message('driverStateV2')
    dat.driverStateV2.leftDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.leftDriverData.faceProb = 1.0
    dat.driverStateV2.rightDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.rightDriverData.faceProb = 1.0
    pm.send('driverStateV2', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    pm.send('driverMonitoringState', dat)

    time.sleep(DT_DMON)


def sensor_callback(simulator_state: SimulatorState, idx):
  imu_callback(simulator_state)
  gps_callback(simulator_state)


def function_runner(function, exit_event: threading.Event):
  i = 0
  while not exit_event.is_set():
    function(i)
    time.sleep(0.01)
    i += 1

class World(ABC):
  @abstractmethod
  def apply_controls(self, steer_sim, throttle_out, brake_out, rk):
    pass
  @abstractmethod
  def tick(self, simulator_state: SimulatorState, startup=False):
    pass


class SimulatorBridge(ABC):
  TICKS_PER_FRAME = 5
  def __init__(self, arguments):
    if arguments.ticks_per_frame:
      self.ticks_per_frame = arguments.ticks_per_frame
    else:
      self.ticks_per_frame = self.TICKS_PER_FRAME
    print(f"Running at {self.ticks_per_frame} tick(s) per frame")
    set_params_enabled()
    self.params = Params()

    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = 20
    msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
    self.params.put("CalibrationParams", msg.to_bytes())

    self.dual_camera = arguments.dual_camera
    self.high_quality = arguments.high_quality

    self._exit_event = threading.Event()
    self._threads = []
    self._keep_alive = True
    self.started = False
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self._exit = threading.Event()
    self.simulator_state = SimulatorState()

  def _on_shutdown(self, signal, frame):
    self._keep_alive = False

  def bridge_keep_alive(self, q: Queue, retries: int):
    try:
      self._run(q)
    finally:
      self.close()

  def close(self):
    self.started = False
    self._exit_event.set()

  def run(self, queue, retries=-1):
    bridge_p = Process(target=self.bridge_keep_alive, args=(queue, retries), daemon=True)
    bridge_p.start()
    return bridge_p

  # Must return object of class `World`, after spawning objects into that world
  @abstractmethod
  def spawn_world(self) -> World:
    pass

  def _run(self, q: Queue):
    world = self.spawn_world()

    # launch fake car threads
    self._threads.append(threading.Thread(target=panda_state_function, args=(self.simulator_state, self._exit_event,)))
    self._threads.append(threading.Thread(target=peripheral_state_function, args=(self._exit_event,)))
    self._threads.append(threading.Thread(target=fake_driver_monitoring, args=(self._exit_event,)))
    self._threads.append(threading.Thread(target=function_runner, args=(partial(can_function, pm, self.simulator_state), self._exit_event,)))
    self._threads.append(threading.Thread(target=function_runner, args=(partial(sensor_callback, self.simulator_state), self._exit_event,)))
    for t in self._threads:
      t.start()

    old_steer = 0

    rk = Ratekeeper(100, print_delay_threshold=None)

    # Simulation tends to be slow in the initial steps. This prevents lagging later
    for _ in range(20):
      world.tick(self.simulator_state, startup=True)
    
    last_manual_message = time.time()
    throttle_manual = steer_manual = brake_manual = 0.

    
    # loop
    while self._keep_alive:
      # 1. Read the throttle, steer and brake from op or manual controls
      # 2. Set instructions in Carla
      # 3. Send current carstate to op via can

      throttle_out = steer_out = brake_out = 0.0
      throttle_op = steer_op = brake_op = 0.0

      self.simulator_state.cruise_button = 0

      if (time.time() - last_manual_message) > 0.25: # haven't recieved a manual message in 0.25 seconds
        throttle_manual = steer_manual = brake_manual = 0.

      # --------------Step 1-------------------------------
      if not q.empty():
        message = q.get()
        m = message.split('_')
        if m[0] == "steer":
          steer_manual = float(m[1])
        elif m[0] == "throttle":
          throttle_manual = float(m[1])
        elif m[0] == "brake":
          brake_manual = float(m[1])
        elif m[0] == "cruise":
          if m[1] == "down":
            self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
          elif m[1] == "up":
            self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
          elif m[1] == "cancel":
            self.simulator_state.cruise_button = CruiseButtons.CANCEL
          elif m[1] == "main":
            self.simulator_state.cruise_button = CruiseButtons.MAIN
        elif m[0] == "ignition":
          self.simulator_state.ignition = not self.simulator_state.ignition
        elif m[0] == "quit":
          break
        
        last_manual_message = time.time()

      self.simulator_state.user_brake = brake_manual
      self.simulator_state.user_gas = throttle_manual
     
      sm.update(0)

      is_openpilot_engaged = sm['controlsState'].active

      if is_openpilot_engaged:

        # TODO gas and brake is deprecated
        throttle_op = clip(sm['carControl'].actuators.accel / 1.6, 0.0, 1.0)
        brake_op = clip(-sm['carControl'].actuators.accel / 4.0, 0.0, 1.0)
        steer_op = sm['carControl'].actuators.steer

      throttle_out = throttle_op if is_openpilot_engaged else throttle_manual
      brake_out = brake_op if is_openpilot_engaged else brake_manual
      steer_out = steer_op if is_openpilot_engaged else steer_manual

      steer_out = steer_rate_limit(old_steer, steer_out)
      old_steer = steer_out

      world.apply_controls(steer_out, throttle_out, brake_out, rk)

      if rk.frame % self.TICKS_PER_FRAME == 0:
        world.tick(self.simulator_state)
      
      rk.keep_time()
      self.started = True
