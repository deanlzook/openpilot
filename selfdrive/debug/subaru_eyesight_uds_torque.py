from enum import IntEnum
import random
import sys
import argparse
from subprocess import check_output, CalledProcessError
from panda import Panda
from panda.python.uds import ACCESS_TYPE, CONTROL_PARAMETER_TYPE, DATA_IDENTIFIER_TYPE, DYNAMIC_DEFINITION_TYPE, RESET_TYPE, ROUTINE_CONTROL_TYPE, DynamicSourceDefinition, UdsClient, SESSION_TYPE
from Crypto.Cipher import AES
import time
from opendbc.can.packer import CANPacker


GEN2_ES_SECRET_KEY = {
  1: b'\x33\xe6\x3c\xa0\x43\x11\x53\x46\x0c\x18\xf1\x06\x4c\x70\xfe\x41',
  16: b'\xA8\x08\x35\x0D\x2B\xAF\x20\x84\xF0\xA5\xC1\x07\x90\xA5\xD5\x06',
  32: b'\x29\x5A\x6E\xBA\xEC\x78\xD7\x74\xD8\xAA\xC1\xE0\xB3\xB5\x75\x0C'
}

class ACCESS_TYPE_LEVEL_1(IntEnum):
  REQUEST_SEED = ACCESS_TYPE.REQUEST_SEED + 2
  SEND_KEY = ACCESS_TYPE.SEND_KEY + 2

def gen2_security_access(seed, version=1):
    cipher = AES.new(GEN2_ES_SECRET_KEY[version], AES.MODE_ECB)
    return cipher.encrypt(seed)

try:
  check_output(["pidof", "boardd"])
  print("boardd is running, please kill openpilot before running this script! (aborted)")
  sys.exit(1)
except CalledProcessError as e:
  if e.returncode != 1: # 1 == no process found (boardd not running)
    raise e

red_panda = Panda()
red_panda.set_safety_mode(Panda.SAFETY_OPENPORT)

packer = CANPacker("subaru_global_2017_generated")

while True:
  msg = packer.make_can_msg("Steering_Torque", 2, {
    "Steer_Warning": random.choice([True, False]),
    "Steer_Error_1": random.choice([True, False]),
    "Steer_Error_2": random.choice([True, False]),
    "Steering_Angle": 0,#random.randint(-90, 90),
    "Steer_Torque_Sensor": -2,
    "Steer_Torque_Output": -1.3,
  })

  red_panda.can_send(msg[0], msg[2], msg[3])

  time.sleep(0.01)
  