"""Regression for CAN reinitialization undoing the existing power-save IRQ mask.

Build board/obj/panda_h7/main.elf first, or select a retained compiled image with
PANDA_ARM_ELF. Missing artifacts/dependencies are errors, never passing skips.
"""
import os
from pathlib import Path

import pytest

from arm_can import Firmware

ROOT = Path(__file__).resolve().parents[2]
REINITS = ("all", "can0", "can1", "can2", "clear0", "clear1", "clear2")


@pytest.fixture(params=[("cuatro", 1), ("cuatro", 2), ("red", 1), ("red", 2)], ids=lambda x: f"{x[0]}-orientation{x[1]}")
def fw(request):
  path = Path(os.environ.get("PANDA_ARM_ELF", ROOT / "board/obj/panda_h7/main.elf"))
  assert path.is_file(), f"Build the H7 ELF first, or set PANDA_ARM_ELF: {path}"
  board, orientation = request.param
  instance = Firmware(path, board=board, orientation=orientation)
  instance.call("can_init_all")
  yield instance, orientation
  instance.close()


def expected_sleep(orientation):
  return [[orientation == 1] * 2, [False] * 2, [orientation == 2] * 2]


def reinit(instance, operation):
  if operation == "all":
    instance.call("can_init_all")
  elif operation.startswith("can"):
    instance.call("llcan_init", instance.CANS[int(operation[-1])])
  else:
    bus = int(operation[-1])
    # Permit the existing 10Hz reset-rate limiter to perform an actual reset.
    instance.write_word(0x40000024, 200001)  # TIM2 CNT
    instance.call("can_clear_send", instance.CANS[bus], bus)
    offset, size = instance.member("can_core_reset_cnt")
    assert instance.word(instance.address("can_health") + bus * size + offset) == 1


def test_power_save_entry_keeps_ignition_can_receiving(fw):
  instance, orientation = fw
  assert instance.can_irqs() == [[True] * 2] * 3
  instance.call("set_power_save_state", True)
  assert instance.can_irqs() == expected_sleep(orientation)


def test_safety_switch_and_repeated_power_request_preserve_sleep(fw):
  instance, orientation = fw
  instance.call("set_power_save_state", True)
  for _ in range(3):
    instance.call("set_safety_mode", 19, 0)  # NOOUTPUT, same path as host offroad transition
    instance.call("set_power_save_state", True)
    assert instance.can_irqs() == expected_sleep(orientation)


@pytest.mark.parametrize("operation", REINITS)
def test_each_reinitialization_preserves_sleeping_irqs(fw, operation):
  instance, orientation = fw
  instance.call("set_power_save_state", True)
  reinit(instance, operation)
  assert instance.can_irqs() == expected_sleep(orientation)


@pytest.mark.parametrize("operation", REINITS)
def test_active_reinitialization_keeps_every_channel_receiving(fw, operation):
  instance, _ = fw
  reinit(instance, operation)
  assert instance.can_irqs() == [[True] * 2] * 3


def test_wake_restores_irqs_and_error_reporting_without_clearing_history(fw):
  instance, orientation = fw
  main_bus = 0 if orientation == 1 else 2
  module_bus = 2 if orientation == 1 else 0
  instance.deliver_error(module_bus)
  assert instance.errors(module_bus) == 1
  instance.call("set_power_save_state", True)
  instance.call("set_safety_mode", 19, 0)
  assert instance.can_irqs() == expected_sleep(orientation)
  instance.deliver_error(module_bus)
  assert instance.errors(module_bus) == 1
  instance.deliver_error(main_bus)
  assert instance.errors(main_bus) == 1  # ignition channel diagnostics stay live
  instance.call("set_power_save_state", False)
  assert instance.can_irqs() == [[True] * 2] * 3
  instance.deliver_error(module_bus)
  assert instance.errors(module_bus) == 2
