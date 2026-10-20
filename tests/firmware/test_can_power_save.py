"""Regression for CAN reinitialization undoing the existing power-save IRQ mask.

Build board/obj/panda_h7/main.elf first, or select a retained compiled image with
PANDA_ARM_ELF. Missing artifacts/dependencies are errors, never passing skips.
"""
import os
import struct
from pathlib import Path

import pytest
from unicorn import UC_HOOK_MEM_WRITE

from arm_can import Firmware
from unicorn.arm_const import UC_ARM_REG_PRIMASK, UC_ARM_REG_R0

ROOT = Path(__file__).resolve().parents[2]
REINITS = ("all", "can0", "can1", "can2", "clear0", "clear1", "clear2")


@pytest.fixture(params=[(board, orientation) for board in ("cuatro", "red") for orientation in (0, 1, 2)],
                ids=lambda x: f"{x[0]}-orientation{x[1]}")
def fw(request):
  path = Path(os.environ.get("PANDA_ARM_ELF", ROOT / "board/obj/panda_h7/main.elf"))
  assert path.is_file(), f"Build the H7 ELF first, or set PANDA_ARM_ELF: {path}"
  board, orientation = request.param
  instance = Firmware(path, board=board, orientation=orientation)
  instance.call("can_init_all")
  yield instance, orientation
  instance.close()


def expected_sleep(orientation):
  return [[orientation != 2] * 2, [False] * 2, [orientation == 2] * 2]


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
  main_bus = 0 if orientation != 2 else 2
  module_bus = 2 if orientation != 2 else 0
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


@pytest.mark.parametrize("requested", [False, True])
@pytest.mark.parametrize("bus", [0, 1, 2])
def test_power_interrupt_during_reinit_preserves_final_policy(fw, requested, bus):
  instance, orientation = fw
  instance.call("set_power_save_state", not requested)
  instance.reinit_with_power_interrupt(bus, requested)
  assert instance.can_irqs() == (expected_sleep(orientation) if requested else [[True] * 2] * 3)


@pytest.mark.parametrize("initial_mask", [0, 1])
def test_policy_restores_callers_interrupt_mask(fw, initial_mask):
  instance, orientation = fw
  instance.uc.reg_write(UC_ARM_REG_PRIMASK, initial_mask)
  instance.call("set_power_save_state", True)
  instance.call("llcan_init", instance.CANS[1])
  assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == initial_mask
  assert instance.can_irqs() == expected_sleep(orientation)
  instance.call("set_power_save_state", False)
  assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == initial_mask
  assert instance.can_irqs() == [[True] * 2] * 3


@pytest.mark.parametrize("requested", [False, True])
def test_reinit_interrupt_during_power_transition_preserves_final_policy(fw, requested):
  instance, orientation = fw
  module_bus = 0 if orientation == 2 else 2
  instance.call("set_power_save_state", not requested)
  instance.power_with_reinit_interrupt(requested, module_bus)
  assert instance.can_irqs() == (expected_sleep(orientation) if requested else [[True] * 2] * 3)


def test_masked_hardware_windows_are_bounded_and_exclude_serial(fw):
  instance, _ = fw
  instance.call("set_power_save_state", True)
  instance.call("can_init_all")
  instance.call("set_power_save_state", False)
  assert instance.max_masked <= 1024
  assert instance.serial_while_masked == 0


def assert_transceivers(instance, orientation, sleeping):
  if instance.word(instance.address("current_board")) == instance.address("board_cuatro"):
    pins = ((0x58020400, 7), (0x58020400, 10), (0x58020C00, 8), (0x58020400, 11))
  else:
    pins = ((0x58021800, 11), (0x58020400, 3), (0x58020C00, 7), (0x58020400, 4))
  main = 2 if orientation == 2 else 0
  for index, (gpio, pin) in enumerate(pins):
    # Actual board callbacks drive active-low enable pins and configure output mode.
    assert bool(instance.word(gpio + 0x14) & (1 << pin)) == (sleeping and index != main)
    assert (instance.word(gpio) >> (2 * pin)) & 3 == 1


def test_saturated_register_map_still_completes_power_transaction(fw):
  instance, orientation = fw
  instance.saturate_register_map()
  instance.max_masked = instance.current_masked = 0
  for sleeping in (True, False, True, False):
    instance.call("set_power_save_state", sleeping)
    assert_transceivers(instance, orientation, sleeping)
    assert instance.can_irqs() == (expected_sleep(orientation) if sleeping else [[True] * 2] * 3)
    assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == 0
  # Explicit pathological bound, separate from the normal-map 1024 instruction
  # gate. Every GPIO shadow lookup exhausts all 1023 bounded collision attempts.
  assert instance.max_masked <= 160000
  assert instance.serial_while_masked == 0


@pytest.mark.parametrize("after_wake", [False, True])
@pytest.mark.parametrize("bus", [0, 1, 2])
@pytest.mark.parametrize("safety_param", [0, 2], ids=["primary", "secondary"])
@pytest.mark.parametrize("kind", ["standard", "extended_fd", "actuation"])
def test_actual_frame_receive_forward_and_tx_before_and_after_wake(fw, after_wake, bus, safety_param, kind):
  instance, orientation = fw
  instance.call("can_set_orientation", orientation == 2)
  extended = fd = kind == "extended_fd"
  instance.call("set_safety_mode", 33, safety_param)  # actual primary/secondary Rivian hooks
  if after_wake:
    for _ in range(3):
      instance.call("set_power_save_state", True)
      assert_transceivers(instance, orientation, True)
      instance.call("set_safety_mode", 19, 0)
      instance.call("set_power_save_state", False)
      assert_transceivers(instance, orientation, False)
    instance.call("set_safety_mode", 33, safety_param)
  payload = bytes(range(64 if fd else 8))
  address = 0x110 if kind == "actuation" else (0x123456 if extended else 0x456)
  instance.deliver_frame(bus, address, payload, extended, fd)
  assert instance.health(bus, "total_rx_cnt") == 1
  assert instance.word(instance.address("rx_buffer_overflow")) == 0
  assert instance.word(instance.address("tx_buffer_overflow")) == 0
  assert instance.word(instance.address("safety_rx_invalid")) == 0
  # Both configurations must continue blocking stock steering command 0x110
  # toward bus0, while forwarding ordinary traffic in the usual directions.
  logical_bus = 2 - bus if orientation == 2 and bus != 1 else bus
  destination = None if kind == "actuation" and logical_bus == 2 else {0: 2, 1: None, 2: 0}[bus]
  assert instance.health(bus, "total_fwd_cnt") == int(destination is not None)
  if destination is None:
    assert instance.transmissions == []
  else:
    assert instance.health(destination, "total_tx_cnt") == 1
    assert len(instance.transmissions) == 1
    actual_bus, fifo = instance.transmissions[0]
    assert actual_bus == destination
    header0, header1 = struct.unpack("<II", fifo[:8])
    assert header0 == (address if extended else address << 18) | (int(extended) << 30)
    assert header1 == ((15 if fd else 8) << 16) | (int(fd) << 21)
    assert fifo[8:8 + len(payload)] == payload
  # The host receives the exact original payload as well as the TX receipt when
  # forwarded. The actual firmware checksum must validate each queued packet.
  queue = instance.address("can_rx_q")
  scratch = 0x2003B000
  received = []
  while instance.word(queue) != instance.word(queue + 4):
    instance.call("can_pop", queue, scratch)
    instance.call("can_check_checksum", scratch)
    assert instance.uc.reg_read(UC_ARM_REG_R0) == 1
    received.append(bytes(instance.uc.mem_read(scratch, 6 + len(payload))))
  assert len(received) == (2 if destination is not None else 1)
  assert all(packet[6:] == payload for packet in received)


# The original IRQ-only repair left the mux/transceiver path untested after a
# sleeping safety switch. These cases execute the real board and control paths.
MUX_BOARDS = [(board, orientation) for board in ("red", "cuatro", "tres") for orientation in (0, 1, 2)]


def mux_firmware(board, orientation, sleeping):
  path = Path(os.environ.get("PANDA_ARM_ELF", ROOT / "board/obj/panda_h7/main.elf"))
  instance = Firmware(path, board=board, orientation=orientation)
  instance.call("can_set_orientation", orientation == 2)
  instance.call("can_init_all")
  instance.call("set_safety_mode", 19, 0)
  instance.call("set_power_save_state", True)
  if not sleeping:
    instance.call("set_power_save_state", False)
  return instance


def mux_pins(instance):
  board = instance.word(instance.address("current_board"))
  if board == instance.address("board_red"):
    return ((0x58020400, 3), (0x58020400, 4))
  return ((0x58020400, 10), (0x58020400, 11))


def assert_mux(instance, orientation, mode, sleeping):
  enabled = [not bool(instance.word(gpio + 0x14) & (1 << pin)) for gpio, pin in mux_pins(instance)]
  selected = int((mode == 0) == (orientation == 2))
  assert enabled[selected] == (not sleeping)
  if sleeping:
    assert enabled == [False, False]
  # The ignition path stays available. Tres physically ties its two enables.
  board = instance.word(instance.address("current_board"))
  if board == instance.address("board_cuatro"):
    gpio, pin = ((0x58020C00, 8) if orientation == 2 else (0x58020400, 7))
  else:
    gpio, pin = ((0x58020C00, 7) if orientation == 2 else (0x58021800, 11))
  assert not instance.word(gpio + 0x14) & (1 << pin)
  if not sleeping:
    # Neither main nor module transceiver may be lost to a stale shared-port
    # GPIO read/modify/write when power wake interrupts mux configuration.
    if board == instance.address("board_cuatro"):
      pins = ((0x58020400, 7), (0x58020C00, 8))
    else:
      pins = ((0x58021800, 11), (0x58020C00, 7))
    assert all(not instance.word(gpio + 0x14) & (1 << pin) for gpio, pin in pins)
  assert instance.can_irqs() == (expected_sleep(orientation) if sleeping else [[True] * 2] * 3)


def control(instance, request, param1=0, param2=0):
  pointer = 0x2003B000
  instance.uc.mem_write(pointer, struct.pack("<BHHH", request, param1, param2, 0))
  instance.call("comms_control_handler", pointer, pointer + 32)


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("mode,param", [(0, 0), (19, 0), (3, 0), (3, 1), (33, 0), (33, 2)])
def test_sleeping_safety_switch_preserves_transceivers(board, orientation, mode, param):
  instance = mux_firmware(board, orientation, True)
  try:
    for _ in range(3):
      control(instance, 0xDC, mode, param)
      assert_mux(instance, orientation, int(mode == 3 and param == 0), True)
      control(instance, 0xE7, 1)  # repeated power-save request must be harmless
      assert_mux(instance, orientation, int(mode == 3 and param == 0), True)
  finally:
    instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("mode", [0, 1])
@pytest.mark.parametrize("sleeping", [False, True])
@pytest.mark.parametrize("initial_mask", [0, 1])
def test_direct_mux_honors_power_policy_and_callers_mask(board, orientation, mode, sleeping, initial_mask):
  instance = mux_firmware(board, orientation, sleeping)
  try:
    instance.uc.reg_write(UC_ARM_REG_PRIMASK, initial_mask)
    instance.max_masked = instance.current_masked = 0
    control(instance, 0xDB, mode)
    assert_mux(instance, orientation, mode, sleeping)
    assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == initial_mask
    if not initial_mask:
      assert instance.max_masked <= 1024
    assert instance.serial_while_masked == 0
  finally:
    instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("initial_mode,initial_power", [(0, False), (0, True), (33, False)])
def test_actual_host_parked_command_sequence_then_ignition_wake(board, orientation, initial_mode, initial_power):
  instance = mux_firmware(board, orientation, initial_power)
  try:
    instance.call("set_safety_mode", initial_mode, 0)
    # This ordering is emitted by the complete actual host function retained in
    # the separate host-policy probe; it is not a replica of the host decision logic.
    if initial_mode == 0:
      control(instance, 0xDC, 19)
    if not initial_power:
      control(instance, 0xE7, 1)
    control(instance, 0xDC, 19)
    assert_mux(instance, orientation, 0, True)
    for counter in (0, 1):
      instance.deliver_frame(2 if orientation == 2 else 0, 0x152,
                             bytes([0, counter, 0, 0, 0, 0, 0, 0x10]))
    assert instance.uc.mem_read(instance.address("ignition_can"), 1) == b"\x01"
    control(instance, 0xE7, 0)
    assert_mux(instance, orientation, 0, False)
    instance.call("set_safety_mode", 33, 0)
    assert_mux(instance, orientation, 0, False)
  finally:
    instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("mode", [0, 1])
@pytest.mark.parametrize("requested", [False, True])
def test_power_request_interrupting_mux_cannot_reenable_sleeping_transceiver(board, orientation, mode, requested):
  name = "red_set_can_mode" if board == "red" else "tres_set_can_mode"
  trace = mux_firmware(board, orientation, not requested)
  try:
    boundaries = trace.policy_boundaries(name, mode)
  finally:
    trace.close()
  for boundary in boundaries:
    instance = mux_firmware(board, orientation, not requested)
    try:
      instance.call_with_interrupt(name, (mode,), "set_power_save_state", (requested,), boundary)
      assert_mux(instance, orientation, mode, requested)
      assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == 0
      assert instance.serial_while_masked == 0
    finally:
      instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("mode", [0, 1])
def test_sleeping_mux_never_pulses_either_transceiver_enable(board, orientation, mode):
  instance = mux_firmware(board, orientation, True)
  writes = []
  def observe(uc, access, address, size, value, data):
    for gpio, pin in mux_pins(instance):
      if address == gpio + 0x14:
        writes.append(bool(value & (1 << pin)))
  hook = instance.uc.hook_add(UC_HOOK_MEM_WRITE, observe, begin=0x58020000, end=0x58021FFF)
  try:
    control(instance, 0xDB, mode)
    assert writes and all(writes)  # active-low: every observed write leaves both disabled
    assert_mux(instance, orientation, mode, True)
  finally:
    instance.uc.hook_del(hook)
    instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("requested", [False, True])
@pytest.mark.parametrize("mode", [0, 1])
def test_mux_interrupting_power_request_preserves_final_policy(board, orientation, requested, mode):
  name = "red_set_can_mode" if board == "red" else "tres_set_can_mode"
  trace = mux_firmware(board, orientation, not requested)
  try:
    boundaries = trace.policy_boundaries("set_power_save_state", requested)
  finally:
    trace.close()
  for boundary in boundaries:
    instance = mux_firmware(board, orientation, not requested)
    try:
      instance.call_with_interrupt("set_power_save_state", (requested,), name, (mode,), boundary)
      assert_mux(instance, orientation, mode, requested)
      assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == 0
      assert instance.serial_while_masked == 0
    finally:
      instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("new_orientation", [0, 1, 2])
def test_sleeping_harness_refresh_reapplies_ignition_transceiver(board, orientation, new_orientation):
  instance = mux_firmware(board, orientation, True)
  try:
    instance.uc.mem_write(instance.address("harness"), bytes([new_orientation]))
    instance.call("can_set_orientation", new_orientation == 2)
    instance.call("can_init_all")
    instance.call("set_safety_mode", 19, 0)
    instance.call("set_power_save_state", True)  # existing tick_handler refresh sequence
    assert_mux(instance, new_orientation, 0, True)
    if board != "tres":
      assert_transceivers(instance, new_orientation, True)
  finally:
    instance.close()


@pytest.mark.parametrize("board,orientation", MUX_BOARDS)
@pytest.mark.parametrize("sleeping", [False, True])
def test_saturated_map_mux_switch_preserves_policy_without_unbounded_masking(board, orientation, sleeping):
  instance = mux_firmware(board, orientation, sleeping)
  try:
    instance.saturate_register_map()
    instance.max_masked = instance.current_masked = 0
    for mode in (0, 1, 0):
      control(instance, 0xDB, mode)
      assert_mux(instance, orientation, mode, sleeping)
    assert instance.max_masked <= 160000
    assert instance.serial_while_masked == 0
    assert instance.uc.reg_read(UC_ARM_REG_PRIMASK) == 0
  finally:
    instance.close()
