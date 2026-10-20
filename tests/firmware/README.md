# CAN power-saving regression

These tests execute an actual H7 ELF under Unicorn. Serial output is bypassed;
peripheral registers use RAM and NVIC enable/disable registers use write-one
semantics. No panda connection or flash occurs. The loader/call mechanism extends
the existing Codex register-interleaving probe; unlike libpanda's stubbed can_init,
it executes actual CAN initialization, safety switching and power-saving code.

Build using the selected application's exact opendbc safety source and toolchain,
then run from the containing application with its test environment:

```sh
PANDA_ARM_ELF=/absolute/path/to/panda/board/obj/panda_h7/main.elf \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -o addopts= \
  -p no:cacheprovider --rootdir=panda/tests/firmware \
  --confcutdir=panda/tests/firmware -q panda/tests/firmware/test_can_power_save.py
```

The explicit pytest boundary also supports standalone worktrees whose directory
names are not importable Python package names. Missing ELF/dependencies are errors,
not skips. The default ELF is this checkout's board/obj/panda_h7/main.elf.

Coverage includes both actual red/cuatro board function tables and both harness
orientations plus no harness: power-save entry, repeated noOutput transitions, individual/all CAN
initializations, actual rate-limited CAN resets, active operation, ignition-channel
availability, retained diagnostic counters and error reporting after wake.
Explicit instruction-boundary interruptions cover power requests during CAN init
and CAN init during a power transition. Tests preserve initial PRIMASK0/1 and bound
normal-fixture hardware-only masking to1024 instructions; UART output must remain
outside it. A separate adversarial test fills all1023 legal register-map slots
through actual register_set calls, forcing every transition GPIO shadow lookup to
exhaust the existing1023-attempt collision loop. It checks completion, GPIO modes
and active-low transceiver enables, IRQ policy and mask restoration within a
separate160000-instruction ceiling. This stress fixture exceeds normal boot-map
occupancy; neither instruction ceiling is a hardware elapsed-time measurement.

Actual receive IRQs consume modeled standard and extended CAN-FD FIFO elements;
RX acknowledgments and write-one-to-clear IR registers are modeled. The unchanged
firmware runs safety hooks, forwarding, software queues, checksums and TX FIFO
writes. Tests verify payloads and counters for every controller, primary/secondary
Rivian safety configurations, both orientations and no harness, before and after
three sleep/noOutput/wake cycles. Stock steering-message forwarding restrictions
must remain intact, including logical bus remapping with a flipped harness.

This is software regression evidence with simulated peripheral behavior. It does
not establish electrical bus conditions, physical wake timing, vehicle alarm causes
or the absence of error accumulation on the vehicle.

The transceiver extension covers actual red/cuatro/tres mode callbacks and USB/SPI
control requests, both mux selections, safety modes SILENT/NOOUTPUT/ELM327/Rivian,
repeated sleep requests and the host's recorded parked command ordering. GPIO
write observation rejects even a transient active-low enable pulse while asleep.
Actual Drive_On frames must still wake the receiver and normal active CAN policy.
Harness refresh uses the existing tick handler's sequence and checks the newly
selected ignition transceiver. Explicit higher-priority requests are injected at
instruction boundaries around actual policy and GPIO reads/writes in both
mux-versus-power directions. Both caller masks are preserved; added mux critical
sections retain the normal1024/saturated160000 instruction ceilings. Tres's
physically tied main CAN enables are modeled through its actual callbacks.
The full-call instruction guard is2000000 to allow the existing saturated register
map's unmasked mux setup; the stricter masked-window guards are unchanged.
