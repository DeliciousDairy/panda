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
orientations: power-save entry, repeated noOutput transitions, individual/all CAN
initializations, actual rate-limited CAN resets, active operation, ignition-channel
availability, retained diagnostic counters and error reporting after wake.

This is software regression evidence with simulated peripheral behavior. It does
not establish electrical bus conditions, physical wake timing, vehicle alarm causes
or the absence of error accumulation on the vehicle.
