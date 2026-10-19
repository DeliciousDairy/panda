"""Actual H7 instructions with RAM peripherals and NVIC enable-bit semantics.

The loader/call boundary reuses the retained Codex panda_register_probe design.
Only serial output and external peripheral behavior are substituted. This is not
electrical CAN, wake-edge, interrupt-latency or vehicle validation.
"""
import struct
from pathlib import Path

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_PRIMASK


class Firmware:
  STOP = 0x08001000
  CANS = (0x4000A000, 0x4000A400, 0x4000D400)
  IRQS = ((19, 21), (20, 22), (159, 160))

  def __init__(self, path, board="cuatro", orientation=1):
    self.file = Path(path).open("rb")
    self.elf = ELFFile(self.file)
    self.symbols = {s.name: s for s in self.elf.get_section_by_name(".symtab").iter_symbols()}
    self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for address, length in [(0, 0x10000), (0x08000000, 0x200000), (0x20000000, 0x40000),
                            (0x24000000, 0x80000), (0x30000000, 0x80000),
                            (0x38000000, 0x10000), (0x40000000, 0x100000),
                            (0x58000000, 0x100000), (0xE000E000, 0x2000)]:
      self.uc.mem_map(address, length)
    for segment in self.elf.iter_segments():
      if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
        self.uc.mem_write(segment["p_vaddr"], segment.data())
    self.uc.mem_write(self.STOP, b"\x00\xbf")
    self.uc.reg_write(UC_ARM_REG_SP, 0x2003F000)
    self.uc.mem_write(self.address("interrupts_enabled"), b"\x01")
    self.skip = {self.address(n) & ~1 for n in ("print", "puth")}
    self.enabled = [0] * 8
    self.power_interrupt = None
    self.can_interrupt = None
    self.break_next = False
    self.break_address = None
    self.current_masked = 0
    self.max_masked = 0
    self.serial_while_masked = 0
    self.pending_register_writes = []
    self.transmissions = []
    self.uc.hook_add(UC_HOOK_CODE, self.code)
    self.uc.hook_add(UC_HOOK_MEM_WRITE, self.nvic_write, begin=0xE000E100, end=0xE000E19F)
    self.uc.hook_add(UC_HOOK_MEM_READ, self.nvic_read, begin=0xE000E100, end=0xE000E19F)
    self.uc.hook_add(UC_HOOK_MEM_READ, self.power_read,
                     begin=self.address("power_save_enabled"), end=self.address("power_save_enabled"))
    for controller in self.CANS:
      self.uc.hook_add(UC_HOOK_MEM_WRITE, self.can_write, begin=controller, end=controller + 0x3FF)
    self.write_word(self.address("current_board"), self.address("board_" + board))
    self.uc.mem_write(self.address("harness"), bytes([orientation]))
    self.error_offset, self.health_size = self.member("total_error_cnt")
    assert self.symbols["can_health"]["st_size"] == 3 * self.health_size

  def address(self, name):
    return self.symbols[name]["st_value"]

  def member(self, name):
    for unit in self.elf.get_dwarf_info().iter_CUs():
      for die in unit.iter_DIEs():
        attribute = die.attributes.get("DW_AT_name")
        if die.tag == "DW_TAG_member" and attribute and attribute.value == name.encode():
          return die.attributes["DW_AT_data_member_location"].value, die.get_parent().attributes["DW_AT_byte_size"].value
    raise ValueError(f"No compiled member information for {name}")

  def word(self, address):
    return struct.unpack("<I", self.uc.mem_read(address, 4))[0]

  def write_word(self, address, value):
    self.uc.mem_write(address, struct.pack("<I", value))

  def code(self, uc, address, size, data):
    for register, value in self.pending_register_writes:
      self.write_word(register, value)
    self.pending_register_writes.clear()
    if uc.reg_read(UC_ARM_REG_PRIMASK):
      self.current_masked += 1
      self.max_masked = max(self.max_masked, self.current_masked)
    else:
      self.current_masked = 0
    if self.break_next:
      self.break_next = False
      self.break_address = address
      uc.emu_stop()
      return
    if address in self.skip:
      self.serial_while_masked += bool(uc.reg_read(UC_ARM_REG_PRIMASK))
      uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

  def can_write(self, uc, access, address, size, value, data):
    for bus, controller in enumerate(self.CANS):
      if address == controller + 0x50:  # IR: write-one-to-clear
        self.pending_register_writes.append((address, self.word(address) & ~value))
      elif address == controller + 0xA8:  # RXF0A: peripheral acknowledges consumed frame
        status = self.word(controller + 0xA4)
        count = status & 0x7F
        assert count > 0
        self.write_word(controller + 0xA4, (count - 1) | (((value + 1) % 46) << 8))
      elif address == controller + 0xD0:  # TXBAR: record the actual transmitted FIFO contents
        assert value == 1  # configured single-element hardware TX FIFO
        fifo = 0x4000AC00 + bus * 3384 + 46 * 72
        self.transmissions.append((bus, bytes(uc.mem_read(fifo, 72))))

  def power_read(self, uc, access, address, size, value, data):
    if self.power_interrupt is not None and not uc.reg_read(UC_ARM_REG_PRIMASK):
      self.break_next = True

  def nvic_write(self, uc, access, address, size, value, data):
    if 0xE000E100 <= address < 0xE000E120:
      self.enabled[(address - 0xE000E100) // 4] |= value
    elif 0xE000E180 <= address < 0xE000E1A0:
      self.enabled[(address - 0xE000E180) // 4] &= ~value
    if self.can_interrupt is not None and not uc.reg_read(UC_ARM_REG_PRIMASK):
      self.break_next = True

  def nvic_read(self, uc, access, address, size, value, data):
    if 0xE000E100 <= address < 0xE000E120:
      self.write_word(address, self.enabled[(address - 0xE000E100) // 4])
    elif 0xE000E180 <= address < 0xE000E1A0:
      self.write_word(address, self.enabled[(address - 0xE000E180) // 4])

  def call(self, name, *args):
    for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2), args):
      self.uc.reg_write(register, value)
    self.uc.reg_write(UC_ARM_REG_LR, self.STOP | 1)
    self.uc.emu_start(self.address(name) | 1, self.STOP, count=300000)
    if self.break_address is not None:
      boundary = self.break_address
      self.break_address = None
      requested = self.power_interrupt
      can_bus = self.can_interrupt
      self.power_interrupt = None
      self.can_interrupt = None
      context = self.uc.context_save()
      self.uc.reg_write(UC_ARM_REG_SP, 0x2003D000)
      if can_bus is None:
        self.call("set_power_save_state", requested)
      else:
        self.call("llcan_init", self.CANS[can_bus])
      self.uc.context_restore(context)
      self.uc.emu_start(boundary | 1, self.STOP, count=300000)
    if self.uc.reg_read(UC_ARM_REG_PC) != self.STOP:
      raise RuntimeError(f"Instruction limit before {name} returned")

  def reinit_with_power_interrupt(self, bus, requested):
    self.power_interrupt = requested
    self.call("llcan_init", self.CANS[bus])
    if self.power_interrupt is not None:
      # Masked decision defers delivery until return. An older ELF without the
      # power-state read also receives the interrupt after its call returns.
      self.power_interrupt = None
      self.call("set_power_save_state", requested)

  def power_with_reinit_interrupt(self, requested, bus):
    self.can_interrupt = bus
    self.call("set_power_save_state", requested)
    if self.can_interrupt is not None:
      self.can_interrupt = None
      self.call("llcan_init", self.CANS[bus])

  def irq_enabled(self, irq):
    return bool(self.enabled[irq // 32] & (1 << (irq % 32)))

  def can_irqs(self):
    return [[self.irq_enabled(irq) for irq in pair] for pair in self.IRQS]

  def errors(self, bus):
    return self.word(self.address("can_health") + bus * self.health_size + self.error_offset)

  def health(self, bus, name):
    offset, size = self.member(name)
    return self.word(self.address("can_health") + bus * size + offset)

  def deliver_frame(self, bus, address, payload, extended=False, fd=False):
    """Supply one peripheral FIFO element; execute the unmodified receive/forward/TX path."""
    dlc = {8: 8, 64: 15}[len(payload)]
    header = (address if extended else address << 18) | (int(extended) << 30)
    fifo = 0x4000AC00 + bus * 3384
    self.uc.mem_write(fifo, struct.pack("<II", header, (dlc << 16) | (int(fd) << 21)) + payload.ljust(64, b"\0"))
    self.write_word(self.CANS[bus] + 0xA4, 1)  # RXF0S: one element, get index zero
    self.write_word(self.CANS[bus] + 0x50, 1)  # RF0N
    if self.irq_enabled(self.IRQS[bus][0]):
      self.call(f"FDCAN{bus + 1}_IT0_IRQ_Handler")

  def saturate_register_map(self):
    """Fill every legal shadow slot using actual register_set, one initial hash per slot.

    Addresses are RAM-modeled peripherals, masks zero: no invented control logic.
    All power-transition GPIO addresses are absent, forcing the existing bounded
    collision-exhaustion path on every GPIO write, beyond normal boot occupancy.
    """
    self.call("init_registers")
    populated = set()
    for address in range(0x40000000, 0x40080000, 4):
      slot = ((address >> 16) ^ (((address + 1) & 0xFFFF) * 23 & 0xFFFF)) & 0x3FF
      if slot == 1023 or slot in populated:
        continue
      self.call("register_set", address, 0, 0)
      assert self.word(self.address("register_map") + slot * 16) == address
      populated.add(slot)
      if len(populated) == 1023:
        return
    raise AssertionError("Could not populate every legal register shadow slot")

  def deliver_error(self, bus):
    """Model a peripheral protocol error; dispatch only if its NVIC line is enabled."""
    self.write_word(self.CANS[bus] + 0x44, 2)  # PSR LEC = form error
    self.write_word(self.CANS[bus] + 0x40, 127 << 8)  # ECR REC=127, TEC=0
    self.write_word(self.CANS[bus] + 0x50, 1 << 27)  # IR PEA
    if self.irq_enabled(self.IRQS[bus][0]):
      self.call(f"FDCAN{bus + 1}_IT0_IRQ_Handler")

  def close(self):
    self.file.close()
