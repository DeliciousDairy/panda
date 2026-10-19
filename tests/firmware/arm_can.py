"""Actual H7 instructions with RAM peripherals and NVIC enable-bit semantics.

The loader/call boundary reuses the retained Codex panda_register_probe design.
Only serial output and external peripheral behavior are substituted. This is not
electrical CAN, wake-edge, interrupt-latency or vehicle validation.
"""
import struct
from pathlib import Path

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB, UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2


class Firmware:
  STOP = 0x08001000
  CANS = (0x4000A000, 0x4000A400, 0x4000D400)
  IRQS = ((19, 21), (20, 22), (159, 160))

  def __init__(self, path, board="cuatro", orientation=1):
    self.file = Path(path).open("rb")
    self.elf = ELFFile(self.file)
    self.symbols = {s.name: s for s in self.elf.get_section_by_name(".symtab").iter_symbols()}
    self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for address, length in [(0x08000000, 0x200000), (0x20000000, 0x40000),
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
    self.uc.hook_add(UC_HOOK_CODE, self.code)
    self.uc.hook_add(UC_HOOK_MEM_WRITE, self.nvic_write, begin=0xE000E100, end=0xE000E19F)
    self.uc.hook_add(UC_HOOK_MEM_READ, self.nvic_read, begin=0xE000E100, end=0xE000E19F)
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
    if address in self.skip:
      uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

  def nvic_write(self, uc, access, address, size, value, data):
    if 0xE000E100 <= address < 0xE000E120:
      self.enabled[(address - 0xE000E100) // 4] |= value
    elif 0xE000E180 <= address < 0xE000E1A0:
      self.enabled[(address - 0xE000E180) // 4] &= ~value

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
    if self.uc.reg_read(UC_ARM_REG_PC) != self.STOP:
      raise RuntimeError(f"Instruction limit before {name} returned")

  def irq_enabled(self, irq):
    return bool(self.enabled[irq // 32] & (1 << (irq % 32)))

  def can_irqs(self):
    return [[self.irq_enabled(irq) for irq in pair] for pair in self.IRQS]

  def errors(self, bus):
    return self.word(self.address("can_health") + bus * self.health_size + self.error_offset)

  def deliver_error(self, bus):
    """Model a peripheral protocol error; dispatch only if its NVIC line is enabled."""
    self.write_word(self.CANS[bus] + 0x44, 2)  # PSR LEC = form error
    self.write_word(self.CANS[bus] + 0x40, 127 << 8)  # ECR REC=127, TEC=0
    self.write_word(self.CANS[bus] + 0x50, 1 << 27)  # IR PEA
    if self.irq_enabled(self.IRQS[bus][0]):
      self.call(f"FDCAN{bus + 1}_IT0_IRQ_Handler")

  def close(self):
    self.file.close()
