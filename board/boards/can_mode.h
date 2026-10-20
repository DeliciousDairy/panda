#pragma once

#include "board_declarations.h"
#include "board/sys/sys.h"

// Mux selection must not undo a sleeping board's transceiver policy. Keep the
// policy read and GPIO update atomic against priority-zero power requests, while
// leaving the slower pin-mux setup outside this hardware-only critical section.
static void set_can_mode_transceiver(uint8_t transceiver, bool enabled) {
#ifdef BOOTSTUB
  current_board->enable_can_transceiver(transceiver, enabled);
#else
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  current_board->enable_can_transceiver(transceiver, enabled && !power_save_enabled);
  __set_PRIMASK(primask);
#endif
}
