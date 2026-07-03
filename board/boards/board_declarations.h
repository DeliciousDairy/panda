#pragma once

#include <stdint.h>
#include <stdbool.h>

// ******************** Prototypes ********************
typedef enum {
  BOOT_STANDBY,
  BOOT_BOOTKICK,
  BOOT_RESET,
} BootState;

typedef void (*board_init)(void);
typedef void (*board_init_bootloader)(void);
typedef void (*board_enable_can_transceiver)(uint8_t transceiver, bool enabled);
typedef void (*board_set_can_mode)(uint8_t mode);
typedef uint32_t (*board_read_voltage_mV)(void);
typedef uint32_t (*board_read_current_mA)(void);
typedef void (*board_set_ir_power)(uint8_t percentage);
typedef void (*board_set_fan_enabled)(bool enabled);
typedef void (*board_set_siren)(bool enabled);
typedef void (*board_set_bootkick)(BootState state);
typedef bool (*board_read_som_gpio)(void);
typedef void (*board_set_amp_enabled)(bool enabled);

struct board {
  harness_configuration *harness_config;
  GPIO_TypeDef * const led_GPIO[3];
  const uint8_t led_pin[3];
  const uint8_t led_pwm_channels[3]; // leave at 0 to disable PWM
  const bool has_spi;
  const bool has_fan;
  const uint16_t avdd_mV;
  const uint8_t fan_enable_cooldown_time;
  board_init init;
  board_init_bootloader init_bootloader;
  board_enable_can_transceiver enable_can_transceiver;
  board_set_can_mode set_can_mode;
  board_read_voltage_mV read_voltage_mV;
  board_read_current_mA read_current_mA;
  board_set_ir_power set_ir_power;
  board_set_fan_enabled set_fan_enabled;
  board_set_siren set_siren;
  board_set_bootkick set_bootkick;
  board_read_som_gpio read_som_gpio;
  board_set_amp_enabled set_amp_enabled;
};

// True when this boot came from a real power-on (POR); false on a software
// reset (e.g. a stop-mode wake on bus activity). Set from RCC->RSR in
// cuatro_init. Bootkicking the SOM on every stop-mode wake cold-boots it on
// each of the car's parked background wakes; on Rivian that periodic 12V
// load step trips the parked-security "phantom alarm" (voltage-deviation
// detection). Boards without a SOM never read this; default keeps stock
// behavior.
bool bootkick_on_power_on = true;

// Bootkick diagnostics: persist kick evidence in TAMP backup registers, which
// survive software/pin resets (including openpilot's startup panda reset) and
// are cleared only by backup-domain power loss. Read back via control 0xd9.
//   BKP0R magic, BKP1R boot count, BKP2R RSR at this boot (pre-RMVF),
//   BKP3R last kick reason (1=ignition edge, 2=harness insertion, 3=power-on),
//   BKP4R kick count, BKP5R uptime_cnt at last kick, BKP6R RSR of kick session.
#define BOOTKICK_DIAG_MAGIC 0xB007D1A6U
#ifndef BOOTSTUB
extern uint32_t uptime_cnt;
#endif
static void bootkick_diag_record(uint32_t reason) {
#ifdef STM32H7
  // self-enable: also reachable on boards that never ran bootkick_diag_init
  RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
  PWR->CR1 |= PWR_CR1_DBP;
  RTC->BKP3R = reason;
  RTC->BKP4R += 1U;
#ifdef BOOTSTUB
  RTC->BKP5R = 0U;  // no uptime counter in the bootstub
#else
  RTC->BKP5R = uptime_cnt;
#endif
  RTC->BKP6R = RTC->BKP2R;
#else
  (void)reason;  // no backup domain in simulation builds
#endif
}
static void bootkick_diag_init(uint32_t rsr_raw, bool power_on) {
#ifdef STM32H7
  RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
  PWR->CR1 |= PWR_CR1_DBP;
  if (RTC->BKP0R != BOOTKICK_DIAG_MAGIC) {
    RTC->BKP0R = BOOTKICK_DIAG_MAGIC;
    RTC->BKP1R = 0U;
    RTC->BKP3R = 0U;
    RTC->BKP4R = 0U;
    RTC->BKP5R = 0U;
    RTC->BKP6R = 0U;
    RTC->BKP7R = 0U;  // suppressed-dip counter
  }
  RTC->BKP1R += 1U;
  RTC->BKP2R = rsr_raw;
  if (power_on) {
    bootkick_diag_record(3U);
  }
#else
  (void)rsr_raw;
  (void)power_on;
#endif
}

// ******************* Definitions ********************
// These should match the enums in cereal/log.capnp and __init__.py
#define HW_TYPE_UNKNOWN 0U
#define HW_TYPE_RED_PANDA 7U
#define HW_TYPE_TRES 9U
#define HW_TYPE_CUATRO 10U

// CAN modes
#define CAN_MODE_NORMAL 0U
#define CAN_MODE_OBD_CAN2 1U

extern struct board board_tres;
extern struct board board_cuatro;
extern struct board board_red;
