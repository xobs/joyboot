#include <stdint.h>
#include "kl17.h"
#include "core_cm0plus.h"
#include "palawan.h"
#include "palawan_bl.h"
#include "radio.h"

enum bootloader_reason bootloader_reason;

int updateRx(void);
int updateTx(void);

struct boot_token {
  uint32_t magic;
  uint32_t boot_count;
};

__attribute__((section("boot_token"))) extern struct boot_token boot_token;
static __attribute__ ((section(".appvectors"))) uint32_t appVectors[64];

static int test_boot_token(void) {
  /*
   * If we find a valid boot token in RAM, the application is asking us explicitly
   * to enter DFU mode. This is used to implement the DFU_DETACH command when the app
   * is running.
   */

  return boot_token.magic == 0x74624346;
}

static int button_held_down(void) {
  int before;
  int after;

  /* Set PTB12 as a GPIO input, slow slew rate */
  PORTB->PCR[12] = (1 << 8) | (1 << 2);

  /* Set it as an input */
  FGPIOB->PDDR &= ~(1 << 12);

  /* If the pin isn't held down, don't enter the bootloader */
  before = !(FGPIOB->PDIR & (1 << 12));
  if (!before)
    return 0;

  /* Sample the pin before, then wait about 500ms and sample again */
  {
    int i;
    for (i = 0; i < 500000; i++) {
      int j;
      for (j = 0; j < 77; j++) {
        asm("");
      }
    }
  }
  after = !(FGPIOB->PDIR & (1 << 12));

  if (!after)
    return 0;

  return 1;
}

static int should_enter_bootloader(void) {
  extern uint32_t __ram_start__;
  extern uint32_t __ram_end__;
  extern uint32_t __app_start__;
  extern uint32_t __app_end__;

  /* Reset the boot token if we've just been powered up for the first time */
  if (RCM->SRS0 & RCM_SRS0_POR) {
    boot_token.magic = 0;
    boot_token.boot_count = 0;
  }

  /* If the special magic number is present, enter the bootloader */
  if (test_boot_token()) {
    bootloader_reason = BOOT_TOKEN_PRESENT;
    return 1;
  }

  /* If the user is holding the button down */
  if (button_held_down()) {
    bootloader_reason = BUTTON_HELD_DOWN;
    return 1;
  }

  /* If we've failed to boot many times, enter the bootloader */
  if (boot_token.boot_count > 3) {
    bootloader_reason = BOOT_FAILED_TOO_MANY_TIMES;
    return 1;
  }

  /* Otherwise, if the application appears valid (i.e. stack is in a sane
   * place, and the program counter is in flash,) boot to it */
  if (((appVectors[0] >= (uint32_t)&__ram_start__) && (appVectors[0] <= (uint32_t)&__ram_end__))
   && ((appVectors[1] >= (uint32_t)&__app_start__) && (appVectors[1] <= (uint32_t)&__app_end__))) {
     bootloader_reason = NOT_ENTERING_BOOTLOADER;
    return 0;
   }

  /* If there is no valid program, enter the bootloader */
  bootloader_reason = NO_PROGRAM_PRESENT;
  return 1;
}

__attribute__((noreturn))
static void boot_app(void) {
  // Relocate IVT to application flash
  __disable_irq();
  SCB->VTOR = (uint32_t) &appVectors[0];

  // Switch the clock mode from FEE back to FEI
  MCG->C1 = /* Clear the IREFS bit to switch to the external reference. */
          MCG_C1_CLKS_FLLPLL |  /* Use FLL for system clock, MCGCLKOUT. */
          MCG_C1_IRCLKEN     |  /* Enable the internal reference clock. */
          MCG_C1_IREFS;         /* Use the internal reference clock. */
  MCG->C6 = 0;  /* PLLS=0: Select FLL as MCG source, not PLL */

  // Refresh watchdog right before launching app
  watchdog_refresh();

  // Clear the boot token, so we don't repeatedly enter DFU mode.
  boot_token.magic = 0;
  boot_token.boot_count++;

  asm volatile (
  "mov lr, %0 \n\t"
  "mov sp, %1 \n\t"
  "bx %2 \n\t"
  : : "r" (0xFFFFFFFF),
  "r" (appVectors[0]),
  "r" (appVectors[1]) );
  while (1)
    ;
}

__attribute__((noreturn))
void bootloader_main(void) {

  if (should_enter_bootloader()) {
    boot_token.magic = 0;
    boot_token.boot_count = 0;

    if (palawanModel() == palawan_rx)
      /* Start USB */
      updateRx();

    else if (palawanModel() == palawan_tx)
      updateTx();
  }

  boot_app();
}
