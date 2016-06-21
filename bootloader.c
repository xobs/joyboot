#include <stdint.h>
#include "kl17.h"
#include "core_cm0plus.h"
#include "palawan.h"
#include "radio.h"

int updateRx(void);
int updateTx(void);

extern uint32_t boot_token;
static __attribute__ ((section(".appvectors"))) uint32_t appVectors[64];

static int test_boot_token()
{
  /*
   * If we find a valid boot token in RAM, the application is asking us explicitly
   * to enter DFU mode. This is used to implement the DFU_DETACH command when the app
   * is running.
   */

  return boot_token == 0x74624346;
}


static int should_enter_bootloader(void) {
  return 1;
}

__attribute__((noreturn))
static void boot_app(void) {
  // Relocate IVT to application flash
  __disable_irq();
  SCB->VTOR = (uint32_t) &appVectors[0];

  // Refresh watchdog right before launching app
  watchdog_refresh();

  // Clear the boot token, so we don't repeatedly enter DFU mode.
  boot_token = 0;

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

    if (palawanModel() == palawan_rx)
      /* Start USB */
      updateRx();

    else if (palawanModel() == palawan_tx)
      updateTx();
  }

  boot_app();
}
