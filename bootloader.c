#include <stdint.h>
#include "palawan.h"
#include "radio.h"

int updateRx(void);
int updateTx(void);

static int should_enter_bootloader(void) {
  return 1;
}

__attribute__((noreturn))
static void boot_app(void) {
  while (1)
    ;
}

__attribute__((noreturn))
void bootloader_main(void) {

  if (!should_enter_bootloader())
    boot_app();

  if (palawanModel() == palawan_rx)
    /* Start USB */
    updateRx();

  else if (palawanModel() == palawan_tx) {
    updateTx();
  }
  while(1)
    ;
}
