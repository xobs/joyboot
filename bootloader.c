#include <stdint.h>
#include "palawan.h"
#include "radio.h"

void bootloader_main(void) {

  if (palawanModel() == palawan_rx) {

    radioStart(radioDevice);
  }
  else if (palawanModel() == palawan_tx) {
    /* Start USB */
  }
  while(1)
    ;
}
