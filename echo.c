#include "radio.h"
int echoBuffer(void *buffer, uint8_t count) {
  radioSend(radioDevice, 0xff, radio_prot_echo, count, buffer);

  int timeout_ms = 10;
  int ms = 0;
  while (ms < timeout_ms) {
		radioPoll(radioDevice);
		int i;
		for (i = 0; i < 5000; i++)
			asm("nop");
    ms++;
  }
}
