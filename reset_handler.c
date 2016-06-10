#include <stdint.h>
#include "kl17.h"

/* Values exported by the linker */
extern unsigned long _eflash;
extern unsigned long _sdtext;
extern unsigned long _edtext;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long __main_stack_end__;

/* Pointer to the Cortex vector table (located at offset 0) */
extern uint32_t *_vectors;

/* A place for the vector table to live while in RAM */
static uint32_t ram_vectors[64] __attribute__ ((aligned (1024)));

static void init_crt(void) {

  /* Relocate data and text sections to RAM */
  uint32_t *src = &_eflash;
  uint32_t *dest = &_sdtext;
  while (dest < &_edtext)
    *dest++ = *src++;

  /* Clear BSS */
  dest = &_sbss;
  while (dest < &_ebss) *dest++ = 0;

  /* Copy IVT to RAM */
  src = (uint32_t *) &_vectors[0];
  dest = &ram_vectors[0];
  while (dest <= &ram_vectors[63])
    *dest++ = *src++;

  /* Switch to IVT now located in RAM */
  SCB->VTOR = (uint32_t) &ram_vectors[0];
}

extern void __early_init(void);

__attribute__ ((section(".startup"), noreturn))
void Reset_Handler(void) {

  init_crt();
  __early_init();
}
