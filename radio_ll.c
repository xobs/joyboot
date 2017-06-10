#include <stdint.h>

#include "palawan.h"
#include "kl17.h"
#include "spi.h"

#define RADIO_REG_DIOMAPPING2 (0x26)
#define RADIO_CLK_DIV1 (0x00)
#define RADIO_CLK_DIV2 (0x01)
#define RADIO_CLK_DIV4 (0x02)
#define RADIO_CLK_DIV8 (0x03)
#define RADIO_CLK_DIV16 (0x04)
#define RADIO_CLK_DIV32 (0x05)
#define RADIO_CLK_RC (0x06)
#define RADIO_CLK_OFF (0x07)

static void radio_reset(void)
{
  GPIOB->PSOR = (1 << 11);
}

static void radio_enable(void)
{
  GPIOB->PCOR = (1 << 11);
}

static void early_usleep(int usec)
{
  int j, k;

  for (j = 0; j < usec; j++)
    for (k = 0; k < 30; k++)
      asm("");
}

static void early_msleep(int msec)
{
  int i, j, k;

  for (i = 0; i < msec; i++)
    for (j = 0; j < 1000; j++)
      for (k = 0; k < 30; k++)
        asm("");
}

/**
 * @brief   Power cycle the radio
 * @details Put the radio into reset, then wait 100 us.  Bring it out of
 *          reset, then wait 5 ms.  Ish.
 */
int radioPowerCycle(void)
{
  /* Mux the reset GPIO */
  GPIOB->PDDR |= ((uint32_t)1 << 11);
  PORTB->PCR[11] = PORTx_PCRn_MUX(1);

  radio_reset();

  /* Cheesy usleep(100).*/
  early_usleep(100);

  radio_enable();

  /* Cheesy msleep(10).*/
  early_msleep(10);

  return 1;
}

/**
 * @brief   Set up mux ports, enable SPI, and set up GPIO.
 * @details The MCU communicates to the radio via SPI and a few GPIO lines.
 *          Set up the pinmux for these GPIO lines, and set up SPI.
 */
void earlyInitRadio(void)
{

  /* Enable Reset GPIO and SPI PORT clocks by unblocking
   * PORTC, PORTD, and PORTE.*/
  SIM->SCGC5 |= (SIM_SCGC5_PORTA | SIM_SCGC5_PORTB);

  /* Map Reset to a GPIO, which is looped from PTE19 back into
     the RESET_B_XCVR port.*/
  GPIOB->PDDR |= ((uint32_t)1 << 11);
  PORTB->PCR[11] = PORTx_PCRn_MUX(1);

  /* Mux PTA12 as DIO0, to receive packets.
   * Enable the interrupt, and mux it as a slow slew rate.
   */
  PORTA->PCR[12] = PORTx_PCRn_MUX(1) | (1 << 2) | (0x9 << 16);
  GPIOA->PDDR &= ~(1 << 12);

  /* Mux PTB2 as DIO1, useful for knowing when the FIFO is empty.
   */
  PORTB->PCR[2] = PORTx_PCRn_MUX(1) | (1 << 2);
  GPIOB->PDDR &= ~(1 << 2);

  /* Keep the radio in reset.*/
  radio_reset();

  spiReadStatus();
  spiDeassertCs();
}

void radioInit(void)
{

  radioPowerCycle();
}
