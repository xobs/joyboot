#include "palawan.h"
#include "kl17.h"

void spiAssertCs(void)
{
  GPIOA->PCOR = (1 << 5);
}

void spiDeassertCs(void)
{
  GPIOA->PSOR = (1 << 5);
}

void spiReadStatus(void)
{
  (void)SPI0->S;
}

/**
 * @brief   Send a byte and discard the response
 *
 * @notapi
 */
void spiXmitByteSync(uint8_t byte)
{
  /* Send the byte */
  SPI0->D = byte;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPTEF))
    asm("");

  /* Discard the response */
  (void)SPI0->D;
}

/**
 * @brief   Send a dummy byte and return the response
 *
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t spiRecvByteSync(void)
{
  /* Send the byte */
  SPI0->D = 0;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPRF))
    asm("");

  /* Discard the response */
  return SPI0->D;
}

void spiInit(void)
{
  /* Enable SPI clock.*/
  SIM->SCGC4 |= SIM_SCGC4_SPI0;

  /* Mux PTA5 as a GPIO, since it's used for Chip Select.*/
  GPIOA->PDDR |= ((uint32_t)1 << 5);
  PORTA->PCR[5] = PORTx_PCRn_MUX(1);

  /* Mux PTB0 as SCK */
  PORTB->PCR[0] = PORTx_PCRn_MUX(3);

  /* Mux PTA6 as MISO */
  PORTA->PCR[6] = PORTx_PCRn_MUX(3);

  /* Mux PTA7 as MOSI */
  PORTA->PCR[7] = PORTx_PCRn_MUX(3);

  /* Initialize the SPI peripheral default values.*/
  SPI0->C1 = 0;
  SPI0->C2 = 0;
  SPI0->BR = 0;

  /* Enable SPI system, and run as a Master.*/
  SPI0->C1 |= (SPIx_C1_SPE | SPIx_C1_MSTR);
}
