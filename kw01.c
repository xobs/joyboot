#include "palawan.h"
#include "kl17.h"
#include "kinetis_adc.h"

#define RADIO_REG_DIOMAPPING2   (0x26)
#define RADIO_CLK_DIV1          (0x00)
#define RADIO_CLK_DIV2          (0x01)
#define RADIO_CLK_DIV4          (0x02)
#define RADIO_CLK_DIV8          (0x03)
#define RADIO_CLK_DIV16         (0x04)
#define RADIO_CLK_DIV32         (0x05)
#define RADIO_CLK_RC            (0x06)
#define RADIO_CLK_OFF           (0x07)

#define PAIR_CFG_ADC_NUM        (23)

#define PALAWAN_CFG_RESISTANCE_THRESH 64
#define PALAWAN_RX_VALUE 65536
#define PALAWAN_RX_PAIR_VALUE 0

#define PALAWAN_TX_VALUE_1 9906
#define PALAWAN_TX_VALUE_1_PAIR 1028
#define PALAWAN_TX_VALUE_2 20547
#define PALAWAN_TX_VALUE_2_PAIR 2582
#define PALAWAN_TX_VALUE_3 32787
#define PALAWAN_TX_VALUE_3_PAIR 5442
#define PALAWAN_TX_VALUE_4 65483
#define PALAWAN_TX_VALUE_4_PAIR 65127

static enum palawan_model _model;

void kl1x_clock_init(void);

enum palawan_model palawanModel(void) {
  /* Sample the strapping resistors, to determine model type */
  if ((_model != palawan_tx) && (_model != palawan_rx)) {
    uint16_t gain;
    int resistance;
    int resistance_min;
    int resistance_max;

    /* Ungate PORTE, where the ADC we're using lives */
    SIM->SCGC5 |= SIM_SCGC5_PORTE;

    /* Configure PTE30 to be an input pad */
    PORTE->PCR[30] = PORTx_PCRn_MUX(0);

    /* Ungate the ADC */
    SIM->SCGC6 |= SIM_SCGC6_ADC0;

    /* Quick-and-dirty calibration */
    ADC0->CFG1 =  ADCx_CFG1_ADIV(ADCx_CFG1_ADIV_DIV_8) |
                  ADCx_CFG1_ADICLK(ADCx_CFG1_ADIVCLK_BUS_CLOCK_DIV_2);

    /* Software trigger, no DMA */
    ADC0->SC2 = 0;

    /* Enable hardware averaging over 32 samples, and run calibration */
    ADC0->SC3 = ADCx_SC3_AVGE |
                ADCx_SC3_AVGS(ADCx_SC3_AVGS_AVERAGE_32_SAMPLES) |
                ADCx_SC3_CAL;

    /* Wait for calibration to finish */
    while (!(ADC0->SC1A & ADCx_SC1n_COCO))
      ;

    /* Adjust gain according to reference manual */

    gain = ((ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 +
             ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS) / 2) | 0x8000;
    ADC0->PG = gain;

    gain = ((ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 +
             ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS) / 2) | 0x8000;
    ADC0->MG = gain;

    /* Reset Rn */
    (void)ADC0->RA;

    /* Configure for 16-bit conversion */
    ADC0->CFG1 =  ADCx_CFG1_ADIV(ADCx_CFG1_ADIV_DIV_8) |
                  ADCx_CFG1_ADICLK(ADCx_CFG1_ADIVCLK_BUS_CLOCK_DIV_2) |
                  ADCx_CFG1_MODE(ADCx_CFG1_MODE_16_BITS);

    /* Perform the sample read, averaging over 32 samples */
    ADC0->SC3 = ADCx_SC3_AVGE |
                ADCx_SC3_AVGS(ADCx_SC3_AVGS_AVERAGE_32_SAMPLES);

    /* Begin the read */
    ADC0->SC1A = ADCx_SC1n_ADCH(PAIR_CFG_ADC_NUM);

    /* Wait for sample to finish */
    while (!(ADC0->SC1A & ADCx_SC1n_COCO))
      ;

    resistance = ADC0->RA;

    /* Figure out which threshold it falls into */
    resistance_min = resistance - (PALAWAN_CFG_RESISTANCE_THRESH / 2);
    resistance_max = resistance + (PALAWAN_CFG_RESISTANCE_THRESH / 2);

    if ((resistance_min < PALAWAN_RX_VALUE)
     && (resistance_max > PALAWAN_RX_VALUE))
      _model = palawan_rx;
    else if ((resistance_min < PALAWAN_RX_PAIR_VALUE)
          && (resistance_max > PALAWAN_RX_PAIR_VALUE))
      _model = palawan_rx;

    else if ((resistance_min < PALAWAN_TX_VALUE_1)
          && (resistance_max > PALAWAN_TX_VALUE_1))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_2)
          && (resistance_max > PALAWAN_TX_VALUE_2))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_3)
          && (resistance_max > PALAWAN_TX_VALUE_3))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_4)
          && (resistance_max > PALAWAN_TX_VALUE_4))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_1_PAIR)
          && (resistance_max > PALAWAN_TX_VALUE_1_PAIR))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_2_PAIR)
          && (resistance_max > PALAWAN_TX_VALUE_2_PAIR))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_3_PAIR)
          && (resistance_max > PALAWAN_TX_VALUE_3_PAIR))
      _model = palawan_tx;

    else if ((resistance_min < PALAWAN_TX_VALUE_4_PAIR)
          && (resistance_max > PALAWAN_TX_VALUE_4_PAIR))
      _model = palawan_tx;
  }

    return _model;
}

static void radio_reset(void) {
  if (palawanModel() == palawan_tx)
    GPIOA->PSOR = (1 << 4);
  else if (palawanModel() == palawan_rx)
    GPIOE->PSOR = (1 << 19);
}

static void radio_enable(void) {
  if (palawanModel() == palawan_tx)
    GPIOA->PCOR = (1 << 4);
  else if (palawanModel() == palawan_rx)
    GPIOE->PCOR = (1 << 19);
}

void spi_assert_cs(void) {
  GPIOD->PCOR = (1 << 0);
}

void spi_deassert_cs(void) {
  GPIOD->PSOR = (1 << 0);
}

void spi_read_status(void) {
  (void)SPI0->S;
}

/**
 * @brief   Send a byte and discard the response
 *
 * @notapi
 */
void spi_xmit_byte_sync(uint8_t byte) {
  /* Send the byte */
  SPI0->DL = byte;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPTEF))
    asm("");

  /* Discard the response */
  (void)SPI0->DL;
}

/**
 * @brief   Send a dummy byte and return the response
 *
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t spi_recv_byte_sync(void) {
  /* Send the byte */
  SPI0->DL = 0;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPRF))
    asm("");

  /* Discard the response */
  return SPI0->DL;
}


/**
 * @brief   Read a value from a specified radio register
 *
 * @param[in] addr      radio register to read from
 * @return              value read from said register
 *
 * @notapi
 */
static uint8_t radio_read_register(uint8_t addr) {
  uint8_t val;

  spi_read_status();
  spi_assert_cs();

  spi_xmit_byte_sync(addr);
  spi_recv_byte_sync();
  val = spi_recv_byte_sync();

  spi_deassert_cs();

  return val;
}

/**
 * @brief   Write a value to a specified radio register
 *
 * @param[in] addr      radio register to write to
 * @param[in] val       value to write to said register
 *
 * @notapi
 */
static void radio_write_register(uint8_t addr, uint8_t val) {

  spi_read_status();
  spi_assert_cs();
  /* Send the address to write */
  spi_xmit_byte_sync(addr | 0x80);

  /* Send the actual value */
  spi_xmit_byte_sync(val);

  (void)spi_recv_byte_sync();
  spi_deassert_cs();
}

static void early_usleep(int usec) {
  int j, k;

  for (j = 0; j < usec; j++)
    for (k = 0; k < 30; k++)
        asm("");
}

static void early_msleep(int msec) {
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
static int radio_power_cycle(void) {
  /* @AC RESET sequence from SX1233 datasheet:
   * RESET high Z
   * RESET = 1, then wait 100us
   * RESET = 0 (high Z), then wait 5ms before using the Radio. 
   */

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
static void early_init_radio(void) {

  /* Enable Reset GPIO and SPI PORT clocks by unblocking
   * PORTC, PORTD, and PORTE.*/
  SIM->SCGC5 |= (SIM_SCGC5_PORTA | SIM_SCGC5_PORTC | SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);

  /* Map Reset to a GPIO, which is looped from PTE19 back into
     the RESET_B_XCVR port.*/
  if (palawanModel() == palawan_tx) {
    GPIOA->PDDR |= ((uint32_t) 1 << 4);
    PORTA->PCR[4] = PORTx_PCRn_MUX(1);
  }
  else if (palawanModel() == palawan_rx) {
    GPIOE->PDDR |= ((uint32_t) 1 << 19);
    PORTE->PCR[19] = PORTx_PCRn_MUX(1);
  }

  /* Enable SPI clock.*/
  SIM->SCGC4 |= SIM_SCGC4_SPI0;

  /* Mux PTD0 as a GPIO, since it's used for Chip Select.*/
  GPIOD->PDDR |= ((uint32_t) 1 << 0);
  PORTD->PCR[0] = PORTx_PCRn_MUX(1);

  /* Mux PTC5 as SCK */
  PORTC->PCR[5] = PORTx_PCRn_MUX(2);

  /* Mux PTC6 as MISO */
  PORTC->PCR[6] = PORTx_PCRn_MUX(2);

  /* Mux PTC7 as MOSI */
  PORTC->PCR[7] = PORTx_PCRn_MUX(2);

  /* Keep the radio in reset.*/
  radio_reset();

  /* Initialize the SPI peripheral default values.*/
  SPI0->C1 = 0;
  SPI0->C2 = 0;
  SPI0->BR = 0;

  /* Enable SPI system, and run as a Master.*/
  SPI0->C1 |= (SPIx_C1_SPE | SPIx_C1_MSTR);

  spi_read_status();
  spi_deassert_cs();
}

/**
 * @brief   Configure the radio to output a given clock frequency
 *
 * @param[in] osc_div   a factor of division, of the form 2^n
 *
 * @notapi
 */

/* This optimization fix causes it to read DIOMAPPING2 three times, and
 * for reasons unknown, this actually causes it to stick.
 * Maybe it's a race condition, I'm not really sure.  But if you don't
 * do the readback afterwards, then the clock gets stuck at 1 MHz rather
 * than 8 MHz, and bad things happen.
 */
#define DIOVAL2_OPTIMIZATION_FIX
static void radio_configure_clko(uint8_t osc_div) {
#ifdef DIOVAL2_OPTIMIZATION_FIX
  uint8_t dioval2 = radio_read_register(RADIO_REG_DIOMAPPING2);
  radio_write_register(RADIO_REG_DIOMAPPING2, (dioval2 & ~7) | osc_div);
  dioval2 = radio_read_register(RADIO_REG_DIOMAPPING2);
  radio_write_register(RADIO_REG_DIOMAPPING2, (dioval2 & ~7) | osc_div);
  (void)radio_read_register(RADIO_REG_DIOMAPPING2);
#else
  radio_write_register(RADIO_REG_DIOMAPPING2,
      (radio_read_register(RADIO_REG_DIOMAPPING2) & ~7) | osc_div);
#endif
}

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {

  switch (palawanModel()) {
    case palawan_tx:
      break;
      
    case palawan_rx:
      break;
      
    default:
      asm("bkpt #0");
      break;
  }

  early_init_radio();
  radio_power_cycle();

  /* 32Mhz/4 = 8 MHz CLKOUT.*/
  radio_configure_clko(RADIO_CLK_DIV1);

  kl1x_clock_init();
}
